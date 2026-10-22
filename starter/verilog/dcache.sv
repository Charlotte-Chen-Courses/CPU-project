/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  dcache.sv                                           //
//                                                                     //
//  Description :  Data cache. DC_SZ bytes, DC_WAYS-way set            //
//                 associative, 8-byte blocks (one mem.sv bus beat),   //
//                 write-back + write-allocate, LRU replacement,       //
//                 blocking (one miss at a time).                      //
//                                                                     //
//                 CPU side (for the LSQ): request valid/ready with    //
//                 store, addr, size, data (low bytes), id; loads get  //
//                 a response (valid, id, the aligned 32-bit word that //
//                 holds addr). ready depends only on state. Accesses  //
//                 must be naturally aligned and in memory range (the  //
//                 LSQ checks this before sending).                    //
//                                                                     //
//                 Hit: a load answers next cycle; a store merges its  //
//                 bytes into the block and sets dirty. Miss: if the   //
//                 victim (invalid way first, else LRU) is dirty it is //
//                 written back (BUS_STORE; mem.sv never acknowledges  //
//                 stores, so the fill starts as soon as the writeback //
//                 is accepted); then BUS_LOAD the block, capture the  //
//                 memory tag in the cycle it is accepted, wait for    //
//                 data with that tag, install it and finish the       //
//                 request (answer the load / merge the store).        //
//                                                                     //
//                 Memory side is P3's mem.sv protocol (CACHE_MODE:    //
//                 8-byte aligned blocks, response tag 0 = not         //
//                 accepted, keep asking).                             //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module dcache #(
    localparam WAYS  = `DC_WAYS,
    localparam SETS  = `DC_SZ / (`DC_BLK * `DC_WAYS),
    localparam IDX_W = $clog2(SETS),
    localparam A_W   = $clog2(`MEM_SIZE_IN_BYTES),     // physical address bits used
    localparam TAG_W = A_W - IDX_W - 3,
    localparam AGE_W = (WAYS > 1) ? $clog2(WAYS) : 1,
    localparam ID_W  = `DMEM_ID_W
) (
    input                          clock,
    input                          reset,

    // CPU side
    input                          req_valid,
    output logic                   req_ready,
    input                          req_store,
    input        [`XLEN-1:0]       req_addr,
    input  MEM_SIZE                req_size,
    input        [`XLEN-1:0]       req_data,
    input        [ID_W-1:0]        req_id,
    output logic                   resp_valid,
    output logic [ID_W-1:0]        resp_id,
    output logic [`XLEN-1:0]       resp_data,

    // memory side (mem.sv)
    output logic [1:0]             mem_command,
    output logic [`XLEN-1:0]       mem_addr,
    output logic [63:0]            mem_data,
    input        [3:0]             mem_response,
    input        [63:0]            mem_rdata,
    input        [3:0]             mem_tag
);

  typedef struct packed {
    logic             valid;
    logic             dirty;
    logic [TAG_W-1:0] tag;
    logic [AGE_W-1:0] age;     // 0 = most recently used
    logic [63:0]      data;
  } DC_LINE;

  DC_LINE lines [SETS][WAYS];

  typedef enum logic [1:0] { IDLE, WB, FILL_REQ, FILL_WAIT } DC_STATE;
  DC_STATE state;

  // the request being serviced by a miss
  logic             m_store;
  logic [A_W-1:0]   m_addr;
  MEM_SIZE          m_size;
  logic [`XLEN-1:0] m_data;
  logic [ID_W-1:0]  m_id;
  logic [AGE_W-1:0] m_way;      // way being refilled
  logic [3:0]       m_memtag;   // memory tag of the outstanding fill

  // statistics (simulation only, read by testbenches)
  /* verilator lint_off PROCASSINIT */
  int n_hits = 0, n_misses = 0, n_writebacks = 0;
  /* verilator lint_on PROCASSINIT */

  // ---------------- helpers ----------------
  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic [IDX_W-1:0] idx_of(input logic [A_W-1:0] a); return a[IDX_W+2:3]; endfunction
  function automatic logic [TAG_W-1:0] tag_of(input logic [A_W-1:0] a); return a[A_W-1:IDX_W+3]; endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // write a store's bytes into a block
  function automatic logic [63:0] merge(input logic [63:0] blk, input logic [2:0] off, input MEM_SIZE s,
                                        input logic [`XLEN-1:0] d);
    automatic int n = (s == BYTE) ? 1 : (s == HALF) ? 2 : 4;
    for (int k = 0; k < 4; k++)   // constant bound: synthesis must unroll it
      if (k < n) blk[8 * (int'(off) + k) +: 8] = d[8 * k +: 8];
    return blk;
  endfunction

  function automatic logic [`XLEN-1:0] word_of(input logic [63:0] blk, input logic off2);
    return off2 ? blk[63:32] : blk[31:0];
  endfunction

  // ---------------- lookup (incoming request) ----------------
  logic [A_W-1:0]   r_addr;
  logic [IDX_W-1:0] r_idx;
  logic             r_hit;
  logic [AGE_W-1:0] r_way, r_victim;
  assign r_addr = req_addr[A_W-1:0];
  assign r_idx  = idx_of(r_addr);

  always_comb begin
    r_hit    = `FALSE;
    r_way    = '0;
    r_victim = '0;
    for (int w = 0; w < WAYS; w++)
      if (lines[r_idx][w].valid && lines[r_idx][w].tag == tag_of(r_addr)) begin
        r_hit = `TRUE;
        r_way = AGE_W'(w);
      end
    // victim: an invalid way if there is one, else the least recently used
    begin
      automatic bit found_inv = 0;
      automatic logic [AGE_W-1:0] oldest = '0;
      for (int w = 0; w < WAYS; w++) begin
        if (!lines[r_idx][w].valid && !found_inv) begin found_inv = 1; r_victim = AGE_W'(w); end
        if (lines[r_idx][w].age >= oldest && !found_inv) begin oldest = lines[r_idx][w].age; r_victim = AGE_W'(w); end
      end
    end
  end

  assign req_ready = (state == IDLE);

  // ---------------- memory side ----------------
  logic [IDX_W-1:0] m_idx;
  assign m_idx = idx_of(m_addr);

  always_comb begin
    mem_command = BUS_NONE;
    mem_addr    = '0;
    mem_data    = '0;
    case (state)
      WB: begin
        mem_command = BUS_STORE;
        mem_addr    = `XLEN'({lines[m_idx][m_way].tag, m_idx, 3'b000});
        mem_data    = lines[m_idx][m_way].data;
      end
      FILL_REQ: begin
        mem_command = BUS_LOAD;
        mem_addr    = `XLEN'({m_addr[A_W-1:3], 3'b000});
      end
      default: ;
    endcase
  end

  // ---------------- state ----------------
  // mark way w of set s as most recently used
  task automatic touch(input logic [IDX_W-1:0] s, input logic [AGE_W-1:0] w);
    for (int v = 0; v < WAYS; v++)
      if (AGE_W'(v) == w) lines[s][v].age <= '0;
      else if (lines[s][v].age < lines[s][w].age) lines[s][v].age <= lines[s][v].age + 1'b1;
  endtask

  always_ff @(posedge clock) begin
    if (reset) begin
      state      <= IDLE;
      resp_valid <= `FALSE;
      for (int s = 0; s < SETS; s++)
        for (int w = 0; w < WAYS; w++) begin
          lines[s][w].valid <= `FALSE;
          lines[s][w].dirty <= `FALSE;
          lines[s][w].age   <= AGE_W'(w);   // distinct ages: a valid LRU order from the start
        end
    end else begin
      resp_valid <= `FALSE;
      case (state)
        IDLE:
          if (req_valid) begin
            if (r_hit) begin
              touch(r_idx, r_way);
              if (req_store) begin
                lines[r_idx][r_way].data  <= merge(lines[r_idx][r_way].data, r_addr[2:0], req_size, req_data);
                lines[r_idx][r_way].dirty <= `TRUE;
              end else begin
                resp_valid <= `TRUE;
                resp_id    <= req_id;
                resp_data  <= word_of(lines[r_idx][r_way].data, r_addr[2]);
              end
            end else begin
              m_store <= req_store;
              m_addr  <= r_addr;
              m_size  <= req_size;
              m_data  <= req_data;
              m_id    <= req_id;
              m_way   <= r_victim;
              state   <= (lines[r_idx][r_victim].valid && lines[r_idx][r_victim].dirty) ? WB : FILL_REQ;
            end
          end
        WB:
          if (mem_response != 4'd0) begin       // writeback accepted (stores are never answered)
            state <= FILL_REQ;
          end
        FILL_REQ:
          if (mem_response != 4'd0) begin
            m_memtag <= mem_response;
            state    <= FILL_WAIT;
          end
        FILL_WAIT:
          if (mem_tag == m_memtag && mem_tag != 4'd0) begin
            lines[m_idx][m_way].valid <= `TRUE;
            lines[m_idx][m_way].tag   <= tag_of(m_addr);
            touch(m_idx, m_way);
            if (m_store) begin
              lines[m_idx][m_way].data  <= merge(mem_rdata, m_addr[2:0], m_size, m_data);
              lines[m_idx][m_way].dirty <= `TRUE;
            end else begin
              lines[m_idx][m_way].data  <= mem_rdata;
              lines[m_idx][m_way].dirty <= `FALSE;
              resp_valid <= `TRUE;
              resp_id    <= m_id;
              resp_data  <= word_of(mem_rdata, m_addr[2]);
            end
            state <= IDLE;
          end
        default: state <= IDLE;
      endcase
    end
  end

  // synopsys translate_off
  /* verilator lint_off BLKSEQ */
  always @(posedge clock)
    if (!reset) begin
      if (state == IDLE && req_valid) begin if (r_hit) n_hits++; else n_misses++; end
      if (state == WB && mem_response != 4'd0) n_writebacks++;
    end
  /* verilator lint_on BLKSEQ */

  always_ff @(posedge clock)
    if (!reset && state == IDLE && req_valid)
      assert (req_addr < `MEM_SIZE_IN_BYTES &&
              !(req_size == HALF && req_addr[0]) && !(req_size == WORD && req_addr[1:0] != 2'b00) &&
              req_size != DOUBLE)
      else $error("dcache: misaligned or out-of-range request %h (size %0d)", req_addr, req_size);
  // synopsys translate_on

endmodule  // dcache
