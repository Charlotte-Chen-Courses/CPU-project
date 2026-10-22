/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  lsq.sv                                              //
//                                                                     //
//  Description :  Load/store queue, conservative version (no load     //
//                 speculation yet). A load queue (LQ) and a store     //
//                 queue (SQ), both circular, allocated in program     //
//                 order at dispatch (lsq_idx goes into the RS         //
//                 packet; a load records the SQ tail = its position   //
//                 among the stores).                                  //
//                                                                     //
//                 Address generation: the FU_LOAD / FU_STORE issue    //
//                 ports write rs1 + imm (and, for stores, rs2) into   //
//                 the entry. Misaligned or out-of-range addresses get //
//                 an exception and never reach memory. A store        //
//                 completes on its port's CDB output (tag p0).        //
//                                                                     //
//                 Loads (LSQ_SPECULATE = 0): a load may proceed only  //
//                 when every older store's address is known.          //
//                 LSQ_SPECULATE = 1: unknown older store addresses    //
//                 do not block; when a store gets its address, any    //
//                 younger overlapping load that already read a value  //
//                 is a violation (unless it forwarded from a store    //
//                 younger than this one), and the oldest one is       //
//                 reported (viol_*) for the ROB to replay at commit.  //
//                 A load given with ld_wait (the load-wait table's    //
//                 prediction, LSQ_SPECULATE = 2) waits as in 0.       //
//                 Either way, if the youngest KNOWN older store that  //
//                 overlaps a load covers all its bytes, the           //
//                 value is forwarded; on a partial overlap the load   //
//                 waits until that store has drained; with no overlap //
//                 it is sent to memory (oldest first). Results go out //
//                 on the FU_LOAD port's CDB output, oldest first.     //
//                                                                     //
//                 Stores: written to memory only after they commit,   //
//                 in order, from the SQ head (one request per cycle;  //
//                 loads have priority). SQ pointers: head (next to    //
//                 drain), cmt (first uncommitted), tail.              //
//                                                                     //
//                 Flush: everything uncommitted goes (LQ tail = head, //
//                 SQ tail = cmt, both after this cycle's commits);    //
//                 committed stores keep draining. Each load request   //
//                 carries a sequence number; a response is accepted   //
//                 only by an issued entry with the same number, so a  //
//                 response for a squashed load is dropped.            //
//                                                                     //
//                 Atomics (A: LR / SC / AMO) come on the store port   //
//                 and are serial (issued as the oldest instruction,   //
//                 nothing younger dispatched), so they need no LQ/SQ  //
//                 entry and may write memory at once: check the       //
//                 address (rs1; misaligned / out of range -> the      //
//                 exception, memory untouched), wait for committed    //
//                 stores to drain, read, compute, write, and return   //
//                 the old value (SC: 0 = success, 1 = fail) on the    //
//                 store port's CDB output. LR sets the reservation    //
//                 (one word); an SC clears it, and so does any store  //
//                 written to that word (store, SC or AMO).            //
//                                                                     //
//                 Memory interface (dcache.sv's CPU side): request    //
//                 (valid/ready), response for loads only (valid, id,  //
//                 the aligned 32-bit word holding the address), any   //
//                 latency, any order. Memory must make an accepted    //
//                 store visible to every load accepted after it.      //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module lsq #(
    localparam W     = `SUPERSCALAR_WIDTH,
    localparam LQ_SZ = `LQ_SZ,
    localparam SQ_SZ = `SQ_SZ,
    localparam LQP   = $clog2(LQ_SZ),
    localparam SQP   = $clog2(SQ_SZ),
    localparam IDW   = `LSQ_IDX_W,
    localparam ID_W  = `DMEM_ID_W
) (
    input                               clock,
    input                               reset,
    input                               flush,

    // dispatch (already gated by fire), in slot order
    input        [W-1:0]                disp_load,
    input        [W-1:0]                disp_store,
    output logic [W-1:0][IDW-1:0]       disp_lsq_idx,   // LQ index for a load, SQ index for a store
    output logic                        lq_space_ok,    // >= W free entries (state only)
    output logic                        sq_space_ok,

    // FU_LOAD port: address in, load results out
    input                               ld_in_valid,
    input  RS_PACKET                    ld_in_pkt,
    input        [`XLEN-1:0]            ld_rs1,
    input                               ld_wait,        // predicted to conflict: wait for older store addresses
    output logic                        ld_ready,
    output logic                        ld_out_valid,
    output CDB_PACKET                   ld_out_pkt,
    input                               ld_out_grant,

    // FU_STORE port: address + data in, completion out
    input                               st_in_valid,
    input  RS_PACKET                    st_in_pkt,
    input        [`XLEN-1:0]            st_rs1,
    input        [`XLEN-1:0]            st_rs2,
    output logic                        st_ready,
    output logic                        st_out_valid,
    output CDB_PACKET                   st_out_pkt,
    input                               st_out_grant,

    // commit (from the ROB, slot order; only non-excepting loads/stores)
    input        [W-1:0]                commit_load,
    input        [W-1:0]                commit_store,

    // data memory (D-cache)
    output logic                        dmem_req_valid,
    input                               dmem_req_ready,
    output logic                        dmem_req_store,
    output logic [`XLEN-1:0]            dmem_req_addr,
    output MEM_SIZE                     dmem_req_size,
    output logic [`XLEN-1:0]            dmem_req_data,  // store data in the low bytes
    output logic [ID_W-1:0]             dmem_req_id,
    input                               dmem_resp_valid,
    input        [ID_W-1:0]             dmem_resp_id,
    input        [`XLEN-1:0]            dmem_resp_data, // aligned word containing the address

    output logic                        stores_pending, // committed stores not yet written

    // memory-order violation: a store got its address and an overlapping younger load had
    // already read memory (or forwarded): the oldest such load must be re-run (ROB replay)
    output logic                        viol_valid,
    output logic [`ROB_IDX_W-1:0]       viol_rob_idx
);

  typedef struct packed {
    logic                  addr_v;     // address generated
    logic [`XLEN-1:0]      addr;
    MEM_SIZE               size;
    logic                  uns;        // lbu / lhu
    logic [`TAG_W-1:0]     tag;
    logic [`ROB_IDX_W-1:0] rob_idx;
    logic [SQP:0]          sq_pos;     // SQ tail at dispatch: stores before it are older
    logic                  issued;     // request sent to memory
    logic [ID_W-1:0]       req_id;
    logic                  done;       // value (or exception) known
    logic                  bcast;      // moved to the output register
    logic [`XLEN-1:0]      value;
    EXCEPTION              exc;
    logic                  wait_st;    // wait for every older store address even when speculating
    logic                  fwd_v;      // value forwarded from the store at SQ position fwd_pos
    logic [SQP:0]          fwd_pos;
  } LQ_ENTRY;

  typedef struct packed {
    logic                  addr_v;     // address + data known
    logic [`XLEN-1:0]      addr;
    MEM_SIZE               size;
    logic [`XLEN-1:0]      data;
    EXCEPTION              exc;
  } SQ_ENTRY;

  LQ_ENTRY lq [LQ_SZ];
  SQ_ENTRY sq [SQ_SZ];
  logic [LQP:0] lq_head, lq_tail;          // one extra wrap bit
  logic [SQP:0] sq_head, sq_cmt, sq_tail;
  logic [ID_W-1:0] next_id;

  // ---------------- helpers ----------------
  function automatic int nbytes(input MEM_SIZE s);
    return (s == BYTE) ? 1 : (s == HALF) ? 2 : 4;
  endfunction

  function automatic logic [`XLEN-1:0] extend(input logic [`XLEN-1:0] raw, input MEM_SIZE s, input logic uns);
    case (s)
      BYTE:    return uns ? {24'b0, raw[7:0]}  : {{24{raw[7]}},  raw[7:0]};
      HALF:    return uns ? {16'b0, raw[15:0]} : {{16{raw[15]}}, raw[15:0]};
      default: return raw;
    endcase
  endfunction

  function automatic EXCEPTION addr_check(input logic [`XLEN-1:0] a, input MEM_SIZE s);
    if ((s == HALF && a[0]) || (s == WORD && a[1:0] != 2'b00) || s == DOUBLE) return EXC_MISALIGNED;
    if (a >= `MEM_SIZE_IN_BYTES) return EXC_ACCESS;   // aligned, so the whole access is in range
    return EXC_NONE;
  endfunction

  // ---------------- occupancy ----------------
  logic [LQP:0] lq_count;
  logic [SQP:0] sq_count;
  assign lq_count       = lq_tail - lq_head;
  assign sq_count       = sq_tail - sq_head;
  assign lq_space_ok    = (int'(LQ_SZ) - int'(lq_count)) >= W;
  assign sq_space_ok    = (int'(SQ_SZ) - int'(sq_count)) >= W;
  assign stores_pending = (sq_head != sq_cmt);

  // ---------------- dispatch: positions ----------------
  logic [W-1:0][SQP:0] disp_sq_pos;
  int n_dl, n_ds;
  always_comb begin
    n_dl = 0; n_ds = 0;
    for (int i = 0; i < W; i++) begin
      disp_sq_pos[i]  = sq_tail + (SQP+1)'(n_ds);   // stores in older slots are older
      disp_lsq_idx[i] = disp_load[i] ? IDW'((int'(lq_tail[LQP-1:0]) + n_dl) % LQ_SZ)
                                     : IDW'((int'(sq_tail[SQP-1:0]) + n_ds) % SQ_SZ);
      n_dl += int'(disp_load[i]);
      n_ds += int'(disp_store[i]);
    end
  end

  // ---------------- address generation ----------------
  logic [`XLEN-1:0] ld_addr, st_addr;
  MEM_SIZE          ld_size, st_size;
  assign ld_addr  = ld_rs1 + `RV32_signext_Iimm(ld_in_pkt.inst);
  assign st_addr  = st_rs1 + `RV32_signext_Simm(st_in_pkt.inst);
  assign ld_size  = MEM_SIZE'(ld_in_pkt.inst[13:12]);
  assign st_size  = MEM_SIZE'(st_in_pkt.inst[13:12]);
  assign ld_ready = `TRUE;                          // the LQ entry already exists
  assign st_ready = (!st_out_valid || st_out_grant) && at_state == AT_IDLE;  // completion register free, no atomic

  // ---------------- per-load disambiguation ----------------
  logic [LQ_SZ-1:0]             lq_valid, can_mem, can_fwd;
  logic [LQ_SZ-1:0][`XLEN-1:0]  fwd_val;
  logic [LQ_SZ-1:0][SQP:0]      fwd_pos;     // SQ position of the store it would forward from
  always_comb begin
    for (int j = 0; j < LQ_SZ; j++) begin
      automatic LQ_ENTRY      e       = lq[j];
      automatic logic [LQP:0] off     = (LQP+1)'(j) - (LQP+1)'(lq_head[LQP-1:0]);
      automatic logic [SQP:0] n_older = e.sq_pos - sq_head;
      automatic bit           unknown = 0, hit = 0;
      automatic SQ_ENTRY      s, h;
      automatic int           hk = 0;
      h = '0;
      lq_valid[j] = (LQP+1)'(off[LQP-1:0]) < lq_count;
      for (int k = 0; k < SQ_SZ; k++)
        if (k < int'(n_older)) begin
          s = sq[(int'(sq_head[SQP-1:0]) + k) % SQ_SZ];
          if (!s.addr_v) begin
            if (`LSQ_SPECULATE == 0 || e.wait_st) unknown = 1;              // speculate: go ahead anyway
          end
          else if (s.exc == EXC_NONE &&                                     // excepting stores never write
                   s.addr < e.addr + `XLEN'(nbytes(e.size)) &&
                   e.addr < s.addr + `XLEN'(nbytes(s.size))) begin
            hit = 1;
            h   = s;                                                        // later k = younger store
            hk  = k;
          end
        end
      can_mem[j] = lq_valid[j] && e.addr_v && !e.done && !e.issued && e.exc == EXC_NONE && !unknown && !hit;
      can_fwd[j] = lq_valid[j] && e.addr_v && !e.done && !e.issued && e.exc == EXC_NONE && !unknown && hit &&
                   h.addr <= e.addr && e.addr + `XLEN'(nbytes(e.size)) <= h.addr + `XLEN'(nbytes(h.size));
      fwd_val[j] = extend(h.data >> (8 * (e.addr - h.addr)), e.size, e.uns);
      fwd_pos[j] = sq_head + (SQP+1)'(hk);
    end
  end

  // ---------------- atomics ----------------
  typedef enum logic [2:0] {AT_IDLE, AT_DRAIN, AT_READ, AT_WAIT, AT_WRITE, AT_DONE} AT_STATE;
  AT_STATE               at_state;
  logic [`XLEN-1:0]      at_addr, at_src, at_wdata, at_result;
  logic [4:0]            at_f5;
  logic [`TAG_W-1:0]     at_tag;
  logic [`ROB_IDX_W-1:0] at_rob;
  logic [ID_W-1:0]       at_id;
  EXCEPTION              at_exc;
  logic                  resv_v;           // LR reservation
  logic [`XLEN-1:2]      resv_word;
  logic                  st_is_amo, at_start;

  assign st_is_amo = (st_in_pkt.inst[6:0] == 7'b0101111);
  assign at_start  = st_in_valid && st_ready && st_is_amo && !flush;

  function automatic logic [`XLEN-1:0] amo_op(input logic [4:0] f5, input logic [`XLEN-1:0] m, input logic [`XLEN-1:0] r);
    case (f5)
      5'b00001: return r;                                                   // amoswap
      5'b00000: return m + r;                                               // amoadd
      5'b00100: return m ^ r;                                               // amoxor
      5'b01100: return m & r;                                               // amoand
      5'b01000: return m | r;                                               // amoor
      5'b10000: return ($signed(m) < $signed(r)) ? m : r;                   // amomin
      5'b10100: return ($signed(m) > $signed(r)) ? m : r;                   // amomax
      5'b11000: return (m < r) ? m : r;                                     // amominu
      default:  return (m > r) ? m : r;                                     // amomaxu
    endcase
  endfunction

  // ---------------- memory request: atomic, else oldest ready load, else drain the oldest committed store ----------------
  logic             req_is_load;
  logic [LQP-1:0]   req_lq;
  always_comb begin
    req_is_load = `FALSE;
    req_lq      = '0;
    for (int o = LQ_SZ - 1; o >= 0; o--) begin                // ends on the oldest
      automatic int j = (int'(lq_head[LQP-1:0]) + o) % LQ_SZ;
      if (can_mem[j]) begin req_is_load = `TRUE; req_lq = LQP'(j); end
    end
    dmem_req_valid = req_is_load || stores_pending;
    dmem_req_store = !req_is_load;
    dmem_req_id    = next_id;
    if (at_state == AT_READ || at_state == AT_WRITE) begin
      // (nothing else is in flight: the atomic is the oldest instruction and stores have drained)
      req_is_load    = `FALSE;
      dmem_req_valid = `TRUE;
      dmem_req_store = (at_state == AT_WRITE);
      dmem_req_addr  = at_addr;
      dmem_req_size  = WORD;
      dmem_req_data  = at_wdata;
    end else if (req_is_load) begin
      dmem_req_addr = lq[req_lq].addr;
      dmem_req_size = lq[req_lq].size;
      dmem_req_data = '0;
    end else begin
      dmem_req_addr = sq[sq_head[SQP-1:0]].addr;
      dmem_req_size = sq[sq_head[SQP-1:0]].size;
      dmem_req_data = sq[sq_head[SQP-1:0]].data;
    end
  end

  // ---------------- result: oldest finished load not yet broadcast ----------------
  logic             out_pick_v;
  logic [LQP-1:0]   out_pick;
  always_comb begin
    out_pick_v = `FALSE;
    out_pick   = '0;
    for (int o = LQ_SZ - 1; o >= 0; o--) begin
      automatic int j = (int'(lq_head[LQP-1:0]) + o) % LQ_SZ;
      if (lq_valid[j] && lq[j].done && !lq[j].bcast) begin out_pick_v = `TRUE; out_pick = LQP'(j); end
    end
  end

  // ---------------- violation check (the store getting its address this cycle) ----------------
  // A load counts as having read a value if it is issued or done, or it is the load accepted by
  // memory this cycle, or it forwards this cycle (those two take effect only at this edge).
  // Not a violation: a load that forwarded from a store younger than this one (that store covers
  // all its bytes, so the value is right whatever this one writes). A forwarding store that has
  // since drained is older than this one: its offset from sq_head is >= SQ_SZ, never < n_older.
  logic [SQP-1:0] st_off;
  always_comb begin
    automatic logic [`XLEN-1:0] sa = st_addr;
    automatic int               sn = nbytes(st_size);
    st_off       = SQP'(st_in_pkt.lsq_idx) - sq_head[SQP-1:0];   // store's age among the SQ entries
    viol_valid   = `FALSE;
    viol_rob_idx = '0;
    for (int o = LQ_SZ - 1; o >= 0; o--) begin                  // ends on the oldest violator
      automatic int           j       = (int'(lq_head[LQP-1:0]) + o) % LQ_SZ;
      automatic LQ_ENTRY      e       = lq[j];
      automatic logic [SQP:0] n_older = e.sq_pos - sq_head;
      automatic bit read_value = e.issued || e.done || can_fwd[j] ||
                                 (req_is_load && req_lq == LQP'(j) && dmem_req_ready);
      automatic bit           fwded   = can_fwd[j] || e.fwd_v;
      automatic logic [SQP:0] fwd_off = (can_fwd[j] ? fwd_pos[j] : e.fwd_pos) - sq_head;
      automatic bit shadowed = fwded && (SQP+1)'(st_off) < fwd_off && fwd_off < n_older;
      if (`LSQ_SPECULATE != 0 && st_in_valid && st_ready && !st_is_amo && !flush && addr_check(st_addr, st_size) == EXC_NONE &&
          lq_valid[j] && e.addr_v && e.exc == EXC_NONE && read_value && !shadowed &&
          (SQP+1)'(st_off) < n_older &&                                       // store is older than the load
          sa < e.addr + `XLEN'(nbytes(e.size)) && e.addr < sa + `XLEN'(sn)) begin // and overlaps it
        viol_valid   = `TRUE;
        viol_rob_idx = e.rob_idx;
      end
    end
  end

  // ---------------- commit counts ----------------
  logic [LQP:0] lq_head_next;
  logic [SQP:0] sq_cmt_next;
  assign lq_head_next = lq_head + (LQP+1)'($countones(commit_load));
  assign sq_cmt_next  = sq_cmt  + (SQP+1)'($countones(commit_store));

  // ---------------- state ----------------
  always_ff @(posedge clock) begin
    if (reset) begin
      lq_head <= '0; lq_tail <= '0;
      sq_head <= '0; sq_cmt <= '0; sq_tail <= '0;
      next_id <= '0;
      ld_out_valid <= `FALSE;
      st_out_valid <= `FALSE;
      at_state     <= AT_IDLE;
      resv_v       <= `FALSE;
    end else begin
      // ---- atomic engine ----
      case (at_state)
        AT_IDLE: if (at_start) begin
          at_exc   <= addr_check(st_rs1, WORD);
          at_addr  <= st_rs1;
          at_src   <= st_rs2;
          at_f5    <= st_in_pkt.inst[31:27];
          at_tag   <= st_in_pkt.dest_tag;
          at_rob   <= st_in_pkt.rob_idx;
          at_result <= '0;
          at_state <= (addr_check(st_rs1, WORD) != EXC_NONE) ? AT_DONE : AT_DRAIN;
        end
        AT_DRAIN: if (!stores_pending) begin
          if (at_f5 == 5'b00011) begin                             // sc.w
            resv_v <= `FALSE;
            if (resv_v && resv_word == at_addr[`XLEN-1:2]) begin
              at_wdata <= at_src; at_result <= '0; at_state <= AT_WRITE;
            end else begin
              at_result <= 1; at_state <= AT_DONE;
            end
          end else at_state <= AT_READ;                              // lr.w, AMOs
        end
        AT_READ: if (dmem_req_ready) begin
          at_id    <= next_id;
          next_id  <= next_id + 1'b1;
          at_state <= AT_WAIT;
        end
        AT_WAIT: if (dmem_resp_valid && dmem_resp_id == at_id) begin
          at_result <= dmem_resp_data;
          if (at_f5 == 5'b00010) begin                             // lr.w
            resv_v    <= `TRUE;
            resv_word <= at_addr[`XLEN-1:2];
            at_state  <= AT_DONE;
          end else begin
            at_wdata <= amo_op(at_f5, dmem_resp_data, at_src);
            at_state <= AT_WRITE;
          end
        end
        AT_WRITE: if (dmem_req_ready) at_state <= AT_DONE;
        default: if (!st_out_valid || st_out_grant) at_state <= AT_IDLE;   // AT_DONE: result goes out below
      endcase
      // reservation: any store written to the reserved word clears it (after an LR sets it above)
      if (dmem_req_valid && dmem_req_ready && dmem_req_store && dmem_req_addr[`XLEN-1:2] == resv_word)
        resv_v <= `FALSE;

      // memory request accepted
      if (dmem_req_valid && dmem_req_ready && at_state != AT_READ && at_state != AT_WRITE) begin
        if (req_is_load) begin
          lq[req_lq].issued <= `TRUE;
          lq[req_lq].req_id <= next_id;
          next_id           <= next_id + 1'b1;
        end else begin
          sq_head <= sq_head + 1'b1;
        end
      end

      // memory response: only the issued entry waiting for this id
      if (dmem_resp_valid)
        for (int j = 0; j < LQ_SZ; j++)
          if (lq_valid[j] && lq[j].issued && !lq[j].done && lq[j].req_id == dmem_resp_id) begin
            lq[j].done  <= `TRUE;
            lq[j].value <= extend(dmem_resp_data >> (8 * lq[j].addr[1:0]), lq[j].size, lq[j].uns);
          end

      // store-to-load forwarding
      for (int j = 0; j < LQ_SZ; j++)
        if (can_fwd[j]) begin
          lq[j].done    <= `TRUE;
          lq[j].value   <= fwd_val[j];
          lq[j].fwd_v   <= `TRUE;
          lq[j].fwd_pos <= fwd_pos[j];
        end

      // load result register
      if (ld_out_valid && ld_out_grant) ld_out_valid <= `FALSE;
      if ((!ld_out_valid || ld_out_grant) && out_pick_v) begin
        ld_out_valid <= `TRUE;
        ld_out_pkt   <= '{tag: lq[out_pick].tag, value: lq[out_pick].value, rob_idx: lq[out_pick].rob_idx,
                          branch: `FALSE, next_pc: '0, exception: lq[out_pick].exc, fflags: '0};
        lq[out_pick].bcast <= `TRUE;
      end

      // store completion register (also carries an atomic's result)
      if (st_out_valid && st_out_grant) st_out_valid <= `FALSE;
      if (at_state == AT_DONE && (!st_out_valid || st_out_grant)) begin
        st_out_valid <= `TRUE;
        st_out_pkt   <= '{tag: at_tag, value: at_result, rob_idx: at_rob,
                          branch: `FALSE, next_pc: '0, exception: at_exc, fflags: '0};
      end

      if (!flush) begin
        // load address generation
        if (ld_in_valid) begin
          automatic logic [LQP-1:0] j = LQP'(ld_in_pkt.lsq_idx);
          automatic EXCEPTION       x = addr_check(ld_addr, ld_size);
          lq[j].addr_v  <= `TRUE;
          lq[j].addr    <= ld_addr;
          lq[j].size    <= ld_size;
          lq[j].uns     <= ld_in_pkt.inst[14];
          lq[j].tag     <= ld_in_pkt.dest_tag;
          lq[j].rob_idx <= ld_in_pkt.rob_idx;
          lq[j].wait_st <= ld_wait;
          lq[j].exc     <= x;
          if (x != EXC_NONE) begin lq[j].done <= `TRUE; lq[j].value <= '0; end
        end
        // store address + data
        if (st_in_valid && st_ready && !st_is_amo) begin
          automatic logic [SQP-1:0] k = SQP'(st_in_pkt.lsq_idx);
          automatic EXCEPTION       x = addr_check(st_addr, st_size);
          sq[k] <= '{addr_v: `TRUE, addr: st_addr, size: st_size, data: st_rs2, exc: x};
          st_out_valid <= `TRUE;
          st_out_pkt   <= '{tag: st_in_pkt.dest_tag, value: '0, rob_idx: st_in_pkt.rob_idx,
                            branch: `FALSE, next_pc: '0, exception: x, fflags: '0};
        end
        // allocation (dispatch is blocked in a flush cycle anyway)
        for (int i = 0; i < W; i++) begin
          if (disp_load[i])
            lq[disp_lsq_idx[i][LQP-1:0]] <= '{addr_v: `FALSE, addr: '0, size: BYTE, uns: `FALSE, tag: '0,
                                              rob_idx: '0, sq_pos: disp_sq_pos[i], issued: `FALSE, req_id: '0,
                                              done: `FALSE, bcast: `FALSE, value: '0, exc: EXC_NONE,
                                              wait_st: `FALSE, fwd_v: `FALSE, fwd_pos: '0};
          if (disp_store[i]) sq[disp_lsq_idx[i][SQP-1:0]].addr_v <= `FALSE;
        end
        lq_tail <= lq_tail + (LQP+1)'($countones(disp_load));
        sq_tail <= sq_tail + (SQP+1)'($countones(disp_store));
      end else begin
        // squash everything uncommitted; committed stores stay and keep draining
        lq_tail      <= lq_head_next;
        sq_tail      <= sq_cmt_next;
        ld_out_valid <= `FALSE;
        st_out_valid <= `FALSE;
      end

      lq_head <= lq_head_next;
      sq_cmt  <= sq_cmt_next;
    end
  end

endmodule  // lsq
