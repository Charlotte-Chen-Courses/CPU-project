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
  assign st_ready = !st_out_valid || st_out_grant;  // completion register free

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
            if (!`LSQ_SPECULATE || e.wait_st) unknown = 1;                  // speculate: go ahead anyway
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

  // ---------------- memory request: oldest ready load, else drain the oldest committed store ----------------
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
    if (req_is_load) begin
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
      if (`LSQ_SPECULATE && st_in_valid && st_ready && !flush && addr_check(st_addr, st_size) == EXC_NONE &&
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
    end else begin
      // memory request accepted
      if (dmem_req_valid && dmem_req_ready) begin
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
                          branch: `FALSE, next_pc: '0, exception: lq[out_pick].exc};
        lq[out_pick].bcast <= `TRUE;
      end

      // store completion register
      if (st_out_valid && st_out_grant) st_out_valid <= `FALSE;

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
        if (st_in_valid && st_ready) begin
          automatic logic [SQP-1:0] k = SQP'(st_in_pkt.lsq_idx);
          automatic EXCEPTION       x = addr_check(st_addr, st_size);
          sq[k] <= '{addr_v: `TRUE, addr: st_addr, size: st_size, data: st_rs2, exc: x};
          st_out_valid <= `TRUE;
          st_out_pkt   <= '{tag: st_in_pkt.dest_tag, value: '0, rob_idx: st_in_pkt.rob_idx,
                            branch: `FALSE, next_pc: '0, exception: x};
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
