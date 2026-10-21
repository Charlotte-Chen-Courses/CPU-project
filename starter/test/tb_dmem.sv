/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  tb_dmem.sv                                          //
//                                                                     //
//  Description :  Testbench data memory with the D-cache's CPU-side   //
//                 interface, harsher than the real D-cache: each      //
//                 cycle a request is accepted with probability        //
//                 ready_pct %; a store is written when accepted; a    //
//                 load reads its word when accepted and answers after //
//                 a random latency (1..max_lat), picking randomly     //
//                 among due answers, so they come back out of order.  //
//                 A request outside memory is a fatal error (the LSQ  //
//                 must never send one). Testbench-only.               //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

/* verilator lint_off BLKSEQ */
/* verilator lint_off PROCASSINIT */
/* verilator lint_off WIDTHEXPAND */

module tb_dmem (
    input                          clock,
    input                          reset,
    input                          req_valid,
    output logic                   req_ready,
    input                          req_store,
    input        [`XLEN-1:0]       req_addr,
    input  MEM_SIZE                req_size,
    input        [`XLEN-1:0]       req_data,
    input        [`DMEM_ID_W-1:0]  req_id,
    output logic                   resp_valid,
    output logic [`DMEM_ID_W-1:0]  resp_id,
    output logic [`XLEN-1:0]       resp_data
);
    logic [7:0] mem [`MEM_SIZE_IN_BYTES];
    int     ready_pct = 75;
    int     max_lat   = 12;
    longint cycle     = 0;
    int     n_loads   = 0, n_stores = 0;

    typedef struct { logic [`DMEM_ID_W-1:0] id; logic [`XLEN-1:0] data; longint due; } pend_t;
    pend_t pend [$];

    function automatic logic [`XLEN-1:0] word_at(input logic [`XLEN-1:0] a);
        automatic int b = int'(a) & ~3;
        return {mem[b + 3], mem[b + 2], mem[b + 1], mem[b]};
    endfunction

    // accept on the rising edge (the DUT's outputs still show this cycle's values here)
    always @(posedge clock) begin
        cycle++;
        if (reset) pend.delete();
        else if (req_valid && req_ready) begin
            if (req_addr >= `MEM_SIZE_IN_BYTES) $fatal(1, "tb_dmem: request to %h is outside memory", req_addr);
            if (req_store) begin
                automatic int n = (req_size == BYTE) ? 1 : (req_size == HALF) ? 2 : 4;
                for (int k = 0; k < n; k++) mem[int'(req_addr) + k] = req_data[8 * k +: 8];
                n_stores++;
            end else begin
                pend.push_back('{id: req_id, data: word_at(req_addr), due: cycle + $urandom_range(1, max_lat)});
                n_loads++;
            end
        end
    end

    // drive ready and at most one due answer for the next edge
    always @(negedge clock) begin
        automatic int due [$];
        req_ready  = !reset && ($urandom % 100 < ready_pct);
        resp_valid = 0;
        resp_id    = '0;
        resp_data  = '0;
        foreach (pend[i]) if (pend[i].due <= cycle) due.push_back(i);
        if (due.size() > 0) begin
            automatic int i = due[$urandom_range(0, due.size() - 1)];
            resp_valid = 1;
            resp_id    = pend[i].id;
            resp_data  = pend[i].data;
            pend.delete(i);
        end
    end
endmodule
