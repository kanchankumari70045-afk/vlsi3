// ====================================================
// Asynchronous FIFO with parameterized data/address size
// ====================================================

module FIFO #(parameter DSIZE = 8,  // Data width
              parameter ASIZE = 4)  // Address width (FIFO depth = 2^ASIZE)
(
    output [DSIZE-1:0] rdata,   // Data output
    output wfull,               // FIFO full flag
    output rempty,              // FIFO empty flag
    input  [DSIZE-1:0] wdata,   // Data input
    input  winc, wclk, wrst_n,  // Write enable, write clock, write reset
    input  rinc, rclk, rrst_n   // Read enable, read clock, read reset
);

    wire [ASIZE-1:0] waddr, raddr;
    wire [ASIZE:0] wptr, rptr;          // Binary pointers
    wire [ASIZE:0] wq2_rptr, rq2_wptr;  // Synchronized pointers (Gray-coded)

    // Synchronize read pointer into write clock domain
    two_ff_sync #(ASIZE+1) sync_r2w (
        .q2   (wq2_rptr),
        .din  (rptr),
        .clk  (wclk),
        .rst_n(wrst_n)
    );

    // Synchronize write pointer into read clock domain
    two_ff_sync #(ASIZE+1) sync_w2r (
        .q2   (rq2_wptr),
        .din  (wptr),
        .clk  (rclk),
        .rst_n(rrst_n)
    );

    // FIFO memory array
    FIFO_memory #(DSIZE, ASIZE) fifomem (
        .rdata   (rdata),
        .wdata   (wdata),
        .waddr   (waddr),
        .raddr   (raddr),
        .wclken  (winc),
        .wfull   (wfull),
        .wclk    (wclk)
    );

    // Read pointer and empty signal logic
    rptr_empty #(ASIZE) rptr_empty_inst (
        .rempty  (rempty),
        .raddr   (raddr),
        .rptr    (rptr),
        .rq2_wptr(rq2_wptr),
        .rinc    (rinc),
        .rclk    (rclk),
        .rrst_n  (rrst_n)
    );

    // Write pointer and full signal logic
    wptr_full #(ASIZE) wptr_full_inst (
        .wfull   (wfull),
        .waddr   (waddr),
        .wptr    (wptr),
        .wq2_rptr(wq2_rptr),
        .winc    (winc),
        .wclk    (wclk),
        .wrst_n  (wrst_n)
    );

endmodule

// ====================================================
// Two flip-flop synchronizer
// ====================================================
module two_ff_sync #(parameter SIZE = 4)(
    output reg [SIZE-1:0] q2,
    input  [SIZE-1:0] din,
    input clk, rst_n
);
    reg [SIZE-1:0] q1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q1 <= 0;
            q2 <= 0;
        end else begin
            q1 <= din;
            q2 <= q1;
        end
    end
endmodule

// ====================================================
// FIFO Memory
// ====================================================
module FIFO_memory #(parameter DSIZE = 8, ASIZE = 4)(
    output [DSIZE-1:0] rdata,
    input  [DSIZE-1:0] wdata,
    input  [ASIZE-1:0] waddr, raddr,
    input  wclken, wfull, wclk
);
    localparam DEPTH = 1 << ASIZE;
    reg [DSIZE-1:0] mem [0:DEPTH-1];

    always @(posedge wclk) begin
        if (wclken && !wfull)
            mem[waddr] <= wdata;
    end

    assign rdata = mem[raddr];
endmodule

// ====================================================
// Read pointer and empty logic
// ====================================================
module rptr_empty #(parameter ASIZE = 4)(
    output reg rempty,
    output [ASIZE-1:0] raddr,
    output reg [ASIZE:0] rptr,
    input  [ASIZE:0] rq2_wptr,
    input  rinc, rclk, rrst_n
);
    reg [ASIZE:0] rbin;
    wire [ASIZE:0] rgray_next, rbin_next;
    wire rempty_val;

    // Binary counter
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rbin <= 0;
            rptr <= 0;
        end else begin
            rbin <= rbin_next;
            rptr <= rgray_next;
        end
    end

    assign rbin_next  = rbin + (rinc & ~rempty);
    assign rgray_next = (rbin_next >> 1) ^ rbin_next;
    assign raddr      = rbin[ASIZE-1:0];

    // Empty when synchronized write ptr == next read ptr
    assign rempty_val = (rgray_next == rq2_wptr);

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n)
            rempty <= 1'b1;
        else
            rempty <= rempty_val;
    end
endmodule

// ====================================================
// Write pointer and full logic
// ====================================================
module wptr_full #(parameter ASIZE = 4)(
    output reg wfull,
    output [ASIZE-1:0] waddr,
    output reg [ASIZE:0] wptr,
    input  [ASIZE:0] wq2_rptr,
    input  winc, wclk, wrst_n
);
    reg [ASIZE:0] wbin;
    wire [ASIZE:0] wgray_next, wbin_next;
    wire wfull_val;

    // Binary counter
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wbin <= 0;
            wptr <= 0;
        end else begin
            wbin <= wbin_next;
            wptr <= wgray_next;
        end
    end

    assign wbin_next  = wbin + (winc & ~wfull);
    assign wgray_next = (wbin_next >> 1) ^ wbin_next;
    assign waddr      = wbin[ASIZE-1:0];

    // Full when next write pointer == read pointer (inverted MSBs)
    assign wfull_val  = (wgray_next == {~wq2_rptr[ASIZE:ASIZE-1], wq2_rptr[ASIZE-2:0]});

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n)
            wfull <= 1'b0;
        else
            wfull <= wfull_val;
    end
endmodule
