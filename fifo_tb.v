`timescale 1ns/1ps

// ======================================================
// Top-level Asynchronous FIFO
// ======================================================
module FIFO #(
    parameter DSIZE = 8,   // Data width
    parameter ASIZE = 4    // Address width
)(
    output [DSIZE-1:0] rdata,  // Read data
    output wfull,              // FIFO full flag
    output rempty,             // FIFO empty flag
    input  [DSIZE-1:0] wdata,  // Write data
    input  winc, wclk, wrst_n, // Write enable, write clock, write reset (active low)
    input  rinc, rclk, rrst_n  // Read enable, read clock, read reset (active low)
);

    wire [ASIZE-1:0] waddr, raddr;
    wire [ASIZE:0] wptr, rptr, wq2_rptr, rq2_wptr;

    // Read pointer synchronized to write domain
    two_ff_sync #(ASIZE+1) sync_r2w (
        .q2   (wq2_rptr),
        .din  (rptr),
        .clk  (wclk),
        .rst_n(wrst_n)
    );

    // Write pointer synchronized to read domain
    two_ff_sync #(ASIZE+1) sync_w2r (
        .q2   (rq2_wptr),
        .din  (wptr),
        .clk  (rclk),
        .rst_n(rrst_n)
    );

    // FIFO Memory
    FIFO_memory #(DSIZE, ASIZE) fifomem (
        .rdata   (rdata),
        .wdata   (wdata),
        .waddr   (waddr),
        .raddr   (raddr),
        .wclk_en (winc),
        .wfull   (wfull),
        .wclk    (wclk),
        .rempty  (rempty)  // Added to control rdata
    );

    // Read pointer & empty flag
    rptr_empty #(ASIZE) rptr_empty_inst (
        .rempty   (rempty),
        .raddr    (raddr),
        .rptr     (rptr),
        .rq2_wptr (rq2_wptr),
        .rinc     (rinc),
        .rclk     (rclk),
        .rrst_n   (rrst_n)
    );

    // Write pointer & full flag
    wptr_full #(ASIZE) wptr_full_inst (
        .wfull    (wfull),
        .waddr    (waddr),
        .wptr     (wptr),
        .wq2_rptr (wq2_rptr),
        .winc     (winc),
        .wclk     (wclk),
        .wrst_n   (wrst_n)
    );

endmodule

// Two flip-flop synchronizer
module two_ff_sync #(parameter SIZE = 4)(
    output reg [SIZE-1:0] q2,
    input  [SIZE-1:0] din,
    input  clk, rst_n
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

// FIFO Memory
module FIFO_memory #(parameter DSIZE = 8, ASIZE = 4)(
    output reg [DSIZE-1:0] rdata,
    input  [DSIZE-1:0] wdata,
    input  [ASIZE-1:0] waddr, raddr,
    input  wclk_en, wfull, wclk, rempty
);
    localparam DEPTH = 1 << ASIZE;
    reg [DSIZE-1:0] mem [0:DEPTH-1];

    always @(posedge wclk) begin
        if (wclk_en && !wfull)
            mem[waddr] <= wdata;
    end

    always @(*) begin
        if (rempty)
            rdata = {DSIZE{1'bX}}; // Output X when empty
        else
            rdata = mem[raddr];
    end
endmodule

// Read pointer and empty flag
module rptr_empty #(parameter ASIZE = 4)(
    output reg rempty,
    output [ASIZE-1:0] raddr,
    output reg [ASIZE:0] rptr,
    input  [ASIZE:0] rq2_wptr,
    input  rinc, rclk, rrst_n
);
    reg [ASIZE:0] rbin;
    wire [ASIZE:0] rgraynext, rbinnext;

    assign rbinnext  = rbin + (rinc & ~rempty);
    assign rgraynext = (rbinnext >> 1) ^ rbinnext;
    assign raddr     = rbin[ASIZE-1:0];

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rbin   <= 0;
            rptr   <= 0;
            rempty <= 1'b1;
        end else begin
            rbin   <= rbinnext;
            rptr   <= rgraynext;
            rempty <= (rgraynext == rq2_wptr);
        end
    end
endmodule

// Write pointer and full flag
module wptr_full #(parameter ASIZE = 4)(
    output reg wfull,
    output [ASIZE-1:0] waddr,
    output reg [ASIZE:0] wptr,
    input  [ASIZE:0] wq2_rptr,
    input  winc, wclk, wrst_n
);
    reg [ASIZE:0] wbin;
    wire [ASIZE:0] wgraynext, wbinnext;

    assign wbinnext  = wbin + (winc & ~wfull);
    assign wgraynext = (wbinnext >> 1) ^ wbinnext;
    assign waddr     = wbin[ASIZE-1:0];

    wire wfull_next;
    assign wfull_next = (wgraynext == {~wq2_rptr[ASIZE:ASIZE-1], wq2_rptr[ASIZE-2:0]});

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wbin  <= 0;
            wptr  <= 0;
            wfull <= 1'b0;
        end else begin
            wbin  <= wbinnext;
            wptr  <= wgraynext;
            wfull <= wfull_next;
        end
    end
endmodule