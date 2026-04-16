`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 31.03.2026 00:44:18
// Design Name: 
// Module Name: tb_fft64
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module tb_fft64();

// =============================================================================
//  Testbench: 64-point Radix-4 FFT
//  Input  : x[n] = n+1  (i.e. 1, 2, 3, ... 64), all imaginary = 0
//  Checks : DC bin (bin 0) and prints all 64 output bins
//
//  Expected DC value (bin 0) = sum(1..64) = 2080
//  After internal /4 scaling per stage (3 stages → /64):
//    bin0_re ≈ 2080 / 64 = 32.5  (integer truncation artefacts expected)
// =============================================================================


    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg         clk;
    reg         rst;
    reg         start;
    reg         din_valid;
    reg  signed [15:0] din_re;
    reg  signed [15:0] din_im;

    wire signed [17:0] dout_re;
    wire signed [17:0] dout_im;
    wire        dout_valid;
    wire        done;

    // -------------------------------------------------------------------------
    // DUT instantiation
    // -------------------------------------------------------------------------
    fft64_radix4_seq UUT (
        .clk       (clk),
        .rst       (rst),
        .start     (start),
        .din_re    (din_re),
        .din_im    (din_im),
        .din_valid (din_valid),
        .dout_re   (dout_re),
        .dout_im   (dout_im),
        .dout_valid(dout_valid),
        .done      (done)
    );

    // -------------------------------------------------------------------------
    // Clock: 10 ns period (100 MHz)
    // -------------------------------------------------------------------------
    initial clk = 0;
    always  #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // Output capture
    // -------------------------------------------------------------------------
    integer bin_idx;
    real    mag;

    // -------------------------------------------------------------------------
    // Stimulus
    // -------------------------------------------------------------------------
    integer k;

    initial begin
        // -- waveform dump (comment out if not needed) --
        $dumpfile("tb_fft64_ramp.vcd");
        $dumpvars(0, tb_fft64_ramp);

        // ---- reset ----
        rst       = 1;
        start     = 0;
        din_valid = 0;
        din_re    = 0;
        din_im    = 0;
        bin_idx   = 0;

        repeat(4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // ---- assert start for one cycle ----
        start = 1;
        @(posedge clk);
        start = 0;

        // ---- feed 64 samples: x[n] = n+1, xi[n] = 0 ----
        //      din_valid must be high for each sample
        $display("=== Feeding input samples ===");
        for (k = 0; k < 64; k = k + 1) begin
            @(posedge clk);
            din_valid = 1;
            din_re    = k + 1;   // 1, 2, 3, ... 64
            din_im    = 0;
            $display("  n=%0d  x[n]=%0d", k, k+1);
        end
        @(posedge clk);
        din_valid = 0;
        din_re    = 0;
        din_im    = 0;

        // ---- wait for done flag ----
        $display("\n=== Waiting for FFT to complete ... ===");
        @(posedge done);
        $display("=== FFT done ===\n");

        // Give a couple extra cycles for display to flush
        repeat(4) @(posedge clk);
        $finish;
    end

    // -------------------------------------------------------------------------
    // Output monitor - runs concurrently, captures each bin as it appears
    // -------------------------------------------------------------------------
    initial bin_idx = 0;

    always @(posedge clk) begin
        if (dout_valid) begin
            mag = $sqrt($itor(dout_re) * $itor(dout_re) +
                        $itor(dout_im) * $itor(dout_im));
            $display("Bin %2d :  Re = %6d   Im = %6d   |X| = %.2f",
                     bin_idx, dout_re, dout_im, mag);
            bin_idx = bin_idx + 1;
        end
    end

    // -------------------------------------------------------------------------
    // Timeout watchdog (prevent infinite simulation)
    // -------------------------------------------------------------------------
    initial begin
        #500000;
        $display("TIMEOUT: simulation exceeded limit");
        $finish;
    end

endmodule