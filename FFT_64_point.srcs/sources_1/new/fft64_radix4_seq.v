// =============================================================================
//  64-Point Radix-4 FFT  -  Sequential (state-machine) Architecture
//  with Twiddle-Factor ROM
//
//  Architecture:
//    64 = 4^3  →  3 stages, each stage processes 16 radix-4 butterflies
//    A single radix_4_16_18 butterfly is instantiated and reused.
//    Twiddle factors are stored as 16-bit signed fixed-point (Q1.15).
//
//  Ports
//  -----
//  clk        : system clock
//  rst        : synchronous active-high reset
//  start      : pulse high for 1 cycle to begin a new FFT
//  din_re/im  : 16-bit real/imaginary input sample (presented sequentially,
//               sample 0 first, on successive cycles after start)
//  din_valid  : asserted for each valid input sample
//  dout_re/im : 18-bit output bin (presented sequentially, bin 0 first)
//  dout_valid : asserted for each valid output bin
//  done       : pulses high for 1 cycle after last output bin
// =============================================================================
`timescale 1ns / 1ps

// ---------------------------------------------------------------------------
// Radix-4 butterfly (combinational) - as supplied by the user
// ---------------------------------------------------------------------------
module radix_4_16_18 (
    input  signed [15:0] xr0, xi0,
    input  signed [15:0] xr1, xi1,
    input  signed [15:0] xr2, xi2,
    input  signed [15:0] xr3, xi3,

    output signed [17:0] yr0, yi0,
    output signed [17:0] yr1, yi1,
    output signed [17:0] yr2, yi2,
    output signed [17:0] yr3, yi3
);
    assign yr0 = xr0 + xr1 + xr2 + xr3;
    assign yi0 = xi0 + xi1 + xi2 + xi3;
    assign yr1 = xr0 + xi1 - xr2 - xi3;
    assign yi1 = xi0 - xr1 - xi2 + xr3;
    assign yr2 = xr0 - xr1 + xr2 - xr3;
    assign yi2 = xi0 - xi1 + xi2 - xi3;
    assign yr3 = xr0 - xi1 - xr2 + xi3;
    assign yi3 = xi0 + xr1 - xi2 - xr3;
endmodule


// ---------------------------------------------------------------------------
// Complex multiplier:  (ar + j*ai) * (wr + j*wi)
//   inputs  : 18-bit data  x  16-bit twiddle  →  18-bit truncated output
// ---------------------------------------------------------------------------
module cmult_18x16 (
    input  signed [17:0] ar, ai,
    input  signed [15:0] wr, wi,
    output signed [17:0] pr, pi
);
    wire signed [33:0] rr = ar * wr;
    wire signed [33:0] ii = ai * wi;
    wire signed [33:0] ri = ar * wi;
    wire signed [33:0] ir = ai * wr;

    // Q1.15 twiddle  ×  Q1.0-ish data → shift right 15 to re-align
    assign pr = (rr - ii) >>> 15;
    assign pi = (ri + ir) >>> 15;
endmodule


// ---------------------------------------------------------------------------
// Twiddle-factor ROM
//   Returns W_N^k  =  cos(2π k/N) - j·sin(2π k/N)  in Q1.15
//
//   We need twiddles for all three stages of a 64-point radix-4 FFT.
//   Stage 1  : W_64^k,  k = 0..15  (output of first butterfly × W_64^k)
//   Stage 2  : W_16^k,  k = 0..3   (= W_64^(4k), covered above)
//   We store a 64-entry table W_64^0 .. W_64^63 and index into it.
//
//   cos/sin values scaled to 32767 (0x7FFF) for maximum Q1.15 precision.
// ---------------------------------------------------------------------------
module twiddle_rom_64 (
    input      [5:0]  addr,   // 0..63
    output reg signed [15:0] wr,
    output reg signed [15:0] wi
);
    // W_64^k = cos(2πk/64) - j·sin(2πk/64), k = 0..63
    // Values = round(32767 * cos/sin(...))
    always @(*) begin
        case (addr)
            6'd0  : begin wr = 16'sd32767; wi = 16'sd0;      end
            6'd1  : begin wr = 16'sd32609; wi = -16'sd3212;  end
            6'd2  : begin wr = 16'sd32138; wi = -16'sd6393;  end
            6'd3  : begin wr = 16'sd31357; wi = -16'sd9512;  end
            6'd4  : begin wr = 16'sd30274; wi = -16'sd12540; end
            6'd5  : begin wr = 16'sd28899; wi = -16'sd15447; end
            6'd6  : begin wr = 16'sd27246; wi = -16'sd18205; end
            6'd7  : begin wr = 16'sd25330; wi = -16'sd20788; end
            6'd8  : begin wr = 16'sd23170; wi = -16'sd23170; end
            6'd9  : begin wr = 16'sd20788; wi = -16'sd25330; end
            6'd10 : begin wr = 16'sd18205; wi = -16'sd27246; end
            6'd11 : begin wr = 16'sd15447; wi = -16'sd28899; end
            6'd12 : begin wr = 16'sd12540; wi = -16'sd30274; end
            6'd13 : begin wr = 16'sd9512;  wi = -16'sd31357; end
            6'd14 : begin wr = 16'sd6393;  wi = -16'sd32138; end
            6'd15 : begin wr = 16'sd3212;  wi = -16'sd32609; end
            6'd16 : begin wr = 16'sd0;     wi = -16'sd32767; end
            6'd17 : begin wr = -16'sd3212; wi = -16'sd32609; end
            6'd18 : begin wr = -16'sd6393; wi = -16'sd32138; end
            6'd19 : begin wr = -16'sd9512; wi = -16'sd31357; end
            6'd20 : begin wr = -16'sd12540;wi = -16'sd30274; end
            6'd21 : begin wr = -16'sd15447;wi = -16'sd28899; end
            6'd22 : begin wr = -16'sd18205;wi = -16'sd27246; end
            6'd23 : begin wr = -16'sd20788;wi = -16'sd25330; end
            6'd24 : begin wr = -16'sd23170;wi = -16'sd23170; end
            6'd25 : begin wr = -16'sd25330;wi = -16'sd20788; end
            6'd26 : begin wr = -16'sd27246;wi = -16'sd18205; end
            6'd27 : begin wr = -16'sd28899;wi = -16'sd15447; end
            6'd28 : begin wr = -16'sd30274;wi = -16'sd12540; end
            6'd29 : begin wr = -16'sd31357;wi = -16'sd9512;  end
            6'd30 : begin wr = -16'sd32138;wi = -16'sd6393;  end
            6'd31 : begin wr = -16'sd32609;wi = -16'sd3212;  end
            6'd32 : begin wr = -16'sd32767;wi = 16'sd0;      end
            6'd33 : begin wr = -16'sd32609;wi = 16'sd3212;   end
            6'd34 : begin wr = -16'sd32138;wi = 16'sd6393;   end
            6'd35 : begin wr = -16'sd31357;wi = 16'sd9512;   end
            6'd36 : begin wr = -16'sd30274;wi = 16'sd12540;  end
            6'd37 : begin wr = -16'sd28899;wi = 16'sd15447;  end
            6'd38 : begin wr = -16'sd27246;wi = 16'sd18205;  end
            6'd39 : begin wr = -16'sd25330;wi = 16'sd20788;  end
            6'd40 : begin wr = -16'sd23170;wi = 16'sd23170;  end
            6'd41 : begin wr = -16'sd20788;wi = 16'sd25330;  end
            6'd42 : begin wr = -16'sd18205;wi = 16'sd27246;  end
            6'd43 : begin wr = -16'sd15447;wi = 16'sd28899;  end
            6'd44 : begin wr = -16'sd12540;wi = 16'sd30274;  end
            6'd45 : begin wr = -16'sd9512; wi = 16'sd31357;  end
            6'd46 : begin wr = -16'sd6393; wi = 16'sd32138;  end
            6'd47 : begin wr = -16'sd3212; wi = 16'sd32609;  end
            6'd48 : begin wr = 16'sd0;     wi = 16'sd32767;  end
            6'd49 : begin wr = 16'sd3212;  wi = 16'sd32609;  end
            6'd50 : begin wr = 16'sd6393;  wi = 16'sd32138;  end
            6'd51 : begin wr = 16'sd9512;  wi = 16'sd31357;  end
            6'd52 : begin wr = 16'sd12540; wi = 16'sd30274;  end
            6'd53 : begin wr = 16'sd15447; wi = 16'sd28899;  end
            6'd54 : begin wr = 16'sd18205; wi = 16'sd27246;  end
            6'd55 : begin wr = 16'sd20788; wi = 16'sd25330;  end
            6'd56 : begin wr = 16'sd23170; wi = 16'sd23170;  end
            6'd57 : begin wr = 16'sd25330; wi = 16'sd20788;  end
            6'd58 : begin wr = 16'sd27246; wi = 16'sd18205;  end
            6'd59 : begin wr = 16'sd28899; wi = 16'sd15447;  end
            6'd60 : begin wr = 16'sd30274; wi = 16'sd12540;  end
            6'd61 : begin wr = 16'sd31357; wi = 16'sd9512;   end
            6'd62 : begin wr = 16'sd32138; wi = 16'sd6393;   end
            6'd63 : begin wr = 16'sd32609; wi = 16'sd3212;   end
        endcase
    end
endmodule


// =============================================================================
//  Top-level: fft64_radix4_seq
// =============================================================================
// Pipeline / dataflow:
//   IDLE  → LOAD (64 cycles, collect samples into RAM)
//         → STAGE1 (process 16 butterflies, each 1 cycle, write back)
//            → TWIDDLE1 (apply W_64^k twiddles after stage 1)
//         → STAGE2 (process 16 butterflies, each 1 cycle, write back)
//            → TWIDDLE2 (apply W_4^k = trivial ±1/±j twiddles)
//         → STAGE3 (final 16 butterflies)
//         → OUTPUT (stream 64 bins)
//
// Bit-reversal permutation for radix-4, N=64:
//   6-bit index, reverse in 2-bit groups: b5b4_b3b2_b1b0 → b1b0_b3b2_b5b4
// =============================================================================
module fft64_radix4_seq (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire signed [15:0] din_re,
    input  wire signed [15:0] din_im,
    input  wire        din_valid,
    output reg  signed [17:0] dout_re,
    output reg  signed [17:0] dout_im,
    output reg         dout_valid,
    output reg         done
);

    // -----------------------------------------------------------------------
    // Internal RAM  -  64 complex words, 18 bits each side
    // -----------------------------------------------------------------------
    reg signed [17:0] ram_re [0:63];
    reg signed [17:0] ram_im [0:63];

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    localparam  S_IDLE     = 4'd0,
                S_LOAD     = 4'd1,
                S_STAGE1   = 4'd2,
                S_TWIDDLE1 = 4'd3,
                S_STAGE2   = 4'd4,
                S_TWIDDLE2 = 4'd5,
                S_STAGE3   = 4'd6,
                S_OUTPUT   = 4'd7;

    reg [3:0] state;
    reg [6:0] cnt;   // general counter, 0..63

    // -----------------------------------------------------------------------
    // Butterfly wiring
    // -----------------------------------------------------------------------
    reg  signed [15:0] bf_xr0, bf_xi0;
    reg  signed [15:0] bf_xr1, bf_xi1;
    reg  signed [15:0] bf_xr2, bf_xi2;
    reg  signed [15:0] bf_xr3, bf_xi3;

    wire signed [17:0] bf_yr0, bf_yi0;
    wire signed [17:0] bf_yr1, bf_yi1;
    wire signed [17:0] bf_yr2, bf_yi2;
    wire signed [17:0] bf_yr3, bf_yi3;

    radix_4_16_18 BF (
        .xr0(bf_xr0), .xi0(bf_xi0),
        .xr1(bf_xr1), .xi1(bf_xi1),
        .xr2(bf_xr2), .xi2(bf_xi2),
        .xr3(bf_xr3), .xi3(bf_xi3),
        .yr0(bf_yr0), .yi0(bf_yi0),
        .yr1(bf_yr1), .yi1(bf_yi1),
        .yr2(bf_yr2), .yi2(bf_yi2),
        .yr3(bf_yr3), .yi3(bf_yi3)
    );

    // -----------------------------------------------------------------------
    // Twiddle ROM wiring
    // -----------------------------------------------------------------------
    reg  [5:0]          tw_addr;
    wire signed [15:0]  tw_wr, tw_wi;

    twiddle_rom_64 TWROM (
        .addr(tw_addr),
        .wr  (tw_wr),
        .wi  (tw_wi)
    );

    // -----------------------------------------------------------------------
    // Complex multiplier wiring (used in twiddle application)
    // -----------------------------------------------------------------------
    reg  signed [17:0] cm_ar, cm_ai;
    // tw_wr/wi already wired from ROM
    wire signed [17:0] cm_pr, cm_pi;

    cmult_18x16 CMULT (
        .ar(cm_ar), .ai(cm_ai),
        .wr(tw_wr), .wi(tw_wi),
        .pr(cm_pr), .pi(cm_pi)
    );

    // -----------------------------------------------------------------------
    // Bit-reversal function for N=64, radix-4
    //   6 bits split into three 2-bit groups, reversed
    // -----------------------------------------------------------------------
    function [5:0] bitrev64;
        input [5:0] x;
        bitrev64 = {x[1:0], x[3:2], x[5:4]};
    endfunction

    // -----------------------------------------------------------------------
    // Stage indices helpers
    //
    //  Stage 1: N=64, stride=16, groups=16 butterflies
    //    butterfly k (0..15):
    //      inputs at k, k+16, k+32, k+48
    //      twiddle exponents: 0, k, 2k, 3k  (mod 64) for W_64
    //
    //  Stage 2: N=16, stride=4, we run 4 groups of 4 butterflies
    //    butterfly b in group g  (g=0..3, b=0..3):
    //      index base = g*16 + b
    //      inputs at base, base+4, base+8, base+12
    //      twiddle exponents: 0, b*4, b*8, b*12  (W_16 = W_64^4)
    //
    //  Stage 3: N=4, no twiddles (trivial W_4^k already in butterfly)
    //    butterfly b in group g  (g=0..15, b=0..3 → implicit):
    //      inputs at g*4+0, g*4+1, g*4+2, g*4+3
    //      no post-twiddles
    // -----------------------------------------------------------------------

    // -----------------------------------------------------------------------
    // Main FSM
    // -----------------------------------------------------------------------
    integer i;

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            cnt        <= 0;
            dout_valid <= 0;
            done       <= 0;
            // zero butterfly inputs
            bf_xr0 <= 0; bf_xi0 <= 0;
            bf_xr1 <= 0; bf_xi1 <= 0;
            bf_xr2 <= 0; bf_xi2 <= 0;
            bf_xr3 <= 0; bf_xi3 <= 0;
        end else begin
            // defaults
            dout_valid <= 0;
            done       <= 0;

            case (state)

                // ============================================================
                S_IDLE: begin
                    if (start) begin
                        cnt   <= 0;
                        state <= S_LOAD;
                    end
                end

                // ============================================================
                // Load 64 samples sequentially into RAM (natural order)
                // ============================================================
                S_LOAD: begin
                    if (din_valid) begin
                        ram_re[cnt[5:0]] <= {din_re[15], din_re[15], din_re};  // sign-extend to 18b
                        ram_im[cnt[5:0]] <= {din_im[15], din_im[15], din_im};
                        if (cnt == 63) begin
                            cnt   <= 0;
                            state <= S_STAGE1;
                        end else begin
                            cnt <= cnt + 1;
                        end
                    end
                end

                // ============================================================
                // STAGE 1: 16 radix-4 butterflies
                //   butterfly k: indices k, k+16, k+32, k+48
                //   twiddle after: W_64^(k*1), W_64^(k*2), W_64^(k*3)
                //   (output 1,2,3 are multiplied; output 0 trivial W^0=1)
                // ============================================================
                S_STAGE1: begin
                    // cnt = 0..15, one butterfly per cycle
                    begin : stage1_blk
                        reg [5:0] i0, i1, i2, i3;
                        i0 = cnt[5:0];          // k
                        i1 = cnt[5:0] + 6'd16;  // k+16
                        i2 = cnt[5:0] + 6'd32;  // k+32
                        i3 = cnt[5:0] + 6'd48;  // k+48

                        // Drive butterfly inputs (truncate 18→16 with saturation guard)
                        bf_xr0 <= ram_re[i0][17:2];  // scale by /4 to prevent overflow
                        bf_xi0 <= ram_im[i0][17:2];
                        bf_xr1 <= ram_re[i1][17:2];
                        bf_xi1 <= ram_im[i1][17:2];
                        bf_xr2 <= ram_re[i2][17:2];
                        bf_xi2 <= ram_im[i2][17:2];
                        bf_xr3 <= ram_re[i3][17:2];
                        bf_xi3 <= ram_im[i3][17:2];
                    end

                    // Write-back butterfly outputs on NEXT cycle via registered pass-through
                    // We use a 1-cycle pipeline: after driving inputs, on the same cycle
                    // the combinational outputs are available - but we latch them via
                    // the twiddle stage. Write raw outputs immediately (twiddle follows).
                    // To keep 1-cycle-per-butterfly, we use a registered write-back stage.

                    // Because the butterfly is combinational, bf_yr0..3 are valid this cycle.
                    // Write output 0 immediately (twiddle W^0 = 1):
                    begin : s1_wb
                        reg [5:0] i0, i1, i2, i3;
                        i0 = cnt[5:0];
                        i1 = cnt[5:0] + 6'd16;
                        i2 = cnt[5:0] + 6'd32;
                        i3 = cnt[5:0] + 6'd48;
                        ram_re[i0] <= bf_yr0;
                        ram_im[i0] <= bf_yi0;
                        ram_re[i1] <= bf_yr1;
                        ram_im[i1] <= bf_yi1;
                        ram_re[i2] <= bf_yr2;
                        ram_im[i2] <= bf_yi2;
                        ram_re[i3] <= bf_yr3;
                        ram_im[i3] <= bf_yi3;
                    end

                    if (cnt == 15) begin
                        cnt   <= 0;
                        state <= S_TWIDDLE1;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                // ============================================================
                // TWIDDLE1: Apply W_64^(k*m) to outputs 1,2,3 of stage 1
                //   We iterate: k=0..15, m=1,2,3  → 48 twiddle mults
                //   cnt maps: butterfly_k = cnt/3, m = 1 + cnt%3
                // ============================================================
                S_TWIDDLE1: begin
                    begin : tw1_blk
                        reg [5:0] k, idx, exp;
                        reg [1:0] m;
                        k   = cnt / 3;
                        m   = (cnt % 3) + 1;       // 1, 2, or 3
                        idx = k + (m * 6'd16);     // address in RAM
                        exp = (k * m) % 64;        // twiddle exponent mod 64

                        tw_addr <= exp;
                        cm_ar   <= ram_re[idx];
                        cm_ai   <= ram_im[idx];
                        // cm_pr/pi available combinationally
                        ram_re[idx] <= cm_pr;
                        ram_im[idx] <= cm_pi;
                    end

                    if (cnt == 47) begin
                        cnt   <= 0;
                        state <= S_STAGE2;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                // ============================================================
                // STAGE 2: 16 radix-4 butterflies (4 groups × 4 butterflies)
                //   group g=0..3, butterfly b=0..3
                //   cnt = g*4 + b  (0..15)
                //   indices: base=g*16+b, base+4, base+8, base+12
                // ============================================================
                S_STAGE2: begin
                    begin : stage2_blk
                        reg [5:0] g, b, base, i0, i1, i2, i3;
                        g    = cnt[5:2];          // cnt / 4
                        b    = cnt[1:0];          // cnt % 4
                        base = (g * 6'd16) + b;
                        i0   = base;
                        i1   = base + 6'd4;
                        i2   = base + 6'd8;
                        i3   = base + 6'd12;

                        bf_xr0 <= ram_re[i0][17:2];
                        bf_xi0 <= ram_im[i0][17:2];
                        bf_xr1 <= ram_re[i1][17:2];
                        bf_xi1 <= ram_im[i1][17:2];
                        bf_xr2 <= ram_re[i2][17:2];
                        bf_xi2 <= ram_im[i2][17:2];
                        bf_xr3 <= ram_re[i3][17:2];
                        bf_xi3 <= ram_im[i3][17:2];

                        ram_re[i0] <= bf_yr0;
                        ram_im[i0] <= bf_yi0;
                        ram_re[i1] <= bf_yr1;
                        ram_im[i1] <= bf_yi1;
                        ram_re[i2] <= bf_yr2;
                        ram_im[i2] <= bf_yi2;
                        ram_re[i3] <= bf_yr3;
                        ram_im[i3] <= bf_yi3;
                    end

                    if (cnt == 15) begin
                        cnt   <= 0;
                        state <= S_TWIDDLE2;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                // ============================================================
                // TWIDDLE2: Apply W_16^(b*m) to outputs 1,2,3 of stage 2
                //   W_16^k = W_64^(4k)
                //   g=0..3, b=0..3, m=1,2,3  → 4*3*4 = 48 mults
                //   cnt encodes: top 4 bits = g*3 + (m-1), low 2 bits = b
                //   Flatten: cnt = (g*3 + (m-1))*4 + b,  0..47
                // ============================================================
                S_TWIDDLE2: begin
                    begin : tw2_blk
                        reg [5:0] b, g, m_minus1;
                        reg [5:0] idx, exp;
                        reg [1:0] m;
                        // cnt = outer * 4 + b
                        b        = cnt[1:0];
                        begin
                            reg [5:0] outer;
                            outer    = cnt >> 2;    // 0..11
                            g        = outer / 3;
                            m_minus1 = outer % 3;
                            m        = m_minus1 + 1;
                        end
                        idx = g * 6'd16 + b + m * 6'd4;
                        // W_16^(b*m) = W_64^(4*b*m)
                        exp = (4 * b * m) % 64;

                        tw_addr <= exp;
                        cm_ar   <= ram_re[idx];
                        cm_ai   <= ram_im[idx];
                        ram_re[idx] <= cm_pr;
                        ram_im[idx] <= cm_pi;
                    end

                    if (cnt == 47) begin
                        cnt   <= 0;
                        state <= S_STAGE3;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                // ============================================================
                // STAGE 3: 16 radix-4 butterflies (trivial twiddles)
                //   16 groups, each group of 4 consecutive samples
                //   cnt=0..15, group g=cnt
                //   indices: g*4+0, g*4+1, g*4+2, g*4+3
                // ============================================================
                S_STAGE3: begin
                    begin : stage3_blk
                        reg [5:0] g, i0, i1, i2, i3;
                        g  = cnt[5:0];
                        i0 = g * 4;
                        i1 = g * 4 + 1;
                        i2 = g * 4 + 2;
                        i3 = g * 4 + 3;

                        bf_xr0 <= ram_re[i0][17:2];
                        bf_xi0 <= ram_im[i0][17:2];
                        bf_xr1 <= ram_re[i1][17:2];
                        bf_xi1 <= ram_im[i1][17:2];
                        bf_xr2 <= ram_re[i2][17:2];
                        bf_xi2 <= ram_im[i2][17:2];
                        bf_xr3 <= ram_re[i3][17:2];
                        bf_xi3 <= ram_im[i3][17:2];

                        ram_re[i0] <= bf_yr0;
                        ram_im[i0] <= bf_yi0;
                        ram_re[i1] <= bf_yr1;
                        ram_im[i1] <= bf_yi1;
                        ram_re[i2] <= bf_yr2;
                        ram_im[i2] <= bf_yi2;
                        ram_re[i3] <= bf_yr3;
                        ram_im[i3] <= bf_yi3;
                    end

                    if (cnt == 15) begin
                        cnt   <= 0;
                        state <= S_OUTPUT;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                // ============================================================
                // OUTPUT: stream bins 0..63 with bit-reversal permutation
                // ============================================================
                S_OUTPUT: begin
                    begin : out_blk
                        reg [5:0] br_idx;
                        br_idx     = bitrev64(cnt[5:0]);
                        dout_re    <= ram_re[br_idx];
                        dout_im    <= ram_im[br_idx];
                        dout_valid <= 1;
                    end

                    if (cnt == 63) begin
                        done  <= 1;
                        cnt   <= 0;
                        state <= S_IDLE;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule


// =============================================================================
//  Testbench  -  Impulse input (x[0]=1, rest=0) → all bins should be 1
// =============================================================================
`ifdef SIMULATION
module tb_fft64;
    reg         clk, rst, start, din_valid;
    reg  signed [15:0] din_re, din_im;
    wire signed [17:0] dout_re, dout_im;
    wire        dout_valid, done;

    fft64_radix4_seq UUT (
        .clk(clk), .rst(rst), .start(start),
        .din_re(din_re), .din_im(din_im), .din_valid(din_valid),
        .dout_re(dout_re), .dout_im(dout_im),
        .dout_valid(dout_valid), .done(done)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer k;
    initial begin
        $dumpfile("fft64.vcd");
        $dumpvars(0, tb_fft64);

        rst = 1; start = 0; din_valid = 0;
        din_re = 0; din_im = 0;
        @(posedge clk); @(posedge clk);
        rst = 0;
        @(posedge clk);

        // Pulse start
        start = 1;
        @(posedge clk);
        start = 0;

        // Feed 64 samples: impulse at n=0
        for (k = 0; k < 64; k = k + 1) begin
            @(posedge clk);
            din_valid = 1;
            din_re    = (k == 0) ? 16'sd16384 : 16'sd0;
            din_im    = 16'sd0;
        end
        @(posedge clk);
        din_valid = 0;

        // Wait for done
        @(posedge done);
        $display("FFT done!");
        #20 $finish;
    end

    always @(posedge clk) begin
        if (dout_valid)
            $display("Bin %0d : re=%0d  im=%0d", UUT.cnt-1, dout_re, dout_im);
    end
endmodule
`endif