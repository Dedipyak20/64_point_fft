// =============================================================================
// Complex Multiplier: (ar + j*ai) * (wr + j*wi)
//
// Multiplies an 18-bit complex data value by a 16-bit twiddle factor.
// The twiddle factors are in Q1.15 fixed-point format.
// Output is truncated back to 18 bits by right-shifting 15 positions.
//
// Inputs:  ar, ai (18-bit data), wr, wi (16-bit Q1.15 twiddle)
// Outputs: pr, pi (18-bit truncated result)
// =============================================================================
`timescale 1ns / 1ps

module cmult_18x16 (
    input  signed [17:0] ar, ai,
    input  signed [15:0] wr, wi,
    output signed [17:0] pr, pi
);
    wire signed [33:0] rr = ar * wr;
    wire signed [33:0] ii = ai * wi;
    wire signed [33:0] ri = ar * wi;
    wire signed [33:0] ir = ai * wr;

    // Q1.15 twiddle × Q1.0-ish data → shift right 15 to re-align
    assign pr = (rr - ii) >>> 15;
    assign pi = (ri + ir) >>> 15;
endmodule