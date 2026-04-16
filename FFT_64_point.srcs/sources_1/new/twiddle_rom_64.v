// =============================================================================
// Twiddle Factor ROM for 64-point FFT
//
// Returns W_N^k = cos(2πk/N) - j·sin(2πk/N) in Q1.15 fixed-point format
//
// Stores a 64-entry lookup table for W_64^0 through W_64^63.
// These twiddle factors are used across all three stages of the radix-4 FFT:
//   - Stage 1: W_64^k, k = 0..15
//   - Stage 2: W_16^k = W_64^(4k), k = 0..3
//   - Stage 3: Trivial (no twiddles needed)
//
// All cos/sin values are scaled by 32767 (0x7FFF) for Q1.15 precision.
// =============================================================================
`timescale 1ns / 1ps

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