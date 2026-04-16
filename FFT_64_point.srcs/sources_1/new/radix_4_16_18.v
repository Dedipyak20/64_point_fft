`timescale 1ns / 1ps

module radix_4_16_18(
    input  signed [15:0] xr0, xi0,
    input  signed [15:0] xr1, xi1,
    input  signed [15:0] xr2, xi2,
    input  signed [15:0] xr3, xi3,
    
    //two bits extra for overflow prtection
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