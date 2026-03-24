// compressor.v — Dynamic Range Compressor
`timescale 1ns/1ps

module compressor #(
    parameter DW = 16,
    parameter THRESHOLD = 16'd9830,   // 0.3 * 32767
    parameter MAKEUP = 32'd49152   // 1.5 * 32768 (Q2.15)
)(
    input wire clk,
    input wire rst_n,
    input wire signed [DW-1:0] pcm_in,
    input wire pcm_valid,
    output reg signed [DW-1:0] pcm_out,
    output reg pcm_out_valid
);

wire signed [DW-1:0] abs_in = pcm_in[DW-1] ? -pcm_in : pcm_in;
wire above   = (abs_in > $signed(THRESHOLD));

// Compressed value: threshold + (abs - threshold) / 4
wire signed [DW-1:0] comp_abs = THRESHOLD + ((abs_in - THRESHOLD) >>> 2);
wire signed [DW-1:0] comp_val = pcm_in[DW-1] ? -comp_abs : comp_abs;

// Apply makeup gain
wire signed [31:0] gained = above ?
    ($signed(comp_val) * $signed(MAKEUP)) >>> 15 :
    ($signed(pcm_in) * $signed(MAKEUP)) >>> 15;

// Saturate
wire signed [DW-1:0] sat_out =
    (gained > 32767) ?  16'sd32767 :
    (gained < -32768) ? -16'sd32768 : gained[DW-1:0];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pcm_out <= 0; pcm_out_valid <= 0;
    end else begin
        pcm_out_valid <= pcm_valid;
        if (pcm_valid) pcm_out <= sat_out;
    end
end
endmodule