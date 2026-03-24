// freq_shaper.v — 16-tap FIR Frequency Shaper (1-5kHz boost, 6dB)
`timescale 1ns/1ps
module freq_shaper #(
    parameter DW   = 16,
    parameter TAPS = 16
)(
    input wire clk,
    input wire rst_n,
    input wire signed [DW-1:0] pcm_in,
    input wire pcm_valid,
    output reg signed [DW-1:0] pcm_out,
    output reg pcm_out_valid
);

// FIR coefficients Q1.15
// Flat passthrough + 6dB boost at 1-5kHz
wire signed [DW-1:0] coeff [0:TAPS-1];
assign coeff[0] = -16'sd96;
assign coeff[1] = -16'sd187;
assign coeff[2] = -16'sd243;
assign coeff[3] = -16'sd187;
assign coeff[4] = 16'sd121;
assign coeff[5] = 16'sd609;
assign coeff[6] = 16'sd1097;
assign coeff[7] = 16'sd1389;
assign coeff[8] = 16'sd1389;
assign coeff[9] = 16'sd1097;
assign coeff[10] = 16'sd609;
assign coeff[11] = 16'sd121;
assign coeff[12] = -16'sd187;
assign coeff[13] = -16'sd243;
assign coeff[14] = -16'sd187;
assign coeff[15] = -16'sd96;

// Delay line
reg signed [DW-1:0] dline [0:TAPS-1];
integer di;
initial begin
    for (di = 0; di < TAPS; di = di + 1)
        dline[di] = 0;
end

// Shift delay line
always @(posedge clk) begin
    if (pcm_valid) begin : shift
        integer j;
        for (j = TAPS-1; j > 0; j = j - 1)
            dline[j] <= dline[j-1];
        dline[0] <= pcm_in;
    end
end

// FIR output
reg signed [31:0] fir_acc;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pcm_out <= 0; pcm_out_valid <= 0; fir_acc <= 0;
    end else begin
        pcm_out_valid <= pcm_valid;
        if (pcm_valid) begin : fir_compute
            integer k;
            reg signed [31:0] acc;
            acc = 0;
            for (k = 0; k < TAPS; k = k + 1)
                acc = acc + (($signed(coeff[k]) * $signed(dline[k])) >>> 15);
            // Saturate
            if (acc > 32767) pcm_out <= 16'sd32767;
            else if (acc < -32768) pcm_out <= -16'sd32768;
            else pcm_out <= acc[DW-1:0];
        end
    end
end
endmodule