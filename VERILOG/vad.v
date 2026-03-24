
// vad.v — Voice Activity Detection

`timescale 1ns/1ps

module vad #(
    parameter DW = 16,
    parameter FRAME_LEN = 320,
    parameter HOP = 128,
    parameter ENERGY_TH = 32'd328,
    parameter ZCR_LOW = 32'd655,
    parameter ZCR_HIGH = 32'd14746
)(
    input wire clk,
    input wire rst_n,
    input wire signed [DW-1:0] pcm_in,
    input wire pcm_valid,
    output reg speech
);

reg signed [DW-1:0] frame_buf [0:FRAME_LEN-1];
reg [8:0] hop_cnt;
reg [8:0] fill_cnt;
reg buf_full;

integer fi;
initial begin
    for (fi = 0; fi < FRAME_LEN; fi = fi + 1)
        frame_buf[fi] = 0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        hop_cnt <= 0;
        fill_cnt <= 0;
        buf_full <= 0;
    end else if (pcm_valid) begin
        begin : shift
            integer k;
            for (k = 0; k < FRAME_LEN-1; k = k + 1)
                frame_buf[k] <= frame_buf[k+1];
            frame_buf[FRAME_LEN-1] <= pcm_in;
        end
        if (fill_cnt < FRAME_LEN) fill_cnt <= fill_cnt + 1;
        else buf_full <= 1;
        hop_cnt <= (hop_cnt == HOP-1) ? 0 : hop_cnt + 1;
    end
end

reg [8:0] comp_cnt;
reg [31:0] energy_acc;
reg [31:0] zcr_acc;
reg signed [DW-1:0] prev_samp;
reg computing;

wire signed [DW-1:0] cur_samp = frame_buf[comp_cnt];
wire zcr_cross = cur_samp[DW-1] ^ prev_samp[DW-1];
wire [31:0] samp_sq = ($signed(cur_samp) * $signed(cur_samp)) >>> 15;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        comp_cnt <= 0; energy_acc <= 0;
        zcr_acc <= 0; prev_samp <= 0;
        computing <= 0; speech <= 0;
    end else begin
        if (pcm_valid && buf_full && hop_cnt == HOP-1) begin
            computing <= 1; comp_cnt <= 0;
            energy_acc <= 0; zcr_acc <= 0;
            prev_samp <= frame_buf[0];
        end
        if (computing) begin
            energy_acc <= energy_acc + samp_sq;
            if (zcr_cross) zcr_acc <= zcr_acc + 1;
            prev_samp <= cur_samp;
            if (comp_cnt == FRAME_LEN-1) begin
                computing <= 0;
                begin : decision
                    reg [31:0] ne, nz;
                    ne = energy_acc / FRAME_LEN;
                    nz = (zcr_acc * 32767) / FRAME_LEN;
                    speech <= (ne > ENERGY_TH && nz > ZCR_LOW && nz < ZCR_HIGH) ? 1 : 0;
                end
            end else
                comp_cnt <= comp_cnt + 1;
        end
    end
end
endmodule