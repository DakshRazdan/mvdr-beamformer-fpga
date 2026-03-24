// spectral_sub.v — Spectral Subtraction Noise Reduction

`timescale 1ns/1ps

module spectral_sub #(
    parameter NBINS = 129,
    parameter DW = 16
)(
    input wire clk,
    input wire rst_n,
    input wire signed [DW-1:0] x_re,
    input wire signed [DW-1:0] x_im,
    input wire [7:0] x_bin,
    input wire x_valid,
    input wire speech,
    output reg signed [DW-1:0]  y_re,
    output reg signed [DW-1:0]  y_im,
    output reg [7:0] y_bin,
    output reg y_valid
);

// Noise PSD estimate per bin (magnitude, Q1.15)
reg [DW-1:0] noise_mag [0:NBINS-1];
// Frame count for averaging
reg [7:0] noise_frames;

integer ni;
initial begin
    for (ni = 0; ni < NBINS; ni = ni + 1)
        noise_mag[ni] = 0;
    noise_frames = 0;
end

// Magnitude computation: approx sqrt(re^2+im^2) using max+min/2
wire [31:0] re2 = ($signed(x_re) * $signed(x_re)) >>> 15;
wire [31:0] im2 = ($signed(x_im) * $signed(x_im)) >>> 15;
wire [31:0] mag2 = re2 + im2;
// Integer sqrt approximation: mag ~ (max + min*0.5)
wire [DW-1:0] abs_re = x_re[DW-1] ? -x_re : x_re;
wire [DW-1:0] abs_im = x_im[DW-1] ? -x_im : x_im;
wire [DW-1:0] mag_approx = (abs_re > abs_im) ?
    abs_re + (abs_im >>> 1) :
    abs_im + (abs_re >>> 1);

// Alpha = 1.5 in Q1.15 needs Q2.15 — use 32-bit
localparam signed [31:0] ALPHA = 32'd49152;  // 1.5 * 32768
localparam signed [DW-1:0] BETA = 16'd328;   // 0.01 * 32768

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        y_valid <= 0; y_re <= 0; y_im <= 0; y_bin <= 0;
        noise_frames <= 0;
        for (ni = 0; ni < NBINS; ni = ni + 1)
            noise_mag[ni] <= 0;
    end else begin
        y_valid <= 0;
        if (x_valid && x_bin < NBINS) begin
            // Update noise estimate during silence
            if (!speech) begin
                if (noise_frames < 255)
                    noise_mag[x_bin] <= (noise_mag[x_bin] + mag_approx) >>> 1;
                if (x_bin == NBINS-1 && noise_frames < 255)
                    noise_frames <= noise_frames + 1;
            end

            // Spectral subtraction
            begin : sub
                reg [DW-1:0] noise_est, mag_sub, mag_floor;
                reg signed [31:0] sub_val;
                reg signed [DW-1:0] scale;

                noise_est = noise_mag[x_bin];
                // alpha * noise
                sub_val = (ALPHA * noise_est) >>> 15;
                // floor = beta * mag
                mag_floor = ($signed(BETA) * $signed(mag_approx)) >>> 15;
                // subtracted magnitude
                mag_sub = (mag_approx > sub_val[15:0]) ?
                        (mag_approx - sub_val[15:0]) : mag_floor;
                if (mag_sub < mag_floor) mag_sub = mag_floor;

                // Scale re and im by mag_sub/mag_approx to preserve phase
                if (mag_approx > 0) begin
                    y_re <= ($signed(x_re) * $signed(mag_sub)) / $signed(mag_approx);
                    y_im <= ($signed(x_im) * $signed(mag_sub)) / $signed(mag_approx);
                end else begin
                    y_re <= 0; y_im <= 0;
                end
            end

            y_bin   <= x_bin;
            y_valid <= 1;
        end
    end
end
endmodule