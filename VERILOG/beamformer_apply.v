// ============================================================================
// beamformer_apply.v  —  Apply MVDR weights to FFT bins
// ----------------------------------------------------------------------------
// Computes: Y(k) = w^H(k) · X(k)
//   = w0*·x0 + w1*·x1 + w2*·x2 + w3*·x3
//
// w^H = conjugate transpose: (a+jb)* = (a-jb)
// So each term: (w_re - j*w_im)(x_re + j*x_im)
//             = (w_re*x_re + w_im*x_im) + j(w_re*x_im - w_im*x_re)
//
// ARCHITECTURE:
//   - Weight RAM: 129 entries, written as mvdr_weights fires w_valid per bin
//     8 separate 1D arrays (one per component) — avoids iverilog 2D array bug
//   - Apply: when x_valid fires, read weights for that bin, compute in 1 cycle
//
// TIMING:
//   - Weight write: 1 cycle per bin (combinational store on w_valid)
//   - Apply: x_valid in → y_valid out 2 cycles later (read + multiply/add)
//
// FIXED POINT:
//   - All inputs Q1.15 (16-bit signed)
//   - Internal: Q2.30 (32-bit) for products, accumulated to Q2.30
//   - Output: truncate back to Q1.15 (shift right 15)
// ============================================================================

`timescale 1ns/1ps

module beamformer_apply #(
    parameter NBINS = 129,
    parameter DW    = 16
)(
    input  wire clk,
    input  wire rst_n,

    // Weight input port (from mvdr_weights)
    input  wire signed [DW-1:0] w0_re, w0_im,
    input  wire signed [DW-1:0] w1_re, w1_im,
    input  wire signed [DW-1:0] w2_re, w2_im,
    input  wire signed [DW-1:0] w3_re, w3_im,
    input  wire [7:0]  w_bin,
    input  wire        w_valid,

    // FFT bin input (4 mics in parallel, same bin same cycle)
    input  wire signed [DW-1:0] x0_re, x0_im,
    input  wire signed [DW-1:0] x1_re, x1_im,
    input  wire signed [DW-1:0] x2_re, x2_im,
    input  wire signed [DW-1:0] x3_re, x3_im,
    input  wire [7:0]  x_bin,
    input  wire        x_valid,

    // Beamformed output
    output reg  signed [DW-1:0] y_re,
    output reg  signed [DW-1:0] y_im,
    output reg  [7:0]  y_bin,
    output reg         y_valid
);

// ============================================================================
// WEIGHT RAM — 8 separate 1D arrays, one per component
// Avoids iverilog 2D array variable-index read bug (returns silent 0)
// ============================================================================
reg signed [DW-1:0] ram_w0r [0:NBINS-1];
reg signed [DW-1:0] ram_w0i [0:NBINS-1];
reg signed [DW-1:0] ram_w1r [0:NBINS-1];
reg signed [DW-1:0] ram_w1i [0:NBINS-1];
reg signed [DW-1:0] ram_w2r [0:NBINS-1];
reg signed [DW-1:0] ram_w2i [0:NBINS-1];
reg signed [DW-1:0] ram_w3r [0:NBINS-1];
reg signed [DW-1:0] ram_w3i [0:NBINS-1];

// ============================================================================
// WEIGHT WRITE — store on w_valid (single cycle, no FSM needed)
// ============================================================================
always @(posedge clk) begin
    if (w_valid) begin
        ram_w0r[w_bin] <= w0_re; ram_w0i[w_bin] <= w0_im;
        ram_w1r[w_bin] <= w1_re; ram_w1i[w_bin] <= w1_im;
        ram_w2r[w_bin] <= w2_re; ram_w2i[w_bin] <= w2_im;
        ram_w3r[w_bin] <= w3_re; ram_w3i[w_bin] <= w3_im;
    end
end

// ============================================================================
// PIPELINE STAGE 1 — register FFT inputs and read weights (1 cycle latency)
// ============================================================================
reg signed [DW-1:0] s1_x0r, s1_x0i;
reg signed [DW-1:0] s1_x1r, s1_x1i;
reg signed [DW-1:0] s1_x2r, s1_x2i;
reg signed [DW-1:0] s1_x3r, s1_x3i;
reg signed [DW-1:0] s1_w0r, s1_w0i;
reg signed [DW-1:0] s1_w1r, s1_w1i;
reg signed [DW-1:0] s1_w2r, s1_w2i;
reg signed [DW-1:0] s1_w3r, s1_w3i;
reg [7:0]  s1_bin;
reg        s1_valid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s1_valid <= 0;
    end else begin
        s1_valid <= x_valid;
        s1_bin   <= x_bin;
        // Latch FFT inputs
        s1_x0r <= x0_re; s1_x0i <= x0_im;
        s1_x1r <= x1_re; s1_x1i <= x1_im;
        s1_x2r <= x2_re; s1_x2i <= x2_im;
        s1_x3r <= x3_re; s1_x3i <= x3_im;
        // Read weights for this bin (1D array, variable index — safe in iverilog)
        s1_w0r <= ram_w0r[x_bin]; s1_w0i <= ram_w0i[x_bin];
        s1_w1r <= ram_w1r[x_bin]; s1_w1i <= ram_w1i[x_bin];
        s1_w2r <= ram_w2r[x_bin]; s1_w2i <= ram_w2i[x_bin];
        s1_w3r <= ram_w3r[x_bin]; s1_w3i <= ram_w3i[x_bin];
    end
end

// ============================================================================
// PIPELINE STAGE 2 — complex dot product w^H · x
//
// w^H_i · x_i = (w_re - j*w_im)(x_re + j*x_im)
//             = (w_re*x_re + w_im*x_im) + j(w_re*x_im - w_im*x_re)
//
// Q1.15 × Q1.15 = Q2.30 (32-bit), accumulate 4 terms, shift >>15 for Q1.15
// 4-term sum: max = 4 × 1.0 × 1.0 = 4.0 — use 34-bit accumulator to be safe
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        y_valid <= 0;
        y_re    <= 0;
        y_im    <= 0;
        y_bin   <= 0;
    end else begin
        y_valid <= s1_valid;
        y_bin   <= s1_bin;

        if (s1_valid) begin
            begin : dotprod
                // w_re*x_re + w_im*x_im  (conj: sign flip on w_im for real part)
                reg signed [31:0] t0r, t1r, t2r, t3r;
                // w_re*x_im - w_im*x_re  (conj: sign flip on w_im for imag part)
                reg signed [31:0] t0i, t1i, t2i, t3i;
                reg signed [33:0] sum_re, sum_im;

                // Term 0
                t0r = (($signed(s1_w0r) * $signed(s1_x0r)) +
                       ($signed(s1_w0i) * $signed(s1_x0i))) >>> 15;
                t0i = (($signed(s1_w0r) * $signed(s1_x0i)) -
                       ($signed(s1_w0i) * $signed(s1_x0r))) >>> 15;
                // Term 1
                t1r = (($signed(s1_w1r) * $signed(s1_x1r)) +
                       ($signed(s1_w1i) * $signed(s1_x1i))) >>> 15;
                t1i = (($signed(s1_w1r) * $signed(s1_x1i)) -
                       ($signed(s1_w1i) * $signed(s1_x1r))) >>> 15;
                // Term 2
                t2r = (($signed(s1_w2r) * $signed(s1_x2r)) +
                       ($signed(s1_w2i) * $signed(s1_x2i))) >>> 15;
                t2i = (($signed(s1_w2r) * $signed(s1_x2i)) -
                       ($signed(s1_w2i) * $signed(s1_x2r))) >>> 15;
                // Term 3
                t3r = (($signed(s1_w3r) * $signed(s1_x3r)) +
                       ($signed(s1_w3i) * $signed(s1_x3i))) >>> 15;
                t3i = (($signed(s1_w3r) * $signed(s1_x3i)) -
                       ($signed(s1_w3i) * $signed(s1_x3r))) >>> 15;

                // Accumulate — 34-bit to handle up to 4.0 in Q1.15
                sum_re = $signed(t0r) + $signed(t1r) +
                        $signed(t2r) + $signed(t3r);
                sum_im = $signed(t0i) + $signed(t1i) +
                        $signed(t2i) + $signed(t3i);

                // Saturate to Q1.15 range [-32768, 32767]
                if (sum_re > 34'sd32767)
                    y_re <= 16'sd32767;
                else if (sum_re < -34'sd32768)
                    y_re <= -16'sd32768;
                else
                    y_re <= sum_re[15:0];

                if (sum_im > 34'sd32767)
                    y_im <= 16'sd32767;
                else if (sum_im < -34'sd32768)
                    y_im <= -16'sd32768;
                else
                    y_im <= sum_im[15:0];
            end
        end
    end
end

endmodule