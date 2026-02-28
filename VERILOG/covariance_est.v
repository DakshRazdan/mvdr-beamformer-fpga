// ============================================================================
// covariance_est.v  —  4×4 Spatial Covariance Matrix Estimator
// ----------------------------------------------------------------------------
// Computes R(k) = alpha * R(k) + x(k) * x(k)^H  for each FFT bin k
//
// WHERE:
//   k       = frequency bin index (0..128, one-sided spectrum)
//   x(k)    = [X0(k), X1(k), X2(k), X3(k)]^T  — 4-mic FFT output vector
//   x(k)^H = conjugate transpose of x(k)
//   alpha   = forgetting factor = 0.95 (Q1.15: 0.95 × 32767 = 31129)
//
// MATRIX STRUCTURE (4×4 Hermitian):
//   R = | R00  R01  R02  R03 |    R[i][j] = sum over frames of Xi * conj(Xj)
//       | R10  R11  R12  R13 |    Hermitian: R[j][i] = conj(R[i][j])
//       | R20  R21  R22  R23 |    Diagonal R[i][i] is always real
//       | R30  R31  R32  R33 |
//
// STORAGE:
//   Full 4×4 = 16 complex elements per bin
//   129 bins × 16 elements × 2 (re+im) × 16 bits = 66KB — fits in BRAM
//   RAM address = bin * 16 + element_index (element = row*4 + col)
//
// DATA FORMAT: Q1.15 fixed point (same as FFT output)
//
// TIMING / DATA FLOW:
//   - Assumes all 4 FFTs output bin k simultaneously each cycle
//   - One bin processed per clock cycle (16 multiply-accumulate ops pipelined)
//   - Full matrix update for all 129 bins = 129 cycles per FFT frame
//   - FFT frame period = 8ms → 129 × 10ns = 1.29us << 8ms (plenty of margin)
//
// PIPELINE (per element R[i][j]):
//   Cycle 0: Read R[i][j] from RAM, compute Xi * conj(Xj)
//   Cycle 1: alpha * R[i][j]  +  Xi * conj(Xj)  →  new R[i][j]
//   Cycle 2: Write new R[i][j] back to RAM
// ============================================================================

module covariance_est #(
    parameter NBINS = 129,    // One-sided spectrum bins (0..128)
    parameter NMICS = 4,      // Number of microphones
    parameter DW    = 16,     // Data width (Q1.15)
    // Forgetting factor alpha = 0.95 in Q1.15
    // 0.95 * 32767 = 31129
    parameter signed [DW-1:0] ALPHA = 16'd31129
)(
    input  wire         clk,
    input  wire         rst_n,

    // Input: 4 complex FFT bins (one bin index, all 4 mics simultaneously)
    input  wire signed [DW-1:0] x0_re,
    input  wire signed [DW-1:0] x0_im,
    input  wire signed [DW-1:0] x1_re,
    input  wire signed [DW-1:0] x1_im,
    input  wire signed [DW-1:0] x2_re,
    input  wire signed [DW-1:0] x2_im,
    input  wire signed [DW-1:0] x3_re,
    input  wire signed [DW-1:0] x3_im,

    input  wire [$clog2(NBINS)-1:0] x_bin,          // Which bin (0..128)
    input  wire                     x_valid,         // One pulse per bin

    // Output: covariance matrix element (for MVDR weight computation)
    // Read interface — MVDR module requests specific R[i][j] at bin k
    input  wire [$clog2(NBINS)-1:0]  rd_bin,        // Bin to read
    input  wire [3:0]                rd_elem,        // Element index (row*4+col)
    input  wire                      rd_en,          // Read enable
    output reg  signed [DW-1:0]      rd_re,          // R[i][j] real
    output reg  signed [DW-1:0]      rd_im,          // R[i][j] imag
    output reg                       rd_valid         // Read data valid (1-cycle latency)
);

// ============================================================================
// COVARIANCE RAM
// Address: bin * 16 + element (element = row*4 + col, 0..15)
// Each entry: 32 bits = 16-bit real + 16-bit imag
// Total entries: 129 * 16 = 2064
// ============================================================================

localparam RAM_DEPTH = NBINS * NMICS * NMICS;  // 129 * 16 = 2064
localparam ALEN      = $clog2(RAM_DEPTH);      // 11 bits

reg signed [DW-1:0] cov_re [0:RAM_DEPTH-1];
reg signed [DW-1:0] cov_im [0:RAM_DEPTH-1];

integer init_i;
initial begin
    for (init_i = 0; init_i < RAM_DEPTH; init_i = init_i + 1) begin
        cov_re[init_i] = 0;
        cov_im[init_i] = 0;
    end
end

// ============================================================================
// UPDATE STATE MACHINE
// When x_valid pulses, we update all 16 elements R[i][j] for bin x_bin
// We process one element per clock cycle: 16 cycles to update one bin
// ============================================================================

reg        updating;
reg [3:0]  elem_cnt;       // 0..15 (row*4 + col)
reg [$clog2(NBINS)-1:0] cur_bin;


// Latch inputs when x_valid arrives

reg signed [DW-1:0] lx_re [0:NMICS-1];
reg signed [DW-1:0] lx_im [0:NMICS-1];


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        updating <= 0;
        elem_cnt <= 0;
        cur_bin  <= 0;
    end else begin
        if (x_valid && !updating) begin

            // Latch all 4 mic inputs

            lx_re[0] <= x0_re; lx_im[0] <= x0_im;
            lx_re[1] <= x1_re; lx_im[1] <= x1_im;
            lx_re[2] <= x2_re; lx_im[2] <= x2_im;
            lx_re[3] <= x3_re; lx_im[3] <= x3_im;

            cur_bin  <= x_bin;
            elem_cnt <= 0;
            updating <= 1;

        end else if (updating) begin
            if (elem_cnt == 15)
                updating <= 0;
            else
                elem_cnt <= elem_cnt + 1;
        end
    end
end


// ============================================================================
// ELEMENT COMPUTATION
// For element (i,j) = elem_cnt:
//   row i = elem_cnt[3:2]  (bits 3:2 = elem / 4)
//   col j = elem_cnt[1:0]  (bits 1:0 = elem mod 4)
//
// Xi * conj(Xj) = (xi_re + j*xi_im)(xj_re - j*xj_im)
//              = (xi_re*xj_re + xi_im*xj_im)
//                + j*(xi_im*xj_re - xi_re*xj_im)
//
// Then: new_R = (alpha * old_R  +  Xi*conj(Xj)) >> 15
//   alpha is Q1.15, old_R is Q1.15 → product is Q2.30 → >> 15 gives Q1.15
//   Xi*conj(Xj): Q1.15 * Q1.15 = Q2.30 → >> 15 gives Q1.15
// ============================================================================

wire [1:0] row_idx = elem_cnt[3:2];
wire [1:0] col_idx = elem_cnt[1:0];

wire signed [DW-1:0] xi_re = lx_re[row_idx];
wire signed [DW-1:0] xi_im = lx_im[row_idx];
wire signed [DW-1:0] xj_re = lx_re[col_idx];
wire signed [DW-1:0] xj_im = lx_im[col_idx];


// RAM address for current element

wire [ALEN-1:0] wr_addr = cur_bin * 16 + elem_cnt;


// Read current R[i][j]

wire signed [DW-1:0] old_re = cov_re[wr_addr];
wire signed [DW-1:0] old_im = cov_im[wr_addr];


// Xi * conj(Xj) — Q2.30 intermediate

wire signed [2*DW-1:0] xij_re_full =
    (xi_re * xj_re) + (xi_im * xj_im);

wire signed [2*DW-1:0] xij_im_full =
    (xi_im * xj_re) - (xi_re * xj_im);


// Truncate to Q1.15

wire signed [DW-1:0] xij_re =
    $signed(xij_re_full) >>> 15;

wire signed [DW-1:0] xij_im =
    $signed(xij_im_full) >>> 15;


// alpha * old_R — Q2.30 intermediate

wire signed [2*DW-1:0] alpha_re_full =
    ALPHA * old_re;

wire signed [2*DW-1:0] alpha_im_full =
    ALPHA * old_im;

wire signed [DW-1:0] alpha_re =
    $signed(alpha_re_full) >>> 15;

wire signed [DW-1:0] alpha_im =
    $signed(alpha_im_full) >>> 15;


// New R[i][j] = alpha * old + Xi*conj(Xj)

wire signed [DW-1:0] new_re =
    alpha_re + xij_re;

wire signed [DW-1:0] new_im =
    alpha_im + xij_im;


// ============================================================================
// RAM WRITE (registered)
// ============================================================================

always @(posedge clk) begin
    if (updating) begin
        cov_re[wr_addr] <= new_re;
        cov_im[wr_addr] <= new_im;
    end
end


// ============================================================================
// READ PORT (for MVDR weight computation)
// 1-cycle latency registered read
// ============================================================================

wire [ALEN-1:0] rd_addr = rd_bin * 16 + rd_elem;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_re    <= 0;
        rd_im    <= 0;
        rd_valid <= 0;
    end else begin
        rd_valid <= rd_en;
        if (rd_en) begin
            rd_re <= cov_re[rd_addr];
            rd_im <= cov_im[rd_addr];
        end
    end
end

endmodule