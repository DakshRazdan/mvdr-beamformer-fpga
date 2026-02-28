// ============================================================================
// covariance_est.v  —  4×4 Spatial Covariance Matrix Estimator
// ----------------------------------------------------------------------------
// R(k) = alpha * R(k) + x(k) * x(k)^H  per FFT bin k
// alpha = 0.95 (Q1.15: 31129)
// 4x4 Hermitian matrix, 16 complex elements per bin
// 129 bins × 16 elements = 2064 RAM entries
// ============================================================================

module covariance_est #(
    parameter NBINS = 129,
    parameter NMICS = 4,
    parameter DW    = 16,
    parameter signed [15:0] ALPHA = 16'd31129
)(
    input  wire         clk,
    input  wire         rst_n,

    // 4 mic FFT inputs (flat ports — no arrays at top level)
    input  wire signed [DW-1:0] x0_re, x0_im,
    input  wire signed [DW-1:0] x1_re, x1_im,
    input  wire signed [DW-1:0] x2_re, x2_im,
    input  wire signed [DW-1:0] x3_re, x3_im,

    input  wire [7:0]  x_bin,    // bin index 0..128
    input  wire        x_valid,

    // Read port for MVDR module
    input  wire [7:0]  rd_bin,
    input  wire [3:0]  rd_elem,
    input  wire        rd_en,
    output reg  signed [DW-1:0] rd_re,
    output reg  signed [DW-1:0] rd_im,
    output reg                  rd_valid
);

// RAM: 2064 entries
localparam RAM_DEPTH = 2064;

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
// LATCH + FSM
// ============================================================================
reg        updating;
reg [3:0]  elem_cnt;
reg [7:0]  cur_bin;

reg signed [DW-1:0] lx_re_0, lx_re_1, lx_re_2, lx_re_3;
reg signed [DW-1:0] lx_im_0, lx_im_1, lx_im_2, lx_im_3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        updating <= 0;
        elem_cnt <= 0;
        cur_bin  <= 0;
    end else begin
        if (x_valid && !updating) begin
            lx_re_0 <= x0_re; lx_im_0 <= x0_im;
            lx_re_1 <= x1_re; lx_im_1 <= x1_im;
            lx_re_2 <= x2_re; lx_im_2 <= x2_im;
            lx_re_3 <= x3_re; lx_im_3 <= x3_im;
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
// ELEMENT MUX (explicit — iverilog cannot index reg arrays with variable wire)
// row_idx = elem_cnt[3:2], col_idx = elem_cnt[1:0]
// ============================================================================
wire [1:0] row_idx = elem_cnt[3:2];
wire [1:0] col_idx = elem_cnt[1:0];

wire signed [DW-1:0] xi_re = (row_idx==0) ? lx_re_0 :
                              (row_idx==1) ? lx_re_1 :
                              (row_idx==2) ? lx_re_2 : lx_re_3;

wire signed [DW-1:0] xi_im = (row_idx==0) ? lx_im_0 :
                              (row_idx==1) ? lx_im_1 :
                              (row_idx==2) ? lx_im_2 : lx_im_3;

wire signed [DW-1:0] xj_re = (col_idx==0) ? lx_re_0 :
                              (col_idx==1) ? lx_re_1 :
                              (col_idx==2) ? lx_re_2 : lx_re_3;

wire signed [DW-1:0] xj_im = (col_idx==0) ? lx_im_0 :
                              (col_idx==1) ? lx_im_1 :
                              (col_idx==2) ? lx_im_2 : lx_im_3;

// ============================================================================
// BUTTERFLY MATH
// ============================================================================
wire [10:0] wr_addr = cur_bin * 16 + elem_cnt;

wire signed [DW-1:0] old_re = cov_re[wr_addr];
wire signed [DW-1:0] old_im = cov_im[wr_addr];

// Xi * conj(Xj)
wire signed [2*DW-1:0] xij_re_full = (xi_re * xj_re) + (xi_im * xj_im);
wire signed [2*DW-1:0] xij_im_full = (xi_im * xj_re) - (xi_re * xj_im);
wire signed [DW-1:0]   xij_re      = $signed(xij_re_full) >>> 15;
wire signed [DW-1:0]   xij_im      = $signed(xij_im_full) >>> 15;

// alpha * old_R
wire signed [2*DW-1:0] alpha_re_full = ALPHA * old_re;
wire signed [2*DW-1:0] alpha_im_full = ALPHA * old_im;
wire signed [DW-1:0]   alpha_re      = $signed(alpha_re_full) >>> 15;
wire signed [DW-1:0]   alpha_im      = $signed(alpha_im_full) >>> 15;

// New R = alpha * old + Xi*conj(Xj)
wire signed [DW-1:0] new_re = alpha_re + xij_re;
wire signed [DW-1:0] new_im = alpha_im + xij_im;

// ============================================================================
// RAM WRITE
// ============================================================================
always @(posedge clk) begin
    if (updating) begin
        cov_re[wr_addr] <= new_re;
        cov_im[wr_addr] <= new_im;
    end
end

// ============================================================================
// READ PORT
// ============================================================================
wire [10:0] rd_addr = rd_bin * 16 + rd_elem;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_re <= 0; rd_im <= 0; rd_valid <= 0;
    end else begin
        rd_valid <= rd_en;
        if (rd_en) begin
            rd_re <= cov_re[rd_addr];
            rd_im <= cov_im[rd_addr];
        end
    end
end

endmodule