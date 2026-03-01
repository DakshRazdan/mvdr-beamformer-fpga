// ============================================================================
// mvdr_weights.v  —  MVDR Weight Computation (per frequency bin)
// ----------------------------------------------------------------------------
// Computes: w(k) = R^{-1}(k) * d(k)  /  (d^H(k) * R^{-1}(k) * d(k))
//
// ALGORITHM: Gauss-Jordan elimination on augmented matrix [R | d]
//   - Reduces [R | d] to [I | R^{-1}*d] in 4 pivot steps
//   - Each pivot: divide pivot row, then eliminate from all other rows
//   - Final augmented column = unnormalized weights u = R^{-1}*d
//   - Normalize: w = u / (d^H * u)
//
// STEERING VECTOR d (target at 0°, broadside):
//   For a linear array with target directly in front, all phase delays = 0
//   d = [1+0j, 1+0j, 1+0j, 1+0j]  (real-valued, all ones)
//   This simplifies normalization: denominator = sum(u) (real part only)
//
// DIAGONAL LOADING (robustness):
//   Before inversion, R_loaded = R + delta*I  (delta = 0.5 in Q1.15 = 16384)
//   Prevents singular matrix from mic mismatch or low-signal conditions
//
// PRECISION:
//   Internal: 32-bit signed (Q2.30) for intermediate calculations
//   Output:   16-bit signed (Q1.15) weights
//   Division: behavioral (synthesis requires pipelined divider IP)
//
// TIMING:
//   - Reads 16 R elements from covariance RAM: 16 cycles
//   - Gauss-Jordan (4 pivots × ~50 cycles): ~200 cycles
//   - Output: 4 cycles
//   - Total: ~250 cycles per bin × 129 bins = ~32K cycles per frame
//   - At 100MHz: 320us << 8ms frame period (4% utilization)
// ============================================================================

module mvdr_weights #(
    parameter NBINS  = 129,
    parameter NMICS  = 4,
    parameter DW     = 16,      // I/O data width (Q1.15)
    parameter IW     = 32,      // Internal width (Q2.30)
    // Diagonal loading delta = 0.5 in Q2.30 (0.5 * 2^30 = 536870912)
    parameter signed [31:0] DELTA = 32'sd536870912
)(
    input  wire         clk,
    input  wire         rst_n,

    // Trigger: start weight computation for one bin
    input  wire         compute,
    input  wire [7:0]   bin_in,

    // Covariance RAM read interface
    output reg  [7:0]   rd_bin,
    output reg  [3:0]   rd_elem,
    output reg          rd_en,
    input  wire signed [DW-1:0] rd_re,
    input  wire signed [DW-1:0] rd_im,
    input  wire         rd_valid,

    // Output: 4 complex MVDR weights for this bin
    output reg  signed [DW-1:0] w0_re, w0_im,
    output reg  signed [DW-1:0] w1_re, w1_im,
    output reg  signed [DW-1:0] w2_re, w2_im,
    output reg  signed [DW-1:0] w3_re, w3_im,
    output reg  [7:0]   w_bin,
    output reg          w_valid
);

// ============================================================================
// AUGMENTED MATRIX STORAGE
// Aug[r][c] for r=0..3 (rows), c=0..4 (cols 0-3 = R, col 4 = d)
// Complex: _re and _im arrays
// Internal precision: Q2.30 (32-bit signed)
// ============================================================================
reg signed [IW-1:0] aug_re [0:3][0:4];
reg signed [IW-1:0] aug_im [0:3][0:4];

// ============================================================================
// STATE MACHINE
// ============================================================================
localparam ST_IDLE      = 4'd0;
localparam ST_READ_R    = 4'd1;   // Read 16 R elements from RAM
localparam ST_SETUP     = 4'd2;   // Add diagonal loading + set d column
localparam ST_PIVOT     = 4'd3;   // Divide pivot row by pivot element
localparam ST_ELIM      = 4'd4;   // Eliminate column from non-pivot rows
localparam ST_NORM      = 4'd5;   // Compute normalization denominator
localparam ST_OUTPUT    = 4'd6;   // Write weights to output registers

reg [3:0]  state;
reg [7:0]  cur_bin;

// Read control
reg [3:0]  read_cnt;    // 0..15, counts R elements being read
reg [3:0]  elem_latch;  // latch of elem index for valid signal

// Gauss-Jordan control
reg [1:0]  pivot_col;   // Current pivot column (0..3)
reg [1:0]  elim_row;    // Current row being eliminated
reg [2:0]  col_cnt;     // Column counter during elimination (0..4)

// Pivot element (complex, IW bits)
reg signed [IW-1:0] pivot_re, pivot_im;
reg signed [IW-1:0] pivot_mag2;  // |pivot|^2 for division

// Factor for current elimination row: factor = aug[elim_row][pivot_col] / pivot
reg signed [IW-1:0] factor_re, factor_im;

// Normalization
reg signed [IW-1:0] denom_re;  // d^H * u = sum of u[0..3] (d=[1,1,1,1])

integer i, j;

// ============================================================================
// COMPLEX MULTIPLY HELPER (combinational, 32-bit result from 32-bit inputs)
// a*b: (ar+jai)(br+jbi) = (ar*br - ai*bi) + j(ar*bi + ai*br)
// Uses 64-bit intermediate then shifts to keep Q2.30
// ============================================================================
function signed [IW-1:0] cmul_re;
    input signed [IW-1:0] ar, ai, br, bi;
    reg signed [63:0] prod;
    begin
        prod    = (ar * br) - (ai * bi);
        cmul_re = prod >>> 30;
    end
endfunction

function signed [IW-1:0] cmul_im;
    input signed [IW-1:0] ar, ai, br, bi;
    reg signed [63:0] prod;
    begin
        prod    = (ar * bi) + (ai * br);
        cmul_im = prod >>> 30;
    end
endfunction

// ============================================================================
// COMPLEX DIVISION: a / b = a * conj(b) / |b|^2
//   result_re = (ar*br + ai*bi) / (br^2 + bi^2)
//   result_im = (ai*br - ar*bi) / (br^2 + bi^2)
// ============================================================================
function signed [IW-1:0] cdiv_re;
    input signed [IW-1:0] ar, ai, br, bi, mag2;
    reg signed [63:0] num;
    begin
        num     = (ar * br) + (ai * bi);
        // mag2 is |b|^2 in Q2.30, num is Q4.60, result should be Q2.30
        // result = num / mag2 — behavioral division
        if (mag2 != 0)
            cdiv_re = (num <<< 30) / mag2;
        else
            cdiv_re = 0;
    end
endfunction

function signed [IW-1:0] cdiv_im;
    input signed [IW-1:0] ar, ai, br, bi, mag2;
    reg signed [63:0] num;
    begin
        num     = (ai * br) - (ar * bi);
        if (mag2 != 0)
            cdiv_im = (num <<< 30) / mag2;
        else
            cdiv_im = 0;
    end
endfunction

// ============================================================================
// MAIN FSM
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state    <= ST_IDLE;
        rd_en    <= 0;
        w_valid  <= 0;
        read_cnt <= 0;
        pivot_col<= 0;
        elim_row <= 0;
        col_cnt  <= 0;
        for (i=0; i<4; i=i+1) begin
            for (j=0; j<5; j=j+1) begin
                aug_re[i][j] <= 0;
                aug_im[i][j] <= 0;
            end
        end
    end else begin
        w_valid <= 0;
        rd_en   <= 0;

        case (state)

        // ------------------------------------------------------------------
        // IDLE: wait for compute pulse
        // ------------------------------------------------------------------
        ST_IDLE: begin
            if (compute) begin
                cur_bin  <= bin_in;
                read_cnt <= 0;
                state    <= ST_READ_R;
            end
        end

        // ------------------------------------------------------------------
        // READ_R: read 16 covariance elements one by one
        // Element index = row*4 + col
        // ------------------------------------------------------------------
        ST_READ_R: begin
            // Issue read for current element
            rd_bin   <= cur_bin;
            rd_elem  <= read_cnt;
            rd_en    <= 1;
            elem_latch <= read_cnt;

            // One cycle later rd_valid fires — but since registered read
            // we capture on next cycle
            if (rd_valid) begin
                // Store into augmented matrix
                aug_re[elem_latch[3:2]][elem_latch[1:0]] <= {{16{rd_re[DW-1]}}, rd_re} <<< 15;
                aug_im[elem_latch[3:2]][elem_latch[1:0]] <= {{16{rd_im[DW-1]}}, rd_im} <<< 15;

                if (read_cnt == 15) begin
                    state <= ST_SETUP;
                end else begin
                    read_cnt <= read_cnt + 1;
                end
            end
        end

        // ------------------------------------------------------------------
        // SETUP: add diagonal loading + set augmented column d=[1,1,1,1]
        // ------------------------------------------------------------------
        ST_SETUP: begin
            // Diagonal loading: R[i][i] += delta
            aug_re[0][0] <= aug_re[0][0] + DELTA;
            aug_re[1][1] <= aug_re[1][1] + DELTA;
            aug_re[2][2] <= aug_re[2][2] + DELTA;
            aug_re[3][3] <= aug_re[3][3] + DELTA;

            // Set d column (col 4) = [1,1,1,1] in Q2.30
            // 1.0 in Q2.30 = 2^30 = 1073741824
            aug_re[0][4] <= 32'sd1073741824;
            aug_re[1][4] <= 32'sd1073741824;
            aug_re[2][4] <= 32'sd1073741824;
            aug_re[3][4] <= 32'sd1073741824;
            aug_im[0][4] <= 0;
            aug_im[1][4] <= 0;
            aug_im[2][4] <= 0;
            aug_im[3][4] <= 0;

            pivot_col <= 0;
            state     <= ST_PIVOT;
        end

        // ------------------------------------------------------------------
        // PIVOT: compute 1/pivot, divide pivot row by pivot
        // pivot = aug[pivot_col][pivot_col] (diagonal, always real for R)
        // ------------------------------------------------------------------
        ST_PIVOT: begin
            pivot_re  <= aug_re[pivot_col][pivot_col];
            pivot_im  <= aug_im[pivot_col][pivot_col];
            pivot_mag2 <= cmul_re(aug_re[pivot_col][pivot_col],
                                aug_im[pivot_col][pivot_col],
                                aug_re[pivot_col][pivot_col],
                                -aug_im[pivot_col][pivot_col])
                        + cmul_im(aug_re[pivot_col][pivot_col],
                                aug_im[pivot_col][pivot_col],
                                aug_re[pivot_col][pivot_col],
                                -aug_im[pivot_col][pivot_col]);
            // Divide entire pivot row by pivot (all 5 columns)
            begin : div_pivot_row
                integer dc;
                for (dc=0; dc<5; dc=dc+1) begin
                    aug_re[pivot_col][dc] <=
                        cdiv_re(aug_re[pivot_col][dc],
                                aug_im[pivot_col][dc],
                                aug_re[pivot_col][pivot_col],
                                aug_im[pivot_col][pivot_col],
                                // mag2 of pivot
                                (aug_re[pivot_col][pivot_col] * aug_re[pivot_col][pivot_col] +
                                 aug_im[pivot_col][pivot_col] * aug_im[pivot_col][pivot_col]) >>> 30);
                    aug_im[pivot_col][dc] <=
                        cdiv_im(aug_re[pivot_col][dc],
                                aug_im[pivot_col][dc],
                                aug_re[pivot_col][pivot_col],
                                aug_im[pivot_col][pivot_col],
                                (aug_re[pivot_col][pivot_col] * aug_re[pivot_col][pivot_col] +
                                 aug_im[pivot_col][pivot_col] * aug_im[pivot_col][pivot_col]) >>> 30);
                end
            end
            elim_row <= (pivot_col == 0) ? 1 : 0;  // start elim from row 0, skip pivot row
            col_cnt  <= 0;
            state    <= ST_ELIM;
        end

        // ------------------------------------------------------------------
        // ELIM: eliminate pivot_col from row elim_row
        // factor = aug[elim_row][pivot_col]
        // row[elim_row] -= factor * row[pivot_col]  (for all 5 cols)
        // ------------------------------------------------------------------
        ST_ELIM: begin
            if (elim_row != pivot_col) begin
                // Compute factor
                factor_re <= aug_re[elim_row][pivot_col];
                factor_im <= aug_im[elim_row][pivot_col];

                // Eliminate all 5 columns
                begin : elim_cols
                    integer ec;
                    for (ec=0; ec<5; ec=ec+1) begin
                        aug_re[elim_row][ec] <= aug_re[elim_row][ec]
                            - cmul_re(aug_re[elim_row][pivot_col],
                                    aug_im[elim_row][pivot_col],
                                    aug_re[pivot_col][ec],
                                    aug_im[pivot_col][ec]);
                        aug_im[elim_row][ec] <= aug_im[elim_row][ec]
                            - cmul_im(aug_re[elim_row][pivot_col],
                                    aug_im[elim_row][pivot_col],
                                    aug_re[pivot_col][ec],
                                    aug_im[pivot_col][ec]);
                    end
                end
            end

            // Advance to next row
            if (elim_row == 3) begin
                // Done eliminating this column
                if (pivot_col == 3) begin
                    // All pivots done — go to normalization
                    state <= ST_NORM;
                end else begin
                    pivot_col <= pivot_col + 1;
                    state     <= ST_PIVOT;
                end
            end else begin
                elim_row <= elim_row + 1;
                // Skip pivot row
                if (elim_row + 1 == pivot_col)
                    elim_row <= elim_row + 2;
            end
        end

        // ------------------------------------------------------------------
        // NORM: compute denominator = d^H * u
        // d = [1,1,1,1] so d^H*u = u[0]+u[1]+u[2]+u[3] (complex sum)
        // unnormalized weights are in aug[0..3][4]
        // w[i] = aug[i][4] / denom
        // ------------------------------------------------------------------
        ST_NORM: begin
            denom_re <= aug_re[0][4] + aug_re[1][4] + aug_re[2][4] + aug_re[3][4];
            state    <= ST_OUTPUT;
        end

        // ------------------------------------------------------------------
        // OUTPUT: normalize and write to output ports
        // ------------------------------------------------------------------
        ST_OUTPUT: begin
            begin : normalize
                reg signed [IW-1:0] d_mag2;
                // denom is real (d is real-valued), so mag2 = denom_re^2
                // Actually denom may have small imag part due to rounding
                // Use real part only as approximation
                d_mag2 = (denom_re >>> 15) * (denom_re >>> 15);

                // w[i] = aug[i][4] / denom_re  (simplified: denom treated as real)
                // Truncate from Q2.30 to Q1.15 (shift right 15)
                w0_re <= cdiv_re(aug_re[0][4], aug_im[0][4],
                                denom_re, 0, (denom_re>>>15)*(denom_re>>>15)) >>> 15;
                w0_im <= cdiv_im(aug_re[0][4], aug_im[0][4],
                                denom_re, 0, (denom_re>>>15)*(denom_re>>>15)) >>> 15;
                w1_re <= cdiv_re(aug_re[1][4], aug_im[1][4],
                                denom_re, 0, (denom_re>>>15)*(denom_re>>>15)) >>> 15;
                w1_im <= cdiv_im(aug_re[1][4], aug_im[1][4],
                                denom_re, 0, (denom_re>>>15)*(denom_re>>>15)) >>> 15;
                w2_re <= cdiv_re(aug_re[2][4], aug_im[2][4],
                                denom_re, 0, (denom_re>>>15)*(denom_re>>>15)) >>> 15;
                w2_im <= cdiv_im(aug_re[2][4], aug_im[2][4],
                                denom_re, 0, (denom_re>>>15)*(denom_re>>>15)) >>> 15;
                w3_re <= cdiv_re(aug_re[3][4], aug_im[3][4],
                                denom_re, 0, (denom_re>>>15)*(denom_re>>>15)) >>> 15;
                w3_im <= cdiv_im(aug_re[3][4], aug_im[3][4],
                                denom_re, 0, (denom_re>>>15)*(denom_re>>>15)) >>> 15;
            end

            w_bin   <= cur_bin;
            w_valid <= 1;
            state   <= ST_IDLE;
        end

        endcase
    end
end

endmodule