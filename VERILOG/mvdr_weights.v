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
//   d = [1+0j, 1+0j, 1+0j, 1+0j]
//
// DIAGONAL LOADING: R_loaded = R + delta*I  (delta = 0.5 in Q1.15 = 16384)
//
// FIX: elim_row was 2-bit, causing wrap-around to 0 when pivot_col=3
//      (skipping row 3 → next = 4 → 2-bit wraps to 0 → infinite loop)
//      Fixed by using 3-bit arithmetic in advancement logic.
// ============================================================================

module mvdr_weights #(
    parameter NBINS  = 129,
    parameter NMICS  = 4,
    parameter DW     = 16,
    parameter IW     = 32,
    parameter signed [31:0] DELTA = 32'sd536870912
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         compute,
    input  wire [7:0]   bin_in,

    output reg  [7:0]   rd_bin,
    output reg  [3:0]   rd_elem,
    output reg          rd_en,
    input  wire signed [DW-1:0] rd_re,
    input  wire signed [DW-1:0] rd_im,
    input  wire         rd_valid,

    output reg  signed [DW-1:0] w0_re, w0_im,
    output reg  signed [DW-1:0] w1_re, w1_im,
    output reg  signed [DW-1:0] w2_re, w2_im,
    output reg  signed [DW-1:0] w3_re, w3_im,
    output reg  [7:0]   w_bin,
    output reg          w_valid
);

reg signed [IW-1:0] aug_re [0:3][0:4];
reg signed [IW-1:0] aug_im [0:3][0:4];

localparam ST_IDLE   = 4'd0;
localparam ST_READ_R = 4'd1;
localparam ST_SETUP  = 4'd2;
localparam ST_PIVOT  = 4'd3;
localparam ST_ELIM   = 4'd4;
localparam ST_NORM   = 4'd5;
localparam ST_OUTPUT = 4'd6;

reg [3:0]  state;
reg [7:0]  cur_bin;
reg [3:0]  read_cnt;
reg [3:0]  elem_latch;
reg [1:0]  pivot_col;
reg [1:0]  elim_row;      // 2-bit is fine for storage (rows 0-3)
reg [2:0]  col_cnt;

reg signed [IW-1:0] pivot_re, pivot_im;
reg signed [IW-1:0] pivot_mag2;
reg signed [IW-1:0] factor_re, factor_im;
reg signed [IW-1:0] denom_re;

integer i, j;

// ============================================================================
// COMPLEX MULTIPLY  (Q2.30 × Q2.30 → Q2.30)
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
// COMPLEX DIVISION  a / b = a * conj(b) / |b|^2
// ============================================================================
function signed [IW-1:0] cdiv_re;
    input signed [IW-1:0] ar, ai, br, bi, mag2;
    reg signed [63:0] num;
    begin
        num = (ar * br) + (ai * bi);
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
        num = (ai * br) - (ar * bi);
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
        state     <= ST_IDLE;
        rd_en     <= 0;
        w_valid   <= 0;
        read_cnt  <= 0;
        pivot_col <= 0;
        elim_row  <= 0;
        col_cnt   <= 0;
        for (i=0; i<4; i=i+1)
            for (j=0; j<5; j=j+1) begin
                aug_re[i][j] <= 0;
                aug_im[i][j] <= 0;
            end
    end else begin
        w_valid <= 0;
        rd_en   <= 0;

        case (state)

        // ------------------------------------------------------------------
        ST_IDLE: begin
            if (compute) begin
                cur_bin  <= bin_in;
                read_cnt <= 0;
                state    <= ST_READ_R;
            end
        end

        // ------------------------------------------------------------------
        ST_READ_R: begin
            rd_bin     <= cur_bin;
            rd_elem    <= read_cnt;
            rd_en      <= 1;
            elem_latch <= read_cnt;

            if (rd_valid) begin
                aug_re[elem_latch[3:2]][elem_latch[1:0]] <= {{16{rd_re[DW-1]}}, rd_re} <<< 15;
                aug_im[elem_latch[3:2]][elem_latch[1:0]] <= {{16{rd_im[DW-1]}}, rd_im} <<< 15;

                if (read_cnt == 15)
                    state <= ST_SETUP;
                else
                    read_cnt <= read_cnt + 1;
            end
        end

        // ------------------------------------------------------------------
        ST_SETUP: begin
            aug_re[0][0] <= aug_re[0][0] + DELTA;
            aug_re[1][1] <= aug_re[1][1] + DELTA;
            aug_re[2][2] <= aug_re[2][2] + DELTA;
            aug_re[3][3] <= aug_re[3][3] + DELTA;

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
        ST_PIVOT: begin
            begin : div_pivot_row
                integer dc;
                reg signed [63:0] pmag;
                pmag = (aug_re[pivot_col][pivot_col] * aug_re[pivot_col][pivot_col]) +
                       (aug_im[pivot_col][pivot_col] * aug_im[pivot_col][pivot_col]);
                for (dc=0; dc<5; dc=dc+1) begin
                    aug_re[pivot_col][dc] <=
                        cdiv_re(aug_re[pivot_col][dc],
                                aug_im[pivot_col][dc],
                                aug_re[pivot_col][pivot_col],
                                aug_im[pivot_col][pivot_col],
                                pmag >>> 30);
                    aug_im[pivot_col][dc] <=
                        cdiv_im(aug_re[pivot_col][dc],
                                aug_im[pivot_col][dc],
                                aug_re[pivot_col][pivot_col],
                                aug_im[pivot_col][pivot_col],
                                pmag >>> 30);
                end
            end
            // Start elimination from row 0 (skip pivot_col in advancement logic)
            elim_row <= 0;
            state    <= ST_ELIM;
        end

        // ------------------------------------------------------------------
        // ELIM: one row per cycle, skip pivot row
        // ------------------------------------------------------------------
        ST_ELIM: begin
            if (elim_row != pivot_col) begin
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

            // ---- FIX: use 3-bit arithmetic to detect over-run past row 3 ----
            begin : advance_elim
                reg [2:0] nxt;
                nxt = {1'b0, elim_row} + 3'd1;             // candidate next row
                if (nxt[1:0] == pivot_col) nxt = nxt + 3'd1; // skip pivot row

                if (nxt > 3'd3) begin
                    // No more rows — pivot column done
                    if (pivot_col == 2'd3)
                        state <= ST_NORM;
                    else begin
                        pivot_col <= pivot_col + 1;
                        state     <= ST_PIVOT;
                    end
                end else begin
                    elim_row <= nxt[1:0];
                end
            end
        end

        // ------------------------------------------------------------------
        ST_NORM: begin
            denom_re <= aug_re[0][4] + aug_re[1][4] + aug_re[2][4] + aug_re[3][4];
            state    <= ST_OUTPUT;
        end

        // ------------------------------------------------------------------
        ST_OUTPUT: begin
            begin : normalize
                reg signed [63:0] dmag2;
                dmag2 = (denom_re >>> 15) * (denom_re >>> 15);

                w0_re <= cdiv_re(aug_re[0][4], aug_im[0][4],
                                denom_re, 0, dmag2[31:0]) >>> 15;
                w0_im <= cdiv_im(aug_re[0][4], aug_im[0][4],
                                denom_re, 0, dmag2[31:0]) >>> 15;
                w1_re <= cdiv_re(aug_re[1][4], aug_im[1][4],
                                denom_re, 0, dmag2[31:0]) >>> 15;
                w1_im <= cdiv_im(aug_re[1][4], aug_im[1][4],
                                denom_re, 0, dmag2[31:0]) >>> 15;
                w2_re <= cdiv_re(aug_re[2][4], aug_im[2][4],
                                denom_re, 0, dmag2[31:0]) >>> 15;
                w2_im <= cdiv_im(aug_re[2][4], aug_im[2][4],
                                denom_re, 0, dmag2[31:0]) >>> 15;
                w3_re <= cdiv_re(aug_re[3][4], aug_im[3][4],
                                denom_re, 0, dmag2[31:0]) >>> 15;
                w3_im <= cdiv_im(aug_re[3][4], aug_im[3][4],
                                denom_re, 0, dmag2[31:0]) >>> 15;
            end
            w_bin   <= cur_bin;
            w_valid <= 1;
            state   <= ST_IDLE;
        end

        endcase
    end
end

endmodule