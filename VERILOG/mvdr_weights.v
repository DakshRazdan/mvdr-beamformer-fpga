// ============================================================================
// mvdr_weights.v  —  MVDR Weight Computation (per frequency bin)  v6
// -
// ROOT CAUSE OF ALL PREVIOUS FAILURES:
//   iverilog silently returns 0 when reading a 2D reg array with a variable
//   first index — aug_re[pivot_col][dc] and aug_re[elim_row][ec] all read 0.
//   Same class of bug as covariance_est.v (that used explicit 4:1 mux fix).
//
// FIX: completely remove 2D arrays and all loops.
//   - 40 flat named registers (a00r..a34i) — zero variable indexing
//   - Explicit state per pivot and per elimination row
//   - ST_LOAD_ISSUE/WAIT pair with case statement for storing — no elem_latch
//   - 64-bit arithmetic throughout for divisions (pivot^2 overflows int32)
// ============================================================================

`timescale 1ns/1ps

module mvdr_weights #(
    parameter NBINS  = 129,
    parameter NMICS  = 4,
    parameter DW = 16,
    parameter IW = 32,
    parameter signed [31:0] DELTA = 32'sd536870912   // 0.5 * 2^30
)(
    input wire clk,
    input wire rst_n,
    input wire compute,
    input wire [7:0] bin_in,
    output reg [7:0] rd_bin,
    output reg [3:0] rd_elem,
    output reg rd_en,
    input wire signed [DW-1:0] rd_re,
    input wire signed [DW-1:0] rd_im,
    input wire rd_valid,
    output reg signed [DW-1:0] w0_re, w0_im,
    output reg signed [DW-1:0] w1_re, w1_im,
    output reg signed [DW-1:0] w2_re, w2_im,
    output reg signed [DW-1:0] w3_re, w3_im,
    output reg [7:0] w_bin,
    output reg w_valid
);

// ============================================================================
// FLAT MATRIX REGISTERS — no 2D arrays, zero variable indexing
// a{row}{col}r / a{row}{col}i   row=0..3, col=0..4  (col4 = d column)
// All Q2.30 (32-bit signed)
// ============================================================================
reg signed [IW-1:0] a00r,a00i, a01r,a01i, a02r,a02i, a03r,a03i, a04r,a04i;
reg signed [IW-1:0] a10r,a10i, a11r,a11i, a12r,a12i, a13r,a13i, a14r,a14i;
reg signed [IW-1:0] a20r,a20i, a21r,a21i, a22r,a22i, a23r,a23i, a24r,a24i;
reg signed [IW-1:0] a30r,a30i, a31r,a31i, a32r,a32i, a33r,a33i, a34r,a34i;

reg signed [63:0] denom_re;   // needs >32 bits: sum of 4 Q2.30 values can reach 4.0*2^30 > 2^31

// ============================================================================
// STATES — one per pivot, one per elimination row
// ============================================================================
localparam [4:0]
    ST_IDLE = 5'd0,
    ST_LOAD_ISSUE = 5'd1,   // assert rd_en for current read_cnt
    ST_LOAD_WAIT = 5'd2,   // wait for rd_valid, store to named reg
    ST_SETUP = 5'd3,   // add diagonal loading, set d column
    ST_P0 = 5'd4,   // pivot col 0: divide row 0 by a00
    ST_E0_1 = 5'd5,   // elim col 0 from row 1
    ST_E0_2 = 5'd6,   // elim col 0 from row 2
    ST_E0_3 = 5'd7,   // elim col 0 from row 3
    ST_P1 = 5'd8,   // pivot col 1: divide row 1 by a11
    ST_E1_0 = 5'd9,   // elim col 1 from row 0
    ST_E1_2 = 5'd10,  // elim col 1 from row 2
    ST_E1_3 = 5'd11,  // elim col 1 from row 3
    ST_P2 = 5'd12,  // pivot col 2: divide row 2 by a22
    ST_E2_0 = 5'd13,  // elim col 2 from row 0
    ST_E2_1 = 5'd14,  // elim col 2 from row 1
    ST_E2_3 = 5'd15,  // elim col 2 from row 3
    ST_P3 = 5'd16,  // pivot col 3: divide row 3 by a33
    ST_E3_0 = 5'd17,  // elim col 3 from row 0
    ST_E3_1 = 5'd18,  // elim col 3 from row 1
    ST_E3_2 = 5'd19,  // elim col 3 from row 2
    ST_NORM = 5'd20,
    ST_OUTPUT = 5'd21;

reg [4:0] state;
reg [7:0] cur_bin;
reg [3:0] read_cnt;

// ============================================================================
// COMPLEX MULTIPLY  Q2.30 × Q2.30 → Q2.30
// Uses 64-bit intermediate — no overflow for values ≤ 2^31
// ============================================================================
function signed [IW-1:0] cmr;   // cmul real part
    input signed [IW-1:0] ar, ai, br, bi;
    reg signed [63:0] p;
    begin p = (ar*br)-(ai*bi); cmr = p>>>30; end
endfunction

function signed [IW-1:0] cmi;   // cmul imag part
    input signed [IW-1:0] ar, ai, br, bi;
    reg signed [63:0] p;
    begin p = (ar*bi)+(ai*br); cmi = p>>>30; end
endfunction

// ============================================================================
// SIGN-EXTEND Q1.15 → Q2.30
// ============================================================================
`define EXT15(x) ($signed({{16{x[DW-1]}}, x}) <<< 15)

// ============================================================================
// 64-bit pivot divide: (val * 2^30) / pivot   result = Q2.30
// pivot is always real positive (Hermitian + diagonal loading)
// overflow check: |val| ≤ 2^31, 2^30 < 2^32 → product ≤ 2^63 ✓
// ============================================================================
`define PDIV_R(val, pvt) \
    (($signed({{32{val[IW-1]}},val}) * 64'sd1073741824) / $signed({{32{pvt[IW-1]}},pvt}))

// ============================================================================
// OUTPUT NORMALIZE: (aug_col4 * 2^15) / denom   result = Q1.15
// ============================================================================
`define WDIV(val) \
    (($signed({{32{val[IW-1]}},val}) * 64'sd32768) / denom_re)

// ============================================================================
// MAIN FSM
// ============================================================================
integer k;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ST_IDLE;
        rd_en <= 0;
        w_valid <= 0;
        read_cnt<= 0;
        a00r<=0;a00i<=0; a01r<=0;a01i<=0; a02r<=0;a02i<=0; a03r<=0;a03i<=0; a04r<=0;a04i<=0;
        a10r<=0;a10i<=0; a11r<=0;a11i<=0; a12r<=0;a12i<=0; a13r<=0;a13i<=0; a14r<=0;a14i<=0;
        a20r<=0;a20i<=0; a21r<=0;a21i<=0; a22r<=0;a22i<=0; a23r<=0;a23i<=0; a24r<=0;a24i<=0;
        a30r<=0;a30i<=0; a31r<=0;a31i<=0; a32r<=0;a32i<=0; a33r<=0;a33i<=0; a34r<=0;a34i<=0;
    end else begin
        w_valid <= 0;
        rd_en <= 0;

        case (state)

        // ------------------------------------------------------------------
        ST_IDLE: if (compute) begin
            cur_bin <= bin_in;
            read_cnt <= 0;
            state <= ST_LOAD_ISSUE;
        end

        // ------------------------------------------------------------------
        // Two-phase RAM read: ISSUE then WAIT per element.
        // No elem_latch — read_cnt is stable during WAIT.
        // ------------------------------------------------------------------
        ST_LOAD_ISSUE: begin
            rd_bin <= cur_bin;
            rd_elem <= read_cnt;
            rd_en <= 1;
            state <= ST_LOAD_WAIT;
        end

        ST_LOAD_WAIT: begin
            if (rd_valid) begin
                // Store to explicitly named register — no variable 2D indexing
                case (read_cnt)
                    4'd0: begin a00r<=`EXT15(rd_re); a00i<=`EXT15(rd_im); end
                    4'd1: begin a01r<=`EXT15(rd_re); a01i<=`EXT15(rd_im); end
                    4'd2: begin a02r<=`EXT15(rd_re); a02i<=`EXT15(rd_im); end
                    4'd3: begin a03r<=`EXT15(rd_re); a03i<=`EXT15(rd_im); end
                    4'd4: begin a10r<=`EXT15(rd_re); a10i<=`EXT15(rd_im); end
                    4'd5: begin a11r<=`EXT15(rd_re); a11i<=`EXT15(rd_im); end
                    4'd6: begin a12r<=`EXT15(rd_re); a12i<=`EXT15(rd_im); end
                    4'd7: begin a13r<=`EXT15(rd_re); a13i<=`EXT15(rd_im); end
                    4'd8: begin a20r<=`EXT15(rd_re); a20i<=`EXT15(rd_im); end
                    4'd9: begin a21r<=`EXT15(rd_re); a21i<=`EXT15(rd_im); end
                    4'd10: begin a22r<=`EXT15(rd_re); a22i<=`EXT15(rd_im); end
                    4'd11: begin a23r<=`EXT15(rd_re); a23i<=`EXT15(rd_im); end
                    4'd12: begin a30r<=`EXT15(rd_re); a30i<=`EXT15(rd_im); end
                    4'd13: begin a31r<=`EXT15(rd_re); a31i<=`EXT15(rd_im); end
                    4'd14: begin a32r<=`EXT15(rd_re); a32i<=`EXT15(rd_im); end
                    4'd15: begin a33r<=`EXT15(rd_re); a33i<=`EXT15(rd_im); end
                    default: ;
                endcase
                if (read_cnt == 4'd15)
                    state <= ST_SETUP;
                else begin
                    read_cnt <= read_cnt + 1;
                    state <= ST_LOAD_ISSUE;
                end
            end
        end

        // ------------------------------------------------------------------
        // SETUP: add diagonal loading δI, set d=[1,1,1,1] (Q2.30 = 2^30)
        // ------------------------------------------------------------------
        ST_SETUP: begin
            a00r <= a00r + DELTA;
            a11r <= a11r + DELTA;
            a22r <= a22r + DELTA;
            a33r <= a33r + DELTA;
            a04r <= 32'sd1073741824; a04i <= 0;
            a14r <= 32'sd1073741824; a14i <= 0;
            a24r <= 32'sd1073741824; a24i <= 0;
            a34r <= 32'sd1073741824; a34i <= 0;
            state <= ST_P0;
        end

        // ------------------------------------------------------------------
        // PIVOT 0: divide row 0 by a00r (real, positive)
        // Result: a00r = 1.0 (Q2.30), others scaled
        // ------------------------------------------------------------------
        ST_P0: begin
            a00r <= 32'sd1073741824; a00i <= 0;   // 1.0 exactly
            a01r <= `PDIV_R(a01r, a00r); a01i <= `PDIV_R(a01i, a00r);
            a02r <= `PDIV_R(a02r, a00r); a02i <= `PDIV_R(a02i, a00r);
            a03r <= `PDIV_R(a03r, a00r); a03i <= `PDIV_R(a03i, a00r);
            a04r <= `PDIV_R(a04r, a00r); a04i <= `PDIV_R(a04i, a00r);
            state <= ST_E0_1;
        end

        // ------------------------------------------------------------------
        // ELIM col 0 from rows 1,2,3:  row_i -= a_i0 * row_0
        // a_i0 is zeroed (it equals 0 after elimination by definition).
        // The factor a_i0 is read BEFORE the NB write zeroes it.
        // ------------------------------------------------------------------
        ST_E0_1: begin
            begin : e01
                reg signed [IW-1:0] fr,fi;
                fr=a10r; fi=a10i;
                a10r<=0; a10i<=0;
                a11r<=a11r-cmr(fr,fi,a01r,a01i); a11i<=a11i-cmi(fr,fi,a01r,a01i);
                a12r<=a12r-cmr(fr,fi,a02r,a02i); a12i<=a12i-cmi(fr,fi,a02r,a02i);
                a13r<=a13r-cmr(fr,fi,a03r,a03i); a13i<=a13i-cmi(fr,fi,a03r,a03i);
                a14r<=a14r-cmr(fr,fi,a04r,a04i); a14i<=a14i-cmi(fr,fi,a04r,a04i);
            end
            state <= ST_E0_2;
        end

        ST_E0_2: begin
            begin : e02
                reg signed [IW-1:0] fr,fi;
                fr=a20r; fi=a20i;
                a20r<=0; a20i<=0;
                a21r<=a21r-cmr(fr,fi,a01r,a01i); a21i<=a21i-cmi(fr,fi,a01r,a01i);
                a22r<=a22r-cmr(fr,fi,a02r,a02i); a22i<=a22i-cmi(fr,fi,a02r,a02i);
                a23r<=a23r-cmr(fr,fi,a03r,a03i); a23i<=a23i-cmi(fr,fi,a03r,a03i);
                a24r<=a24r-cmr(fr,fi,a04r,a04i); a24i<=a24i-cmi(fr,fi,a04r,a04i);
            end
            state <= ST_E0_3;
        end

        ST_E0_3: begin
            begin : e03
                reg signed [IW-1:0] fr,fi;
                fr=a30r; fi=a30i;
                a30r<=0; a30i<=0;
                a31r<=a31r-cmr(fr,fi,a01r,a01i); a31i<=a31i-cmi(fr,fi,a01r,a01i);
                a32r<=a32r-cmr(fr,fi,a02r,a02i); a32i<=a32i-cmi(fr,fi,a02r,a02i);
                a33r<=a33r-cmr(fr,fi,a03r,a03i); a33i<=a33i-cmi(fr,fi,a03r,a03i);
                a34r<=a34r-cmr(fr,fi,a04r,a04i); a34i<=a34i-cmi(fr,fi,a04r,a04i);
            end
            state <= ST_P1;
        end

        // ------------------------------------------------------------------
        // PIVOT 1: divide row 1 by a11r
        // ------------------------------------------------------------------
        ST_P1: begin
            a10r <= 0; a10i <= 0;   // already 0, make explicit
            a11r <= 32'sd1073741824; a11i <= 0;
            a12r <= `PDIV_R(a12r, a11r); a12i <= `PDIV_R(a12i, a11r);
            a13r <= `PDIV_R(a13r, a11r); a13i <= `PDIV_R(a13i, a11r);
            a14r <= `PDIV_R(a14r, a11r); a14i <= `PDIV_R(a14i, a11r);
            state <= ST_E1_0;
        end

        ST_E1_0: begin
            begin : e10
                reg signed [IW-1:0] fr,fi;
                fr=a01r; fi=a01i;
                a01r<=0; a01i<=0;
                a02r<=a02r-cmr(fr,fi,a12r,a12i); a02i<=a02i-cmi(fr,fi,a12r,a12i);
                a03r<=a03r-cmr(fr,fi,a13r,a13i); a03i<=a03i-cmi(fr,fi,a13r,a13i);
                a04r<=a04r-cmr(fr,fi,a14r,a14i); a04i<=a04i-cmi(fr,fi,a14r,a14i);
            end
            state <= ST_E1_2;
        end

        ST_E1_2: begin
            begin : e12
                reg signed [IW-1:0] fr,fi;
                fr=a21r; fi=a21i;
                a21r<=0; a21i<=0;
                a22r<=a22r-cmr(fr,fi,a12r,a12i); a22i<=a22i-cmi(fr,fi,a12r,a12i);
                a23r<=a23r-cmr(fr,fi,a13r,a13i); a23i<=a23i-cmi(fr,fi,a13r,a13i);
                a24r<=a24r-cmr(fr,fi,a14r,a14i); a24i<=a24i-cmi(fr,fi,a14r,a14i);
            end
            state <= ST_E1_3;
        end

        ST_E1_3: begin
            begin : e13
                reg signed [IW-1:0] fr,fi;
                fr=a31r; fi=a31i;
                a31r<=0; a31i<=0;
                a32r<=a32r-cmr(fr,fi,a12r,a12i); a32i<=a32i-cmi(fr,fi,a12r,a12i);
                a33r<=a33r-cmr(fr,fi,a13r,a13i); a33i<=a33i-cmi(fr,fi,a13r,a13i);
                a34r<=a34r-cmr(fr,fi,a14r,a14i); a34i<=a34i-cmi(fr,fi,a14r,a14i);
            end
            state <= ST_P2;
        end

        // ------------------------------------------------------------------
        // PIVOT 2: divide row 2 by a22r
        // ------------------------------------------------------------------
        ST_P2: begin
            a20r <= 0; a20i <= 0;
            a21r <= 0; a21i <= 0;
            a22r <= 32'sd1073741824; a22i <= 0;
            a23r <= `PDIV_R(a23r, a22r); a23i <= `PDIV_R(a23i, a22r);
            a24r <= `PDIV_R(a24r, a22r); a24i <= `PDIV_R(a24i, a22r);
            state <= ST_E2_0;
        end

        ST_E2_0: begin
            begin : e20
                reg signed [IW-1:0] fr,fi;
                fr=a02r; fi=a02i;
                a02r<=0; a02i<=0;
                a03r<=a03r-cmr(fr,fi,a23r,a23i); a03i<=a03i-cmi(fr,fi,a23r,a23i);
                a04r<=a04r-cmr(fr,fi,a24r,a24i); a04i<=a04i-cmi(fr,fi,a24r,a24i);
            end
            state <= ST_E2_1;
        end

        ST_E2_1: begin
            begin : e21
                reg signed [IW-1:0] fr,fi;
                fr=a12r; fi=a12i;
                a12r<=0; a12i<=0;
                a13r<=a13r-cmr(fr,fi,a23r,a23i); a13i<=a13i-cmi(fr,fi,a23r,a23i);
                a14r<=a14r-cmr(fr,fi,a24r,a24i); a14i<=a14i-cmi(fr,fi,a24r,a24i);
            end
            state <= ST_E2_3;
        end

        ST_E2_3: begin
            begin : e23
                reg signed [IW-1:0] fr,fi;
                fr=a32r; fi=a32i;
                a32r<=0; a32i<=0;
                a33r<=a33r-cmr(fr,fi,a23r,a23i); a33i<=a33i-cmi(fr,fi,a23r,a23i);
                a34r<=a34r-cmr(fr,fi,a24r,a24i); a34i<=a34i-cmi(fr,fi,a24r,a24i);
            end
            state <= ST_P3;
        end

        // ------------------------------------------------------------------
        // PIVOT 3: divide row 3 by a33r
        // ------------------------------------------------------------------
        ST_P3: begin
            a30r <= 0; a30i <= 0;
            a31r <= 0; a31i <= 0;
            a32r <= 0; a32i <= 0;
            a33r <= 32'sd1073741824; a33i <= 0;
            a34r <= `PDIV_R(a34r, a33r); a34i <= `PDIV_R(a34i, a33r);
            state <= ST_E3_0;
        end

        ST_E3_0: begin
            begin : e30
                reg signed [IW-1:0] fr,fi;
                fr=a03r; fi=a03i;
                a03r<=0; a03i<=0;
                a04r<=a04r-cmr(fr,fi,a34r,a34i); a04i<=a04i-cmi(fr,fi,a34r,a34i);
            end
            state <= ST_E3_1;
            
        end

        ST_E3_1: begin
            begin : e31
                reg signed [IW-1:0] fr,fi;
                fr=a13r; fi=a13i;
                a13r<=0; a13i<=0;
                a14r<=a14r-cmr(fr,fi,a34r,a34i); a14i<=a14i-cmi(fr,fi,a34r,a34i);
            end
            state <= ST_E3_2;
        end

        ST_E3_2: begin
            begin : e32
                reg signed [IW-1:0] fr,fi;
                fr=a23r; fi=a23i;
                a23r<=0; a23i<=0;
                a24r<=a24r-cmr(fr,fi,a34r,a34i); a24i<=a24i-cmi(fr,fi,a34r,a34i);
            end
            state <= ST_NORM;
        end

        // ------------------------------------------------------------------
        // NORM: denom = sum of d column (d=[1,1,1,1] so sum = d^H * u)
        // ------------------------------------------------------------------
        ST_NORM: begin
            denom_re <= $signed({{32{a04r[31]}},a04r}) + $signed({{32{a14r[31]}},a14r}) + $signed({{32{a24r[31]}},a24r}) + $signed({{32{a34r[31]}},a34r});
            state <= ST_OUTPUT;
        end

        // ------------------------------------------------------------------
        // OUTPUT: w_Q1.15 = (a_i4_Q2.30 * 2^15) / denom_Q2.30
        // 64-bit: max a_i4 ~ 2^30, * 2^15 = 2^45 << 2^63 ✓
        // ------------------------------------------------------------------
        ST_OUTPUT: begin
            if (denom_re != 0) begin
                w0_re <= `WDIV(a04r); w0_im <= `WDIV(a04i);
                w1_re <= `WDIV(a14r); w1_im <= `WDIV(a14i);
                w2_re <= `WDIV(a24r); w2_im <= `WDIV(a24i);
                w3_re <= `WDIV(a34r); w3_im <= `WDIV(a34i);
            end else begin
                w0_re<=0;w0_im<=0; w1_re<=0;w1_im<=0;
                w2_re<=0;w2_im<=0; w3_re<=0;w3_im<=0;
            end
            w_bin   <= cur_bin;
            w_valid <= 1;
            state   <= ST_IDLE;
        end

        endcase
    end
end

endmodule