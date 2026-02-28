// ============================================================================
// covariance_est_tb.v  —  Testbench for Covariance Matrix Estimator
// ----------------------------------------------------------------------------
// Test 1: Single frame, check R[0][0] = |X0|^2 (diagonal element, real-only)
// Test 2: Two frames with forgetting, check alpha decay: R = alpha*R + new
// Test 3: Check R[0][1] = X0*conj(X1) cross-correlation term
// Test 4: Check R[1][0] = conj(R[0][1]) (Hermitian symmetry)
// ============================================================================

`timescale 1ns/1ps

module covariance_est_tb;

localparam DW     = 16;
localparam NBINS  = 129;
localparam NMICS  = 4;
localparam CLK_P  = 10;

// Q1.15 alpha = 0.95 * 32767 = 31129
localparam signed [DW-1:0] ALPHA = 16'd31129;

reg clk, rst_n;


// Inputs
reg signed [DW-1:0] x0_re, x0_im;
reg signed [DW-1:0] x1_re, x1_im;
reg signed [DW-1:0] x2_re, x2_im;
reg signed [DW-1:0] x3_re, x3_im;

reg [$clog2(NBINS)-1:0] x_bin;
reg x_valid;


// Read port
reg [$clog2(NBINS)-1:0] rd_bin;
reg [3:0]               rd_elem;
reg                     rd_en;

wire signed [DW-1:0]    rd_re, rd_im;
wire                    rd_valid;



covariance_est #(
    .NBINS(NBINS), .NMICS(NMICS), .DW(DW), .ALPHA(ALPHA)
) dut (

    .clk(clk),
    .rst_n(rst_n),

    .x0_re(x0_re),
    .x0_im(x0_im),

    .x1_re(x1_re),
    .x1_im(x1_im),

    .x2_re(x2_re),
    .x2_im(x2_im),

    .x3_re(x3_re),
    .x3_im(x3_im),

    .x_bin(x_bin),
    .x_valid(x_valid),

    .rd_bin(rd_bin),
    .rd_elem(rd_elem),
    .rd_en(rd_en),

    .rd_re(rd_re),
    .rd_im(rd_im),
    .rd_valid(rd_valid)
);


initial clk = 0;
always #(CLK_P/2) clk = ~clk;


// Helper: Q1.15 multiply (for expected value computation)

function signed [DW-1:0] q15_mul;
    input signed [DW-1:0] a, b;
    reg signed [2*DW-1:0] prod;
    begin
        prod   = a * b;
        q15_mul = prod >>> 15;
    end
endfunction



// Send one bin update

task send_bin;
    input [$clog2(NBINS)-1:0] bin;
    input signed [DW-1:0] r0, i0, r1, i1, r2, i2, r3, i3;
    begin
        @(negedge clk);

        x0_re=r0; x0_im=i0;
        x1_re=r1; x1_im=i1;
        x2_re=r2; x2_im=i2;
        x3_re=r3; x3_im=i3;

        x_bin   = bin;
        x_valid = 1;

        @(negedge clk);

        x_valid = 0;
    end
endtask



// Read one element, wait for valid

task read_elem;
    input [$clog2(NBINS)-1:0] bin;
    input [3:0] elem;
    begin

        @(negedge clk);

        rd_bin  = bin;
        rd_elem = elem;
        rd_en   = 1;

        @(negedge clk);

        rd_en   = 0;

        @(posedge rd_valid);
        @(posedge clk); // settle

    end
endtask



integer pass_count, fail_count;


// Check with ±2 tolerance (fixed-point rounding)

task check_val;

    input signed [DW-1:0] got;
    input signed [DW-1:0] exp;
    input [127:0] name;

    begin

        if (got >= exp-2 && got <= exp+2) begin

            $display("  PASS %s: got=%0d exp=%0d", name, $signed(got), $signed(exp));
            pass_count = pass_count + 1;

        end else begin

            $display("  FAIL %s: got=%0d exp=%0d", name, $signed(got), $signed(exp));
            fail_count = fail_count + 1;

        end
    end

endtask



reg signed [DW-1:0] exp_re, exp_im;
reg signed [DW-1:0] x0r, x0i, x1r, x1i;



initial begin

    $dumpfile("covariance_est.vcd");
    $dumpvars(0, covariance_est_tb);

    pass_count = 0;
    fail_count = 0;

    rst_n   = 0;
    x_valid = 0;
    rd_en   = 0;

    x0_re=0; x1_re=0; x2_re=0; x3_re=0;
    x0_im=0; x1_im=0; x2_im=0; x3_im=0;

    x_bin = 0;
    rd_bin = 0;
    rd_elem = 0;


    repeat(10) @(posedge clk);

    rst_n = 1;

    repeat(5) @(posedge clk);


    $display("=== Covariance Estimator Testbench ===");


    // ----------------------------------------------------------------
    // TEST 1: R[0][0] after first frame = X0 * conj(X0) = |X0|^2
    // ----------------------------------------------------------------

    $display("\nTest 1: R[0][0] = |X0|^2 after first frame");

    x0r = 16'd8192;
    x0i = 16'd0;

    send_bin(0, x0r, x0i, 0,0,0,0,0,0);

    repeat(20) @(posedge clk);

    read_elem(0,4'd0);

    exp_re = 16'd2048;

    check_val(rd_re, exp_re, "R[0][0] re");
    check_val(rd_im, 16'd0,  "R[0][0] im");



    // ----------------------------------------------------------------
    // TEST 2: Alpha decay
    // ----------------------------------------------------------------

    $display("\nTest 2: Alpha decay after 2nd frame");

    send_bin(0, x0r, x0i, 0,0,0,0,0,0);

    repeat(20) @(posedge clk);

    read_elem(0,4'd0);

    exp_re = q15_mul(ALPHA,16'd2048)+16'd2048;

    check_val(rd_re,exp_re,"R[0][0] re after 2nd frame");



    // ----------------------------------------------------------------
    // TEST 3: Cross correlation
    // ----------------------------------------------------------------

    $display("\nTest 3: R[0][1]");

    x1r=0;
    x1i=8192;

    send_bin(1,x0r,x0i,x1r,x1i,0,0,0,0);

    repeat(20) @(posedge clk);

    read_elem(1,4'd1);

    exp_re=0;
    exp_im=-2048;

    check_val(rd_re,exp_re,"R[0][1] re");
    check_val(rd_im,exp_im,"R[0][1] im");



    // ----------------------------------------------------------------
    // TEST 4: Hermitian symmetry
    // ----------------------------------------------------------------

    $display("\nTest 4: Hermitian symmetry");

    read_elem(1,4'd4);

    exp_re=0;
    exp_im=2048;

    check_val(rd_re,exp_re,"R[1][0] re");
    check_val(rd_im,exp_im,"R[1][0] im");


    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);

    #200;
    $finish;

end



initial begin
    #(CLK_P*10000);
    $display("TIMEOUT");
    $finish;
end

endmodule