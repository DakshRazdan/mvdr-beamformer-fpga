// ============================================================================
// covariance_est_tb.v  —  Testbench for Covariance Matrix Estimator
// ----------------------------------------------------------------------------
// Test 1: R[0][0] = |X0|^2 after first frame
// Test 2: Alpha decay after second frame
// Test 3: R[0][1] = X0 * conj(X1) cross-correlation
// Test 4: R[1][0] = conj(R[0][1]) Hermitian symmetry
// ============================================================================

`timescale 1ns/1ps

module covariance_est_tb;

localparam DW    = 16;
localparam NBINS = 129;
localparam CLK_P = 10;
localparam signed [DW-1:0] ALPHA = 16'd31129;

reg clk, rst_n;

// DUT inputs — flat ports
reg signed [DW-1:0] x0_re, x0_im;
reg signed [DW-1:0] x1_re, x1_im;
reg signed [DW-1:0] x2_re, x2_im;
reg signed [DW-1:0] x3_re, x3_im;
reg [7:0]  x_bin;
reg        x_valid;

// Read port
reg [7:0]  rd_bin;
reg [3:0]  rd_elem;
reg        rd_en;
wire signed [DW-1:0] rd_re, rd_im;
wire                 rd_valid;

covariance_est #(.NBINS(NBINS), .DW(DW), .ALPHA(ALPHA)) dut (
    .clk(clk), .rst_n(rst_n),
    .x0_re(x0_re), .x0_im(x0_im),
    .x1_re(x1_re), .x1_im(x1_im),
    .x2_re(x2_re), .x2_im(x2_im),
    .x3_re(x3_re), .x3_im(x3_im),
    .x_bin(x_bin), .x_valid(x_valid),
    .rd_bin(rd_bin), .rd_elem(rd_elem), .rd_en(rd_en),
    .rd_re(rd_re), .rd_im(rd_im), .rd_valid(rd_valid)
);

initial clk = 0;
always #(CLK_P/2) clk = ~clk;

// Q1.15 multiply helper
function signed [DW-1:0] q15_mul;
    input signed [DW-1:0] a, b;
    reg signed [2*DW-1:0] p;
    begin p = a * b; q15_mul = p >>> 15; end
endfunction

// Send one bin update (1-cycle pulse)
task send_bin;
    input [7:0]           bin;
    input signed [DW-1:0] r0,i0,r1,i1,r2,i2,r3,i3;
    begin
        @(negedge clk);
        x0_re=r0; x0_im=i0;
        x1_re=r1; x1_im=i1;
        x2_re=r2; x2_im=i2;
        x3_re=r3; x3_im=i3;
        x_bin=bin; x_valid=1;
        @(negedge clk);
        x_valid=0;
    end
endtask

// Read one element — 1 cycle latency
task read_elem;
    input [7:0] bin;
    input [3:0] elem;
    begin
        @(negedge clk);
        rd_bin=bin; rd_elem=elem; rd_en=1;
        @(posedge clk); // rd_valid fires here
        @(negedge clk);
        rd_en=0;
        @(posedge clk); // data stable
    end
endtask

integer pass_count, fail_count;

task check_val;
    input signed [DW-1:0] got, exp;
    input [127:0] name;
    begin
        if (got >= exp-2 && got <= exp+2) begin
            $display("  PASS %-12s got=%0d exp=%0d", name, $signed(got), $signed(exp));
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL %-12s got=%0d exp=%0d", name, $signed(got), $signed(exp));
            fail_count = fail_count + 1;
        end
    end
endtask

reg signed [DW-1:0] exp_re, exp_im;

initial begin
    $dumpfile("covariance_est.vcd");
    $dumpvars(0, covariance_est_tb);

    pass_count=0; fail_count=0;
    rst_n=0; x_valid=0; rd_en=0;
    x0_re=0; x0_im=0; x1_re=0; x1_im=0;
    x2_re=0; x2_im=0; x3_re=0; x3_im=0;
    x_bin=0; rd_bin=0; rd_elem=0;

    repeat(10) @(posedge clk);
    rst_n=1;
    repeat(5)  @(posedge clk);

    $display("=== Covariance Estimator Testbench ===");

    // ----------------------------------------------------------
    // TEST 1: R[0][0] = |X0|^2
    // X0=8192+0j -> R[0][0] = (8192*8192)>>15 = 2048
    // ----------------------------------------------------------
    $display("\nTest 1: R[0][0] = |X0|^2");
    send_bin(0, 8192,0, 0,0, 0,0, 0,0);
    repeat(25) @(posedge clk);
    read_elem(0, 0);

    check_val(rd_re, 16'd2048, "R00 re");
    check_val(rd_im, 16'd0,    "R00 im");

    // ----------------------------------------------------------
    // TEST 2: Alpha decay — send same bin again
    // new R[0][0] = alpha*2048 + 2048
    // ----------------------------------------------------------
    $display("\nTest 2: Alpha decay");
    send_bin(0, 8192,0, 0,0, 0,0, 0,0);
    repeat(25) @(posedge clk);
    read_elem(0, 0);

    exp_re = q15_mul(ALPHA, 16'd2048) + 16'd2048;
    check_val(rd_re, exp_re, "R00 alpha");

    // ----------------------------------------------------------
    // TEST 3: R[0][1] = X0*conj(X1)
    // X0=8192+0j, X1=0+8192j
    // re = 8192*0 + 0*8192 = 0
    // im = 0*0 - 8192*8192 = -67108864 >> 15 = -2048
    // ----------------------------------------------------------
    $display("\nTest 3: R[0][1] = X0*conj(X1)");
    send_bin(1, 8192,0, 0,8192, 0,0, 0,0);
    repeat(25) @(posedge clk);
    read_elem(1, 1); // elem 1 = row0*4+col1

    check_val(rd_re, 16'd0,    "R01 re");
    check_val(rd_im, -16'd2048,"R01 im");

    // ----------------------------------------------------------
    // TEST 4: Hermitian — R[1][0] = conj(R[0][1]) = (0, +2048)
    // ----------------------------------------------------------
    $display("\nTest 4: Hermitian R[1][0] = conj(R[0][1])");
    read_elem(1, 4); // elem 4 = row1*4+col0

    check_val(rd_re, 16'd0,   "R10 re");
    check_val(rd_im, 16'd2048,"R10 im");

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #200; $finish;
end

initial begin
    #(CLK_P * 10000);
    $display("TIMEOUT"); $finish;
end

endmodule