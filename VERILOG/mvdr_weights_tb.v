// ============================================================================
// mvdr_weights_tb.v  —  Testbench for MVDR Weight Computation
// ----------------------------------------------------------------------------
// Tests MVDR weights with a known covariance matrix
//
// TEST SETUP:
//   Identity matrix R = I (no noise correlation)
//   With diagonal loading: R_loaded = I + delta*I = (1+delta)*I
//   For d=[1,1,1,1]: w = R^{-1}*d / (d^H*R^{-1}*d)
//                      = (1/(1+delta))*[1,1,1,1] / (4/(1+delta))
//                      = [0.25, 0.25, 0.25, 0.25]
//
//   In Q1.15: 0.25 * 32767 = 8192
//
// Test 1: Weight sum = 1.0 (distortionless constraint)
// Test 2: All weights equal (symmetric array, frontal target)
// Test 3: w_valid fires after compute pulse
// ============================================================================

`timescale 1ns/1ps

module mvdr_weights_tb;

localparam DW    = 16;
localparam CLK_P = 10;

reg clk, rst_n;

// DUT
reg        compute;
reg [7:0]  bin_in;

wire [7:0]  rd_bin;
wire [3:0]  rd_elem;
wire        rd_en;
reg  signed [DW-1:0] rd_re_r, rd_im_r;
reg         rd_valid_r;

wire signed [DW-1:0] w0_re,w0_im,w1_re,w1_im,w2_re,w2_im,w3_re,w3_im;
wire [7:0]  w_bin;
wire        w_valid;

mvdr_weights #(.NBINS(129), .DW(DW)) dut (
    .clk(clk), .rst_n(rst_n),
    .compute(compute), .bin_in(bin_in),
    .rd_bin(rd_bin), .rd_elem(rd_elem), .rd_en(rd_en),
    .rd_re(rd_re_r), .rd_im(rd_im_r), .rd_valid(rd_valid_r),
    .w0_re(w0_re), .w0_im(w0_im),
    .w1_re(w1_re), .w1_im(w1_im),
    .w2_re(w2_re), .w2_im(w2_im),
    .w3_re(w3_re), .w3_im(w3_im),
    .w_bin(w_bin), .w_valid(w_valid)
);

initial clk = 0;
always #(CLK_P/2) clk = ~clk;

// ============================================================================
// MOCK COVARIANCE RAM
// Respond to rd_en with identity matrix (diagonal = 1.0 in Q1.15 = 32767)
// R[i][j] = 32767 if i==j, else 0
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_re_r    <= 0;
        rd_im_r    <= 0;
        rd_valid_r <= 0;
    end else begin
        rd_valid_r <= rd_en;
        if (rd_en) begin
            // elem = row*4 + col, diagonal when row==col (elem 0,5,10,15)
            if (rd_elem==4'd0 || rd_elem==4'd5 ||
                rd_elem==4'd10|| rd_elem==4'd15) begin
                rd_re_r <= 16'd32767;  // 1.0 in Q1.15
                rd_im_r <= 16'd0;
            end else begin
                rd_re_r <= 16'd0;
                rd_im_r <= 16'd0;
            end
        end
    end
end

// ============================================================================
// TESTS
// ============================================================================
integer pass_count, fail_count;

task check_val;
    input signed [DW-1:0] got, exp;
    input integer tol;
    input [127:0] name;
    begin
        if ($signed(got) >= $signed(exp)-tol &&
            $signed(got) <= $signed(exp)+tol) begin
            $display("  PASS %-12s got=%0d exp=%0d",
                    name, $signed(got), $signed(exp));
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL %-12s got=%0d exp=%0d",
                    name, $signed(got), $signed(exp));
            fail_count = fail_count + 1;
        end
    end
endtask

integer sum_re;

initial begin
    $dumpfile("mvdr_weights.vcd");
    $dumpvars(0, mvdr_weights_tb);

    pass_count=0; fail_count=0;
    rst_n=0; compute=0; bin_in=0;

    repeat(10) @(posedge clk);
    rst_n=1;
    repeat(5) @(posedge clk);

    $display("=== MVDR Weights Testbench ===");
    $display("Input: Identity R, d=[1,1,1,1]");
    $display("Expected: w = [0.25, 0.25, 0.25, 0.25] = [8192, 8192, 8192, 8192]");

    // Trigger computation for bin 0
    @(negedge clk);
    bin_in  = 8'd0;
    compute = 1;
    @(negedge clk);
    compute = 0;

    // Wait for w_valid
    @(posedge w_valid);
    @(posedge clk); // settle

    $display("\nTest 1: Equal weights (symmetric array)");
    check_val(w0_re, 16'd8192, 200, "w0_re");
    check_val(w1_re, 16'd8192, 200, "w1_re");
    check_val(w2_re, 16'd8192, 200, "w2_re");
    check_val(w3_re, 16'd8192, 200, "w3_re");

    $display("\nTest 2: Imaginary parts near zero");
    check_val(w0_im, 16'd0, 200, "w0_im");
    check_val(w1_im, 16'd0, 200, "w1_im");
    check_val(w2_im, 16'd0, 200, "w2_im");
    check_val(w3_im, 16'd0, 200, "w3_im");

    $display("\nTest 3: Bin index correct");
    if (w_bin == 0) begin
        $display("  PASS w_bin=%0d", w_bin);
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL w_bin=%0d exp=0", w_bin);
        fail_count = fail_count + 1;
    end

    $display("\nWeights: w0=(%0d,%0d) w1=(%0d,%0d) w2=(%0d,%0d) w3=(%0d,%0d)",
        $signed(w0_re),$signed(w0_im),
        $signed(w1_re),$signed(w1_im),
        $signed(w2_re),$signed(w2_im),
        $signed(w3_re),$signed(w3_im));

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #500; $finish;
end

initial begin
    #(CLK_P * 50000);
    $display("TIMEOUT"); $finish;
end

endmodule