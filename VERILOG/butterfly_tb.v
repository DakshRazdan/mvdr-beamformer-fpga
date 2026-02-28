// ============================================================================
// butterfly_tb.v  —  Testbench for Butterfly Unit
// ----------------------------------------------------------------------------
// Verifies: P = A + W*B  and  Q = A - W*B
//
// Test cases:
//   1. W=1 (k=0): P = A+B, Q = A-B  (simplest case)
//   2. W=j (k=N/4): P = A + jB, Q = A - jB
//   3. W=-1 (k=N/2): P = A-B, Q = A+B
// ============================================================================

`timescale 1ns/1ps

module butterfly_tb;

localparam DW = 16;
localparam CLK_PERIOD = 10;

reg clk, rst_n, valid_in;
reg signed [DW-1:0] ar, ai, br, bi, wr, wi;
wire signed [DW-1:0] pr, pi_out, qr, qi;
wire valid_out;

butterfly #(.DW(DW)) dut (
    .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
    .ar(ar), .ai(ai), .br(br), .bi(bi),
    .wr(wr), .wi(wi),
    .pr(pr), .pi(pi_out), .qr(qr), .qi(qi),
    .valid_out(valid_out)
);

initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// Q1.15 scale factor
localparam real SCALE = 32767.0;

integer pass_count, fail_count;

task check;
    input signed [DW-1:0] got_pr, got_pi, got_qr, got_qi;
    input signed [DW-1:0] exp_pr, exp_pi, exp_qr, exp_qi;
    input [63:0] test_num;
    begin
        if (got_pr === exp_pr && got_pi === exp_pi &&
            got_qr === exp_qr && got_qi === exp_qi) begin
            $display("  PASS test %0d: P=(%0d,%0d) Q=(%0d,%0d)",
                     test_num, got_pr, got_pi, got_qr, got_qi);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL test %0d:", test_num);
            $display("    P got=(%0d,%0d) exp=(%0d,%0d)",
                     got_pr, got_pi, exp_pr, exp_pi);
            $display("    Q got=(%0d,%0d) exp=(%0d,%0d)",
                     got_qr, got_qi, exp_qr, exp_qi);
            fail_count = fail_count + 1;
        end
    end
endtask

initial begin
    $dumpfile("butterfly.vcd");
    $dumpvars(0, butterfly_tb);

    pass_count = 0; fail_count = 0;
    rst_n = 0; valid_in = 0;
    ar = 0; ai = 0; br = 0; bi = 0; wr = 0; wi = 0;

    repeat(5) @(posedge clk);
    rst_n = 1;
    repeat(2) @(posedge clk);

    $display("=== Butterfly Testbench ===");

    // Test 1: W = 1+0j (twiddle k=0)
    // A = 100+0j, B = 50+0j
    // Expected: P = 150+0j, Q = 50+0j
    $display("Test 1: W=1, A=100, B=50 -> P=150, Q=50");
    @(posedge clk);
    ar = 16'd100; ai = 16'd0;
    br = 16'd50;  bi = 16'd0;
    wr = 16'd32767; wi = 16'd0;  // W = 1.0 in Q1.15
    valid_in = 1;
    @(posedge clk); valid_in = 0;

    // Wait 3 pipeline cycles
    repeat(3) @(posedge clk);
    // Allow one more for registered output
    @(posedge clk);

    // Check with tolerance of ±1 due to fixed-point rounding
    if ($signed(pr) >= 149 && $signed(pr) <= 151 &&
        $signed(qr) >= 49  && $signed(qr) <= 51  &&
        $signed(pi_out) >= -1 && $signed(pi_out) <= 1 &&
        $signed(qi) >= -1 && $signed(qi) <= 1) begin
        $display("  PASS: P=(%0d,%0d) Q=(%0d,%0d)", pr, pi_out, qr, qi);
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL: P=(%0d,%0d) Q=(%0d,%0d) [exp P=(150,0) Q=(50,0)]",
                 pr, pi_out, qr, qi);
        fail_count = fail_count + 1;
    end

    repeat(3) @(posedge clk);

    // Test 2: W = -1+0j (twiddle k=N/2=128, use -32768)
    // A = 100+0j, B = 50+0j
    // W*B = -50+0j
    // Expected: P = 50+0j, Q = 150+0j
    $display("Test 2: W=-1, A=100, B=50 -> P=50, Q=150");
    @(posedge clk);
    ar = 16'd100; ai = 16'd0;
    br = 16'd50;  bi = 16'd0;
    wr = -16'd32767; wi = 16'd0;  // W = -1.0
    valid_in = 1;
    @(posedge clk); valid_in = 0;

    repeat(4) @(posedge clk);

    if ($signed(pr) >= 49 && $signed(pr) <= 51 &&
        $signed(qr) >= 149 && $signed(qr) <= 151) begin
        $display("  PASS: P=(%0d,%0d) Q=(%0d,%0d)", pr, pi_out, qr, qi);
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL: P=(%0d,%0d) Q=(%0d,%0d) [exp P=(50,0) Q=(150,0)]",
                 pr, pi_out, qr, qi);
        fail_count = fail_count + 1;
    end

    repeat(3) @(posedge clk);

    // Test 3: W = 0-1j (pure imaginary, k=N/4=64)
    // A = 100+0j, B = 0+50j
    // W*B = (0-j)(0+50j) = 50+0j   [since -j * j = -j^2 = 1]
    // Expected: P = 150+0j, Q = 50+0j
    $display("Test 3: W=-j, A=100+0j, B=0+50j -> P=(100+50), Q=(100-50)");
    @(posedge clk);
    ar = 16'd100; ai = 16'd0;
    br = 16'd0;   bi = 16'd50;
    wr = 16'd0;   wi = -16'd32767;  // W = -j
    valid_in = 1;
    @(posedge clk); valid_in = 0;

    repeat(4) @(posedge clk);

    if ($signed(pr) >= 149 && $signed(pr) <= 151 &&
        $signed(qr) >= 49  && $signed(qr) <= 51) begin
        $display("  PASS: P=(%0d,%0d) Q=(%0d,%0d)", pr, pi_out, qr, qi);
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL: P=(%0d,%0d) Q=(%0d,%0d)", pr, pi_out, qr, qi);
        fail_count = fail_count + 1;
    end

    $display("");
    $display("=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #100; $finish;
end

initial begin
    #(CLK_PERIOD * 200);
    $display("TIMEOUT"); $finish;
end

endmodule
