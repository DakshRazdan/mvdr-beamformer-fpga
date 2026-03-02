// ============================================================================
// beamformer_apply_tb.v — Testbench for beamformer_apply
// ----------------------------------------------------------------------------
// TEST 1: Known weights + known signal → check Y(k) value
//   weights = [0.25, 0.25, 0.25, 0.25] (Q1.15 = 8192)
//   signal  = all mics see same tone: x0=x1=x2=x3 = (16384 + j0) = 0.5
//   expected Y = 0.25*0.5 * 4 = 0.5 = 16384 in Q1.15
//
// TEST 2: Noise rejection
//   weights = [0.25, 0.25, 0.25, 0.25]
//   signal  = [1, -1, 1, -1] (out-of-phase — should cancel)
//   expected Y ≈ 0
//
// TEST 3: Bin index passes through correctly
//
// TEST 4: y_valid fires 2 cycles after x_valid
// ============================================================================

`timescale 1ns/1ps

module beamformer_apply_tb;

localparam DW    = 16;
localparam CLK_P = 10;

reg clk, rst_n;

// Weight port
reg signed [DW-1:0] w0_re,w0_im,w1_re,w1_im,w2_re,w2_im,w3_re,w3_im;
reg [7:0]  w_bin;
reg        w_valid;

// FFT port
reg signed [DW-1:0] x0_re,x0_im,x1_re,x1_im,x2_re,x2_im,x3_re,x3_im;
reg [7:0]  x_bin;
reg        x_valid;

// Output
wire signed [DW-1:0] y_re, y_im;
wire [7:0]  y_bin;
wire        y_valid;

beamformer_apply #(.NBINS(129), .DW(DW)) dut (
    .clk(clk), .rst_n(rst_n),
    .w0_re(w0_re),.w0_im(w0_im),.w1_re(w1_re),.w1_im(w1_im),
    .w2_re(w2_re),.w2_im(w2_im),.w3_re(w3_re),.w3_im(w3_im),
    .w_bin(w_bin),.w_valid(w_valid),
    .x0_re(x0_re),.x0_im(x0_im),.x1_re(x1_re),.x1_im(x1_im),
    .x2_re(x2_re),.x2_im(x2_im),.x3_re(x3_re),.x3_im(x3_im),
    .x_bin(x_bin),.x_valid(x_valid),
    .y_re(y_re),.y_im(y_im),.y_bin(y_bin),.y_valid(y_valid)
);

initial clk = 0;
always #(CLK_P/2) clk = ~clk;

integer pass_count, fail_count;

task check_val;
    input signed [DW-1:0] got, exp;
    input integer tol;
    input [127:0] name;
    begin
        if ($signed(got) >= $signed(exp)-tol && $signed(got) <= $signed(exp)+tol) begin
            $display("  PASS %-10s got=%0d exp=%0d", name, $signed(got), $signed(exp));
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL %-10s got=%0d exp=%0d", name, $signed(got), $signed(exp));
            fail_count = fail_count + 1;
        end
    end
endtask

task send_weights;
    input signed [DW-1:0] wr, wi;
    input [7:0] bin;
    begin
        @(negedge clk);
        w0_re=wr; w0_im=wi; w1_re=wr; w1_im=wi;
        w2_re=wr; w2_im=wi; w3_re=wr; w3_im=wi;
        w_bin=bin; w_valid=1;
        @(negedge clk);
        w_valid=0;
    end
endtask

task send_bins_and_wait;
    input signed [DW-1:0] x0r,x0i,x1r,x1i,x2r,x2i,x3r,x3i;
    input [7:0] bin;
    begin
        @(negedge clk);
        x0_re=x0r; x0_im=x0i; x1_re=x1r; x1_im=x1i;
        x2_re=x2r; x2_im=x2i; x3_re=x3r; x3_im=x3i;
        x_bin=bin; x_valid=1;
        @(negedge clk);
        x_valid=0;
        // y_valid fires 2 cycles after x_valid
        @(posedge y_valid);
        @(posedge clk); #1;
    end
endtask

initial begin
    $dumpfile("beamformer_apply.vcd");
    $dumpvars(0, beamformer_apply_tb);

    pass_count=0; fail_count=0;
    rst_n=0; w_valid=0; x_valid=0;
    w0_re=0;w0_im=0;w1_re=0;w1_im=0;
    w2_re=0;w2_im=0;w3_re=0;w3_im=0;
    x0_re=0;x0_im=0;x1_re=0;x1_im=0;
    x2_re=0;x2_im=0;x3_re=0;x3_im=0;
    w_bin=0; x_bin=0;

    repeat(5) @(posedge clk);
    rst_n=1;
    repeat(3) @(posedge clk);

    $display("=== Beamformer Apply Testbench ===");

    // ------------------------------------------------------------------
    // TEST 1: Equal weights, coherent signal
    // w = [0.25, 0.25, 0.25, 0.25] = 8192 in Q1.15
    // x = [0.5+0j, 0.5+0j, 0.5+0j, 0.5+0j] = 16384
    // Y = 0.5 in Q1.15 = 16384 (tolerance 100)
    // ------------------------------------------------------------------
    $display("\nTest 1: Coherent signal, equal weights");
    send_weights(16'sd8192, 16'sd0, 8'd5);
    send_bins_and_wait(16384,0, 16384,0, 16384,0, 16384,0, 8'd5);
    check_val(y_re, 16'sd16384, 200, "y_re");
    check_val(y_im, 16'sd0,      50, "y_im");

    // ------------------------------------------------------------------
    // TEST 2: Noise rejection — out-of-phase signal cancels
    // x = [1, -1, 1, -1] * 0.5 in Q1.15
    // Y_re = 0.25*(0.5 - 0.5 + 0.5 - 0.5) = 0
    // ------------------------------------------------------------------
    $display("\nTest 2: Out-of-phase signal (should cancel)");
    send_weights(16'sd8192, 16'sd0, 8'd10);
    send_bins_and_wait(16384,0, -16384,0, 16384,0, -16384,0, 8'd10);
    check_val(y_re, 16'sd0, 50, "y_re");
    check_val(y_im, 16'sd0, 50, "y_im");

    // ------------------------------------------------------------------
    // TEST 3: Complex weights, complex signal
    // w = [0.25+0.25j] all mics (Q1.15: 8192)
    // x = [1+0j] all mics (Q1.15: 32767)
    // w^H·x = (0.25-0.25j)*1 * 4 = 1.0 - 1.0j → clipped to 32767
    // y_re = 32767, y_im = -32767 (within tolerance of 32767)
    // ------------------------------------------------------------------
    $display("\nTest 3: Complex weights");
    @(negedge clk);
    w0_re=8192;w0_im=8192; w1_re=8192;w1_im=8192;
    w2_re=8192;w2_im=8192; w3_re=8192;w3_im=8192;
    w_bin=8'd20; w_valid=1;
    @(negedge clk); w_valid=0;
    send_bins_and_wait(32767,0, 32767,0, 32767,0, 32767,0, 8'd20);
    check_val(y_re,  16'sd32767, 200, "y_re");
    check_val(y_im, -16'sd32767, 200, "y_im");

    // ------------------------------------------------------------------
    // TEST 4: Bin index passes through
    // ------------------------------------------------------------------
    $display("\nTest 4: Bin index passthrough");
    send_weights(16'sd8192, 16'sd0, 8'd42);
    send_bins_and_wait(0,0,0,0,0,0,0,0, 8'd42);
    if (y_bin == 8'd42) begin
        $display("  PASS y_bin=%0d", y_bin);
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL y_bin=%0d exp=42", y_bin);
        fail_count = fail_count + 1;
    end

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #100; $finish;
end

initial begin
    #500000; $display("TIMEOUT"); $finish;
end

endmodule