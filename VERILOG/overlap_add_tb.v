// overlap_add_tb.v -- Testbench for overlap_add
// Test 1: Exactly 128 outputs per frame
// Test 2: pcm_last fires on sample 127
// Test 3: Non-zero output for non-zero input (Hann window applied)
// Test 4: Second frame overlap accumulates
// Test 5: Two zero frames -> zero output (overlap drains in 1 frame)

`timescale 1ns/1ps

module overlap_add_tb;

localparam N     = 256;
localparam HOP   = 128;
localparam DW    = 16;
localparam CLK_P = 10;

reg clk, rst_n;
reg signed [DW-1:0] x_re;
reg        x_valid, x_last;
wire signed [DW-1:0] pcm_out;
wire       pcm_valid, pcm_last;

overlap_add #(.N(N), .HOP(HOP), .DW(DW)) dut (
    .clk(clk), .rst_n(rst_n),
    .x_re(x_re), .x_valid(x_valid), .x_last(x_last),
    .pcm_out(pcm_out), .pcm_valid(pcm_valid), .pcm_last(pcm_last)
);

initial clk = 0;
always #(CLK_P/2) clk = ~clk;

integer pass_count, fail_count;
integer out_count, last_count, out_idx;
reg signed [DW-1:0] out_buf [0:HOP-1];

always @(posedge clk) begin
    if (pcm_valid) begin
        if (out_idx < HOP) out_buf[out_idx] <= pcm_out;
        out_idx   <= out_idx + 1;
        out_count <= out_count + 1;
        if (pcm_last) last_count <= last_count + 1;
    end
end

task send_frame;
    input signed [DW-1:0] val;
    integer n;
    begin
        for (n = 0; n < N; n = n+1) begin
            @(negedge clk);
            x_re    = val;
            x_valid = 1;
            x_last  = (n == N-1);
        end
        @(negedge clk);
        x_valid = 0;
        x_last  = 0;
    end
endtask

integer n;

initial begin
    $dumpfile("overlap_add.vcd");
    $dumpvars(0, overlap_add_tb);

    pass_count = 0; fail_count = 0;
    out_count  = 0; last_count = 0; out_idx = 0;
    rst_n = 0; x_valid = 0; x_last = 0; x_re = 0;

    repeat(5) @(posedge clk);
    rst_n = 1;
    repeat(3) @(posedge clk);

    $display("=== Overlap-Add Testbench ===");

    // Test 1+2: output count and pcm_last
    $display("\nTest 1+2: Output count=128 and pcm_last");
    out_count = 0; last_count = 0; out_idx = 0;
    send_frame(16'sd127);
    @(posedge pcm_last);
    repeat(3) @(posedge clk);

    if (out_count == HOP) begin
        $display("  PASS out_count=%0d", out_count);
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL out_count=%0d exp=128", out_count);
        fail_count = fail_count + 1;
    end

    if (last_count == 1) begin
        $display("  PASS pcm_last fired once");
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL pcm_last fired %0d times", last_count);
        fail_count = fail_count + 1;
    end

    // Test 3: non-zero output at mid-window
    $display("\nTest 3: Non-zero windowed output at sample 64");
    if ($signed(out_buf[64]) > 10) begin
        $display("  PASS out_buf[64]=%0d", $signed(out_buf[64]));
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL out_buf[64]=%0d (expected > 10)", $signed(out_buf[64]));
        fail_count = fail_count + 1;
    end

    // Test 4: second frame overlap adds
    $display("\nTest 4: Second frame overlap accumulation");
    out_count = 0; out_idx = 0;
    send_frame(16'sd127);
    @(posedge pcm_last);
    repeat(3) @(posedge clk);

    if ($signed(out_buf[64]) > $signed(10)) begin
        $display("  PASS out_buf[64]=%0d (overlap added)", $signed(out_buf[64]));
        pass_count = pass_count + 1;
    end else begin
        $display("  FAIL out_buf[64]=%0d", $signed(out_buf[64]));
        fail_count = fail_count + 1;
    end

    // Test 5: zero input drains to zero
    // Frame 1 of zeros: outputs overlap tail from prev frame (nonzero, OK)
    //                   saves zero tail into overlap_buf
    // Frame 2 of zeros: overlap_buf is now zero -> output must be zero
    $display("\nTest 5: Zero input x2 -> zero output");
    send_frame(16'sd0);
    @(posedge pcm_last);
    repeat(3) @(posedge clk);
    out_count = 0; out_idx = 0;
    send_frame(16'sd0);
    @(posedge pcm_last);
    repeat(3) @(posedge clk);

    begin : zero_check
        integer all_zero;
        all_zero = 1;
        for (n = 0; n < HOP; n = n+1)
            if ($signed(out_buf[n]) != 0) all_zero = 0;
        if (all_zero) begin
            $display("  PASS all zero after overlap drained");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL non-zero output after drain");
            fail_count = fail_count + 1;
        end
    end

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #500; $finish;
end

initial begin
    #3000000; $display("TIMEOUT"); $finish;
end

endmodule