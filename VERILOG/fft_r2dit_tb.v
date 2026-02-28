// ============================================================================
// fft_r2dit_tb.v  —  Testbench for 256-point Radix-2 DIT FFT
// ----------------------------------------------------------------------------
// Test 1: DC input (all 1s) -> bin 0 should be peak
// Test 2: Single tone at k=8 -> bin 8 (or 248) should be peak
// Test 3: Frame completes (y_last fires)
// ============================================================================

`timescale 1ns/1ps

module fft_r2dit_tb;

localparam N          = 256;
localparam DW         = 16;
localparam LOGN       = 8;
localparam CLK_PERIOD = 10;

reg  clk, rst_n;
reg  signed [DW-1:0] x_re;
reg  x_valid;
wire signed [DW-1:0] y_re, y_im;
wire y_valid, y_last;

fft_r2dit #(.N(N), .DW(DW), .LOGN(LOGN)) dut (
    .clk(clk), .rst_n(rst_n),
    .x_re(x_re), .x_valid(x_valid),
    .y_re(y_re), .y_im(y_im),
    .y_valid(y_valid), .y_last(y_last)
);

initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// Capture outputs
reg signed [DW-1:0] out_re [0:N-1];
reg signed [DW-1:0] out_im [0:N-1];
reg [8:0] out_count;

always @(posedge clk) begin
    if (rst_n && y_valid) begin
        out_re[out_count] <= y_re;
        out_im[out_count] <= y_im;
        if (out_count == N-1)
            out_count <= 0;
        else
            out_count <= out_count + 1;
    end
end

integer s;
real pi_val;

// Send N samples with x_valid each cycle
task send_frame;
    input [0:0] use_tone;
    integer ss;
    begin
        for (ss = 0; ss < N; ss = ss + 1) begin
            @(negedge clk);
            if (use_tone)
                x_re = $rtoi($cos(2.0*pi_val*8.0*ss/N) * 8192.0);
            else
                x_re = 16'd8192;
            x_valid = 1;
            @(negedge clk);
            x_valid = 0;
        end
    end
endtask

task wait_for_last;
    begin
        @(posedge y_last);
        repeat(5) @(posedge clk);
    end
endtask

integer bin_idx;
integer peak_bin;
reg [31:0] peak_mag2, mag2;

task find_peak;
    begin
        peak_bin  = 0;
        peak_mag2 = 0;
        for (bin_idx = 0; bin_idx < N; bin_idx = bin_idx + 1) begin
            mag2 = out_re[bin_idx]*out_re[bin_idx] + out_im[bin_idx]*out_im[bin_idx];
            if (mag2 > peak_mag2) begin
                peak_mag2 = mag2;
                peak_bin  = bin_idx;
            end
        end
    end
endtask

integer pass_count, fail_count;

initial begin
    $dumpfile("fft_r2dit.vcd");
    $dumpvars(0, fft_r2dit_tb);
    pi_val     = 3.14159265358979;
    pass_count = 0; fail_count = 0;
    rst_n = 0; x_valid = 0; x_re = 0; out_count = 0;
    repeat(10) @(posedge clk);
    rst_n = 1;
    repeat(5)  @(posedge clk);

    $display("=== FFT 256-point Testbench ===");

    // Test 1: DC input
    $display("\nTest 1: DC input -> peak expected at bin 0");
    send_frame(0);
    wait_for_last;
    find_peak;
    $display("  Peak bin = %0d, mag2 = %0d", peak_bin, peak_mag2);
    $display("  Bin[0] = (%0d, %0d)", $signed(out_re[0]), $signed(out_im[0]));
    if (peak_bin == 0) begin
        $display("  PASS"); pass_count = pass_count + 1;
    end else begin
        $display("  FAIL"); fail_count = fail_count + 1;
    end

    // Test 2: Tone at bin 8
    $display("\nTest 2: Tone k=8 -> peak at bin 8 or 248");
    send_frame(1);
    wait_for_last;
    find_peak;
    $display("  Peak bin = %0d, mag2 = %0d", peak_bin, peak_mag2);
    $display("  Bin[8]   = (%0d, %0d)", $signed(out_re[8]),   $signed(out_im[8]));
    $display("  Bin[248] = (%0d, %0d)", $signed(out_re[248]), $signed(out_im[248]));
    if (peak_bin == 8 || peak_bin == 248) begin
        $display("  PASS"); pass_count = pass_count + 1;
    end else begin
        $display("  FAIL"); fail_count = fail_count + 1;
    end

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #100; $finish;
end

initial begin
    #(CLK_PERIOD * 100000);
    $display("TIMEOUT"); $finish;
end

endmodule
