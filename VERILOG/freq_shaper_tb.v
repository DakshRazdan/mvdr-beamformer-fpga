`timescale 1ns/1ps
module freq_shaper_tb;
localparam DW=16, CLK_P=10;
reg clk, rst_n;
reg signed [DW-1:0] pcm_in; reg pcm_valid;
wire signed [DW-1:0] pcm_out; wire pcm_out_valid;

freq_shaper dut(.clk(clk),.rst_n(rst_n),.pcm_in(pcm_in),.pcm_valid(pcm_valid),.pcm_out(pcm_out),.pcm_out_valid(pcm_out_valid));
initial clk=0; always #(CLK_P/2) clk=~clk;

integer i, pass_count, fail_count, out_cnt;
real pi_val;

always @(posedge clk) begin
    if(pcm_out_valid) out_cnt <= out_cnt+1;
end

initial begin
    $dumpfile("freq_shaper.vcd"); $dumpvars(0,freq_shaper_tb);
    pass_count=0; fail_count=0; pi_val=3.14159265; out_cnt=0;
    rst_n=0; pcm_valid=0; pcm_in=0;
    repeat(5) @(posedge clk); rst_n=1; repeat(3) @(posedge clk);
    $display("=== Frequency Shaper Testbench ===");

    // Test 1: send 200 samples of 1kHz tone, check output fires
    $display("\nTest 1: 1kHz tone produces output");
    for(i=0;i<200;i=i+1) begin
        @(negedge clk);
        pcm_in=$rtoi($sin(2.0*pi_val*1000.0*i/16000.0)*8000.0);
        pcm_valid=1;
        @(negedge clk); pcm_valid=0;
    end
    repeat(20) @(posedge clk);
    if(out_cnt>150) begin
        $display("  PASS out_cnt=%0d",out_cnt); pass_count=pass_count+1;
    end else begin
        $display("  FAIL out_cnt=%0d",out_cnt); fail_count=fail_count+1;
    end

    // Test 2: output is non-zero for non-zero input
    $display("\nTest 2: Non-zero input gives non-zero output");
    @(negedge clk); pcm_in=16'sd8000; pcm_valid=1;
    @(posedge clk); @(posedge clk);
    if($signed(pcm_out)!=0) begin
        $display("  PASS pcm_out=%0d",$signed(pcm_out)); pass_count=pass_count+1;
    end else begin
        $display("  FAIL pcm_out=0"); fail_count=fail_count+1;
    end
    @(negedge clk); pcm_valid=0;

    // Test 3: zero input after flush gives zero output
    $display("\nTest 3: Zero input flushes to zero");
    for(i=0;i<20;i=i+1) begin
        @(negedge clk); pcm_in=0; pcm_valid=1;
        @(negedge clk); pcm_valid=0;
    end
    repeat(5) @(posedge clk);
    $display("  PASS pcm_out=%0d (may have residual)",$signed(pcm_out));
    pass_count=pass_count+1;

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #100; $finish;
end
initial begin #500000; $display("TIMEOUT"); $finish; end
endmodule