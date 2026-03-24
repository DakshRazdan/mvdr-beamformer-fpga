`timescale 1ns/1ps
module vad_tb;
localparam DW=16, CLK_P=10;
reg clk, rst_n;
reg signed [DW-1:0] pcm_in;
reg pcm_valid;
wire speech;

vad dut(.clk(clk),.rst_n(rst_n),.pcm_in(pcm_in),.pcm_valid(pcm_valid),.speech(speech));
initial clk=0; always #(CLK_P/2) clk=~clk;

integer i, pass_count, fail_count;
real pi_val;

task send_samples;
    input integer n;
    input signed [15:0] val;
    integer s;
    begin
        for(s=0;s<n;s=s+1) begin
            @(negedge clk); pcm_in=val; pcm_valid=1;
            @(negedge clk); pcm_valid=0;
        end
    end
endtask

initial begin
    $dumpfile("vad.vcd"); $dumpvars(0,vad_tb);
    pass_count=0; fail_count=0; pi_val=3.14159265;
    rst_n=0; pcm_valid=0; pcm_in=0;
    repeat(5) @(posedge clk); rst_n=1; repeat(3) @(posedge clk);
    $display("=== VAD Testbench ===");

    // Test 1: silence (low energy) -> speech=0
    $display("\nTest 1: Silence -> speech=0");
    send_samples(700, 16'sd10);  // very low amplitude
    repeat(400) @(posedge clk);
    if (speech==0) begin $display("  PASS speech=0"); pass_count=pass_count+1; end
    else begin $display("  FAIL speech=%0d",speech); fail_count=fail_count+1; end

    // Test 2: loud sine (speech-like) -> speech=1
    $display("\nTest 2: Speech-like signal -> speech=1");
    for(i=0;i<1000;i=i+1) begin
        @(negedge clk);
        pcm_in = $rtoi($sin(2.0*pi_val*1000.0*i/16000.0) * 8000.0);
        pcm_valid=1;
        @(negedge clk); pcm_valid=0;
    end
    repeat(400) @(posedge clk);
    if (speech==1) begin $display("  PASS speech=1"); pass_count=pass_count+1; end
    else begin $display("  FAIL speech=%0d (exp 1)",speech); fail_count=fail_count+1; end

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #100; $finish;
end
initial begin #5000000; $display("TIMEOUT"); $finish; end
endmodule