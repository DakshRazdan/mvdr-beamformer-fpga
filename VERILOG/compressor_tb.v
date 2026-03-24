`timescale 1ns/1ps
module compressor_tb;
localparam DW=16, CLK_P=10;
reg clk, rst_n;
reg signed [DW-1:0] pcm_in; reg pcm_valid;
wire signed [DW-1:0] pcm_out; wire pcm_out_valid;

compressor dut(.clk(clk),.rst_n(rst_n),.pcm_in(pcm_in),.pcm_valid(pcm_valid),.pcm_out(pcm_out),.pcm_out_valid(pcm_out_valid));
initial clk=0; always #(CLK_P/2) clk=~clk;

integer pass_count, fail_count;
reg signed [DW-1:0] captured;

task send_and_capture;
    input signed [DW-1:0] val;
    begin
        @(negedge clk); pcm_in=val; pcm_valid=1;
        @(posedge clk); // valid asserted
        @(posedge clk); // output registered
        captured = pcm_out;
        @(negedge clk); pcm_valid=0;
        repeat(2) @(posedge clk);
    end
endtask

task check;
    input signed [DW-1:0] got, lo, hi;
    input [127:0] name;
    begin
        if($signed(got)>=$signed(lo) && $signed(got)<=$signed(hi)) begin
            $display("  PASS %-12s got=%0d",name,$signed(got)); pass_count=pass_count+1;
        end else begin
            $display("  FAIL %-12s got=%0d exp=[%0d,%0d]",name,$signed(got),$signed(lo),$signed(hi)); fail_count=fail_count+1;
        end
    end
endtask

initial begin
    $dumpfile("compressor.vcd"); $dumpvars(0,compressor_tb);
    pass_count=0; fail_count=0; captured=0;
    rst_n=0; pcm_valid=0; pcm_in=0;
    repeat(5) @(posedge clk); rst_n=1; repeat(3) @(posedge clk);
    $display("=== Compressor Testbench ===");

    // Test 1: below threshold input=5000, makeup=1.5 -> ~7500
    $display("\nTest 1: Below threshold (input=5000 exp ~7500)");
    send_and_capture(16'sd5000);
    check(captured, 16'sd6000, 16'sd9000, "pcm_out");

    // Test 2: above threshold input=20000 -> compressed < 20000*1.5
    $display("\nTest 2: Above threshold (input=20000)");
    send_and_capture(16'sd20000);
    if($signed(captured)>0 && $signed(captured)<32767) begin
        $display("  PASS compressed got=%0d",$signed(captured)); pass_count=pass_count+1;
    end else begin
        $display("  FAIL got=%0d",$signed(captured)); fail_count=fail_count+1;
    end

    // Test 3: saturation
    $display("\nTest 3: Saturation (input=32767)");
    send_and_capture(16'sd32767);
    check(captured, 16'sd20000, 16'sd32767, "pcm_out");

    // Test 4: negative input
    $display("\nTest 4: Negative input (input=-20000)");
    send_and_capture(-16'sd20000);
    if($signed(captured)<0) begin
        $display("  PASS negative got=%0d",$signed(captured)); pass_count=pass_count+1;
    end else begin
        $display("  FAIL got=%0d",$signed(captured)); fail_count=fail_count+1;
    end

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #100; $finish;
end
initial begin #100000; $display("TIMEOUT"); $finish; end
endmodule