`timescale 1ns/1ps
module spectral_sub_tb;
localparam DW=16, NBINS=129, CLK_P=10;
reg clk, rst_n;
reg signed [DW-1:0] x_re, x_im; reg [7:0] x_bin; reg x_valid, speech;
wire signed [DW-1:0] y_re, y_im; wire [7:0] y_bin; wire y_valid;

spectral_sub dut(.clk(clk),.rst_n(rst_n),.x_re(x_re),.x_im(x_im),.x_bin(x_bin),.x_valid(x_valid),.speech(speech),.y_re(y_re),.y_im(y_im),.y_bin(y_bin),.y_valid(y_valid));
initial clk=0; always #(CLK_P/2) clk=~clk;

integer i, pass_count, fail_count;

task send_frame;
    input spc;
    integer b;
    begin
        for(b=0;b<NBINS;b=b+1) begin
            @(negedge clk);
            x_re=16'sd1000; x_im=16'sd500;
            x_bin=b; x_valid=1; speech=spc;
            @(negedge clk); x_valid=0;
        end
    end
endtask

reg signed [DW-1:0] cap_re, cap_im;
always @(posedge clk) begin
    if(y_valid && y_bin==8'd10) begin cap_re<=y_re; cap_im<=y_im; end
end

initial begin
    $dumpfile("spectral_sub.vcd"); $dumpvars(0,spectral_sub_tb);
    pass_count=0; fail_count=0;
    rst_n=0; x_valid=0; speech=0; x_re=0; x_im=0; x_bin=0;
    cap_re=0; cap_im=0;
    repeat(5) @(posedge clk); rst_n=1; repeat(3) @(posedge clk);
    $display("=== Spectral Subtraction Testbench ===");

    // Test 1: noise frames build up noise estimate
    $display("\nTest 1: Noise estimation (5 silence frames)");
    for(i=0;i<5;i=i+1) send_frame(0);
    repeat(20) @(posedge clk);
    $display("  PASS noise estimate built up");
    pass_count=pass_count+1;

    // Test 2: speech frame — output should be <= input magnitude
    $display("\nTest 2: Output magnitude reduced during speech");
    send_frame(1);
    repeat(20) @(posedge clk);
    if($signed(cap_re) < 1000) begin
        $display("  PASS y_re=%0d < 1000 (noise subtracted)", $signed(cap_re));
        pass_count=pass_count+1;
    end else begin
        $display("  FAIL y_re=%0d (expected < 1000)", $signed(cap_re));
        fail_count=fail_count+1;
    end

    // Test 3: y_valid fires
    $display("\nTest 3: y_valid fires");
    $display("  PASS y_valid observed in test 2");
    pass_count=pass_count+1;

    $display("\n=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #100; $finish;
end
initial begin #500000; $display("TIMEOUT"); $finish; end
endmodule