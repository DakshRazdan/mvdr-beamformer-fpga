`timescale 1ns/1ps
`include "cic_decimator.v"
module cic_decimator_tb;

localparam INPUT_WIDTH  = 24;
localparam OUTPUT_WIDTH = 16;
localparam M_STAGES     = 3;
localparam R_DECIMATE   = 192;
localparam CLK_PERIOD   = 10;

reg                     clk, rst_n;
reg  [INPUT_WIDTH-1:0]  x_in;
reg                     x_valid;
wire [OUTPUT_WIDTH-1:0] y_out;
wire                    y_valid;

cic_decimator #(
    .INPUT_WIDTH (INPUT_WIDTH),
    .OUTPUT_WIDTH(OUTPUT_WIDTH),
    .M_STAGES    (M_STAGES),
    .R_DECIMATE  (R_DECIMATE)
) 
dut (
    .clk(clk), .rst_n(rst_n),
    .x_in(x_in), .x_valid(x_valid),
    .y_out(y_out), .y_valid(y_valid)
);

initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// Count inputs and outputs separately to avoid race condition
integer in_count;
integer out_count;

always @(posedge clk) begin
    if (!rst_n) begin
        in_count  <= 0;
        out_count <= 0;
    end else begin
        if (x_valid)  in_count  <= in_count  + 1;
        if (y_valid) begin
            out_count <= out_count + 1;
            $display("  Output %0d: y_out=%0d at in_count=%0d", out_count+1, $signed(y_out), in_count);
        end
    end
end

integer i;
real sine_val;
real pi;

initial begin
    pi      = 3.14159265358979;
    $dumpfile("cic_decimator.vcd");
    $dumpvars(0, cic_decimator_tb);
    rst_n   = 0;
    x_in    = 0;
    x_valid = 0;
    repeat(10) @(posedge clk);
    rst_n <= 1; // Fixed: Use non-blocking to prevent reset race conditions
    repeat(5)  @(posedge clk);

    $display("=== CIC Decimator TB: R=%0d M=%0d ===", R_DECIMATE, M_STAGES);

    // Send exactly R_DECIMATE*30 samples = 30 output samples expected
    $display("Sending %0d input samples (expect %0d outputs)...", R_DECIMATE*30, 30);
    for (i = 0; i < R_DECIMATE*30; i = i+1) begin
        @(posedge clk);
        // sine_val can remain blocking because it is evaluated and consumed immediately 
        sine_val = $sin(2.0 * pi * 1000.0 * i / 3072000.0);
        
        // Fixed: Use non-blocking assignments for synchronous stimulus
        x_in    <= $rtoi(sine_val * 4194304.0); // 2^22
        x_valid <= 1'b1;
    end

    // Stop sending and wait for last output
    @(posedge clk); 
    x_valid <= 0;
    repeat(20) @(posedge clk);

    $display("");
    $display("=== RESULT ===");
    $display("Total inputs:  %0d", in_count);
    $display("Total outputs: %0d", out_count);

    if (out_count == 30)
        $display("PASS: Correct number of outputs (30)");
    else
        $display("FAIL: Got %0d outputs, expected 30", out_count);

    if (out_count > 0) begin
        $display("PASS: CIC is producing output - decimation working");
        $display("Decimation ratio = %0d / %0d = %0d (expected %0d)",
                in_count, out_count, in_count/out_count, R_DECIMATE);
    end

    #100; $finish;
end

initial begin
    #(CLK_PERIOD * R_DECIMATE * 100);
    $display("TIMEOUT"); $finish;
end

endmodule