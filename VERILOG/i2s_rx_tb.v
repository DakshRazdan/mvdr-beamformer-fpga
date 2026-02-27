`timescale 1ns/1ps

module i2s_rx_tb;

localparam SYS_CLK_FREQ = 100_000_000;
localparam BCLK_FREQ    = 3_072_000;
localparam DATA_WIDTH   = 24;

localparam [23:0] TEST_LEFT  = 24'hA5_B6_C7;
localparam [23:0] TEST_RIGHT = 24'h12_34_56;

localparam SYS_CLK_PERIOD = 10;           // 10ns = 100MHz
localparam BCLK_PERIOD    = 326;          // ~326ns = 3.072MHz

reg         clk, rst_n;
reg         bclk, ws, sdata;
wire [23:0] left_out, right_out;
wire        valid;

i2s_rx #(.DATA_WIDTH(DATA_WIDTH)) dut (
    .clk(clk), .rst_n(rst_n),
    .bclk(bclk), .ws(ws), .sdata(sdata),
    .left_out(left_out), .right_out(right_out), .valid(valid)
);

initial clk = 0;
always #(SYS_CLK_PERIOD/2) clk = ~clk;

initial bclk = 0;
always #(BCLK_PERIOD/2) bclk = ~bclk;

// I2S frame task ? correct I2S timing:
// WS transitions on negedge BCLK, then ONE dead BCLK cycle, then data starts
task send_i2s_frame;
    input [23:0] left_data;
    input [23:0] right_data;
    integer i;
    begin
        // LEFT CHANNEL (WS low)
        @(negedge bclk); ws = 1'b0;
        @(negedge bclk); // I2S spec: 1 dead cycle after WS transition

        for (i = 23; i >= 0; i = i-1) begin
            @(negedge bclk);
            sdata = left_data[i];
        end
        repeat(7) @(negedge bclk); sdata = 0; // padding

        // RIGHT CHANNEL (WS high)
        @(negedge bclk); ws = 1'b1;
        @(negedge bclk); // I2S spec: 1 dead cycle

        for (i = 23; i >= 0; i = i-1) begin
            @(negedge bclk);
            sdata = right_data[i];
        end
        repeat(7) @(negedge bclk); sdata = 0;
    end
endtask

integer pass_count, fail_count;

initial begin
    rst_n = 0; ws = 1; sdata = 0;
    pass_count = 0; fail_count = 0;
    repeat(10) @(posedge clk);
    rst_n = 1;
    repeat(5) @(posedge clk);

    $display("=== I2S RX Testbench ===");
    $display("Sending TEST_LEFT=0x%06X  TEST_RIGHT=0x%06X", TEST_LEFT, TEST_RIGHT);

    // Send 5 frames to let synchronizer settle
    repeat(5) send_i2s_frame(TEST_LEFT, TEST_RIGHT);
    repeat(50) @(posedge clk);

    $display("Captured: left_out=0x%06X  right_out=0x%06X", left_out, right_out);

    if (left_out  === TEST_LEFT)  begin $display("PASS: left_out correct");  pass_count=pass_count+1; end
    else                               begin $display("FAIL: left_out  got 0x%06X expected 0x%06X", left_out, TEST_LEFT); fail_count=fail_count+1; end

    if (right_out === TEST_RIGHT) begin $display("PASS: right_out correct"); pass_count=pass_count+1; end
    else                               begin $display("FAIL: right_out got 0x%06X expected 0x%06X", right_out, TEST_RIGHT); fail_count=fail_count+1; end

    // Test update
    repeat(3) send_i2s_frame(24'hFFFFFF, 24'h000001);
    repeat(50) @(posedge clk);

    if (left_out  === 24'hFFFFFF) begin $display("PASS: update left_out");  pass_count=pass_count+1; end
    else                               begin $display("FAIL: update left_out=0x%06X", left_out); fail_count=fail_count+1; end

    if (right_out === 24'h000001) begin $display("PASS: update right_out"); pass_count=pass_count+1; end
    else                               begin $display("FAIL: update right_out=0x%06X", right_out); fail_count=fail_count+1; end

    $display("=== RESULT: %0d PASS, %0d FAIL ===", pass_count, fail_count);
    #100; $finish;
end

// Watchdog
initial begin
    #(BCLK_PERIOD * 64 * 500);
    $display("TIMEOUT"); $finish;
end

endmodule
