// ============================================================================
// i2s_master_clk.v  —  I2S Master Clock Generator
// ----------------------------------------------------------------------------
// Generates BCLK and WS (LRCLK) for all 4 INMP441 microphones.
// All mics share the same BCLK and WS — this is what keeps them
// phase-synchronized, which is the fundamental requirement for beamforming.
// Any phase difference between mics introduced HERE would be a systematic
// error that cannot be corrected by the beamformer.
//
// CLOCK MATH:
//   Target sample rate:    16,000 Hz
//   Bits per frame:        64 (32 per channel, I2S standard)
//   Required BCLK:         16,000 × 64 = 1,024,000 Hz (1.024 MHz)
//
//   With 100 MHz system clock:
//   BCLK divider = 100,000,000 / 1,024,000 / 2 ≈ 48.8 → use 49
//   Actual BCLK  = 100,000,000 / (2 × 49) ≈ 1,020,408 Hz
//   Actual Fs    = 1,020,408 / 64 ≈ 15,944 Hz  (close enough for audio)
//
//   Alternatively use a PLL to generate exact 16.384 MHz MCLK and divide:
//   MCLK = 16.384 MHz, BCLK = MCLK/16 = 1.024 MHz, WS = BCLK/64 = 16 kHz
//   This is preferred in a real PCB design — use PLL for exact Fs.
//
// FOR BASYS 3 / ARTY A7 (100 MHz onboard clock):
//   Use Xilinx MMCM/PLL in Vivado IP catalog to generate exact MCLK.
//   Connect MCLK output to this module's clk_in with appropriate dividers.
// ============================================================================

module i2s_master_clk #(
    parameter SYS_CLK_FREQ = 100_000_000,  // System clock frequency (Hz)
    parameter BCLK_FREQ    = 1_024_000,    // Target BCLK frequency (Hz)
    parameter BITS_PER_CH  = 32            // Bits per channel (32 = standard)
)(
    input  wire clk,      // System clock
    input  wire rst_n,

    output reg  bclk,     // Bit clock → all 4 INMP441 BCLK pins
    output reg  ws,       // Word select / LRCLK → all 4 INMP441 WS pins
    output wire  mclk      // Master clock (optional, some boards need it)
);

// BCLK divider
localparam BCLK_DIV = SYS_CLK_FREQ / BCLK_FREQ / 2;  // Half period counter

// WS divider: WS toggles every BITS_PER_CH BCLK cycles
localparam WS_DIV = BITS_PER_CH;

reg [$clog2(BCLK_DIV)-1:0]  bclk_cnt;
reg [$clog2(WS_DIV)-1:0]    ws_cnt;
reg                          bclk_rise_pulse;

// BCLK generation
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bclk      <= 1'b0;
        bclk_cnt  <= '0;
        bclk_rise_pulse <= 1'b0;
    end else begin
        bclk_rise_pulse <= 1'b0;
        if (bclk_cnt == BCLK_DIV - 1) begin
            bclk_cnt <= '0;
            bclk     <= ~bclk;
            if (!bclk)  // About to go HIGH
                bclk_rise_pulse <= 1'b1;
        end else begin
            bclk_cnt <= bclk_cnt + 1'b1;
        end
    end
end

// WS generation: toggle every BITS_PER_CH BCLK rising edges
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ws     <= 1'b0;
        ws_cnt <= '0;
    end else if (bclk_rise_pulse) begin
        if (ws_cnt == WS_DIV - 1) begin
            ws_cnt <= '0;
            ws     <= ~ws;
        end else begin
            ws_cnt <= ws_cnt + 1'b1;
        end
    end
end

// MCLK = 4× BCLK (some codecs need this, INMP441 does not)
// Connect to 4× faster PLL output in real design
assign mclk = clk;  // Placeholder — override with PLL output

endmodule
