// ============================================================================
// i2s_rx.v  ?  I2S Receiver for INMP441 MEMS Microphones (Verilog-2001)
// ----------------------------------------------------------------------------
// Captures 24-bit stereo audio from a single I2S data line.
// Two instances = 4 microphones (2 per instance, Left + Right channel).
//
// APPROACH: State machine with bit counter.
//   Rather than latching shift register on WS transition (fragile with
//   synchronizer delays), we detect WS edge, wait 1 BCLK, then count
//   exactly DATA_WIDTH rising BCLK edges and latch at the end.
//   This is robust regardless of synchronizer pipeline depth.
// ============================================================================

module i2s_rx #(
    parameter DATA_WIDTH = 24
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  bclk,
    input  wire                  ws,
    input  wire                  sdata,
    output reg  [DATA_WIDTH-1:0] left_out,
    output reg  [DATA_WIDTH-1:0] right_out,
    output reg                   valid
);

// --- 3-stage synchronizers (plain Verilog-2001, no '0 syntax) ---
reg bclk_r1, bclk_r2, bclk_r3;
reg ws_r1,   ws_r2,   ws_r3;
reg sd_r1,   sd_r2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bclk_r1 <= 1'b0; bclk_r2 <= 1'b0; bclk_r3 <= 1'b0;
        ws_r1   <= 1'b0; ws_r2   <= 1'b0; ws_r3   <= 1'b0;
        sd_r1   <= 1'b0; sd_r2   <= 1'b0;
    end else begin
        bclk_r1 <= bclk; bclk_r2 <= bclk_r1; bclk_r3 <= bclk_r2;
        ws_r1   <= ws;   ws_r2   <= ws_r1;   ws_r3   <= ws_r2;
        sd_r1   <= sdata; sd_r2  <= sd_r1;
    end
end

wire bclk_rise = bclk_r2 & ~bclk_r3;
wire ws_fall   = ~ws_r2  &  ws_r3;   // L?R: WS goes high (left done)
wire ws_rise   =  ws_r2  & ~ws_r3;   // R?L: WS goes low  (right done... next frame)

// --- State machine ---
// State: IDLE, WAIT_LEFT, CAPTURE_LEFT, WAIT_RIGHT, CAPTURE_RIGHT
localparam S_IDLE         = 3'd0;
localparam S_WAIT_LEFT    = 3'd1;
localparam S_CAP_LEFT     = 3'd2;
localparam S_WAIT_RIGHT   = 3'd3;
localparam S_CAP_RIGHT    = 3'd4;

reg [2:0]            state;
reg [4:0]            bit_cnt;      // Counts up to DATA_WIDTH (max 24)
reg [DATA_WIDTH-1:0] shift;
reg                  wait_skip;   // Used to skip 2nd dead BCLK in WAIT states

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state     <= S_IDLE;
        bit_cnt   <= 5'd0;
        shift     <= {DATA_WIDTH{1'b0}};
        left_out  <= {DATA_WIDTH{1'b0}};
        right_out <= {DATA_WIDTH{1'b0}};
        valid     <= 1'b0;
        wait_skip <= 1'b0;
    end else begin
        valid <= 1'b0;  // Default: valid is a single-cycle pulse

        case (state)
            // Wait for the WS falling edge that signals left channel start
            S_IDLE: begin
                if (ws_fall) begin
                    state   <= S_WAIT_LEFT;
                    bit_cnt <= 5'd0;
                end
            end

            // I2S spec: WS changes on falling BCLK, first data bit appears on the
            // NEXT falling BCLK, valid on the rising BCLK after that.
            // That means there are 2 dead rising BCLK edges after WS transition.
            // We count both with wait_skip before starting capture.
            S_WAIT_LEFT: begin
                if (bclk_rise) begin
                    if (!wait_skip) begin
                        wait_skip <= 1'b1;  // First dead rising edge ? skip
                    end else begin
                        wait_skip <= 1'b0;
                        state     <= S_CAP_LEFT;
                        bit_cnt   <= 5'd0;
                        shift     <= {DATA_WIDTH{1'b0}};
                    end
                end
            end

            // Capture DATA_WIDTH bits MSB-first on BCLK rising edges
            S_CAP_LEFT: begin
                if (bclk_rise) begin
                    shift   <= {shift[DATA_WIDTH-2:0], sd_r2};
                    bit_cnt <= bit_cnt + 5'd1;
                    if (bit_cnt == DATA_WIDTH - 1) begin
                        // All bits received ? latch and wait for right channel
                        left_out <= {shift[DATA_WIDTH-2:0], sd_r2};
                        state    <= S_WAIT_RIGHT;
                        bit_cnt  <= 5'd0;
                    end
                end
            end

            // Wait for WS to go high, then skip 2 dead rising BCLKs
            S_WAIT_RIGHT: begin
                if (ws_r2 && bclk_rise) begin
                    if (!wait_skip) begin
                        wait_skip <= 1'b1;
                    end else begin
                        wait_skip <= 1'b0;
                        state     <= S_CAP_RIGHT;
                        bit_cnt   <= 5'd0;
                        shift     <= {DATA_WIDTH{1'b0}};
                    end
                end
            end

            // Capture right channel
            S_CAP_RIGHT: begin
                if (bclk_rise) begin
                    shift   <= {shift[DATA_WIDTH-2:0], sd_r2};
                    bit_cnt <= bit_cnt + 5'd1;
                    if (bit_cnt == DATA_WIDTH - 1) begin
                        right_out <= {shift[DATA_WIDTH-2:0], sd_r2};
                        valid     <= 1'b1;   // Both channels complete
                        state     <= S_IDLE;
                        bit_cnt   <= 5'd0;
                    end
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
