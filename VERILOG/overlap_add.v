// overlap_add.v -- 50% Overlap-Add Reconstruction
`timescale 1ns/1ps

module overlap_add #(
    parameter N   = 256,
    parameter HOP = 128,
    parameter DW  = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire signed [DW-1:0] x_re,
    input  wire        x_valid,
    input  wire        x_last,
    output reg  signed [DW-1:0] pcm_out,
    output reg         pcm_valid,
    output reg         pcm_last
);

// Hann window ROM Q1.15
reg signed [DW-1:0] hann [0:N-1];
integer hi;
initial begin
    for (hi = 0; hi < N; hi = hi + 1)
        hann[hi] = $rtoi((0.5 - 0.5*$cos(2.0*3.14159265358979*hi/N)) * 32767.0);
end

// Overlap buffer: windowed tail of previous frame
reg signed [DW-1:0] overlap_buf [0:HOP-1];
integer oi;
initial begin
    for (oi = 0; oi < HOP; oi = oi + 1) overlap_buf[oi] = 0;
end

// Frame buffer: windowed current frame
reg signed [DW-1:0] frame_buf [0:N-1];
integer fi;
initial begin
    for (fi = 0; fi < N; fi = fi + 1) frame_buf[fi] = 0;
end

localparam ST_IDLE   = 2'd0;
localparam ST_WINDOW = 2'd1;
localparam ST_OUTPUT = 2'd2;

reg [1:0]  state;
reg [7:0]  cnt;     // 0..255 window, 0..127 output

// ============================================================================
// COMBINATIONAL WIRES for array reads — avoids iverilog blocking-assign issue
// 1D arrays with variable index in wire assign are safe in iverilog
// ============================================================================
wire signed [DW-1:0] fb_val  = frame_buf[cnt];          // frame_buf[cnt]
wire signed [DW-1:0] ob_val  = overlap_buf[cnt];         // overlap_buf[cnt]
wire signed [DW-1:0] fb_tail = frame_buf[cnt + HOP];     // frame_buf[cnt+128]
wire signed [DW-1:0] hw_val  = hann[cnt];                // hann[cnt]

// Overlap-add sum — 17-bit to catch overflow before saturation
wire signed [16:0] oa_sum = $signed(fb_val) + $signed(ob_val);

// Saturated output
wire signed [DW-1:0] oa_sat =
    (oa_sum > 17'sd32767)  ? 16'sd32767  :
    (oa_sum < -17'sd32768) ? -16'sd32768 :
    oa_sum[DW-1:0];

// Windowed input: x_re * hann[cnt] >> 15
wire signed [2*DW-1:0] win_full = $signed(x_re) * $signed(hw_val);
wire signed [DW-1:0]   win_val  = win_full >>> 15;

// ============================================================================
// FSM
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state     <= ST_IDLE;
        cnt       <= 0;
        pcm_valid <= 0;
        pcm_last  <= 0;
        pcm_out   <= 0;
    end else begin
        pcm_valid <= 0;
        pcm_last  <= 0;

        case (state)

        ST_IDLE: begin
            if (x_valid) begin
                frame_buf[cnt] <= win_val;
                if (cnt == N-1) begin
                    cnt   <= 0;
                    state <= ST_OUTPUT;
                end else
                    cnt <= cnt + 1;
            end
        end

        ST_WINDOW: begin  // alias — ST_IDLE handles both via cnt
            state <= ST_IDLE;
        end

        ST_OUTPUT: begin
            pcm_out   <= oa_sat;
            pcm_valid <= 1;
            pcm_last  <= (cnt == HOP-1);
            overlap_buf[cnt] <= fb_tail;  // save second half as new overlap
            if (cnt == HOP-1) begin
                cnt   <= 0;
                state <= ST_IDLE;
            end else
                cnt <= cnt + 1;
        end

        default: state <= ST_IDLE;
        endcase
    end
end

endmodule