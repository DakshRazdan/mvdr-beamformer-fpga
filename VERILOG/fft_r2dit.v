// ============================================================================
// fft_r2dit.v  —  256-point Radix-2 DIT FFT
// ----------------------------------------------------------------------------
// Decimation-In-Time Cooley-Tukey FFT
//
// ARCHITECTURE: Iterative, single-butterfly, 8-stage sequential
//   - N = 256 points, S = log2(256) = 8 stages
//   - One butterfly computed per clock cycle
//   - Total compute cycles: 8 stages × 128 butterflies = 1024 cycles
//   - At 100MHz: 1024 × 10ns = 10.24us << 8ms frame period (plenty of margin)
//
// DATA FLOW:
//   ST_IDLE    → wait for x_valid
//   ST_LOAD    → accept 256 samples, store in bit-reversed order
//   ST_COMPUTE → run 8 FFT stages in-place on RAM
//   ST_OUTPUT  → stream 256 complex bins out
//
// MEMORY: 2 arrays of 256 × 16-bit (re and im), synthesizes to BRAM
//
// TWIDDLE FACTORS: ROM of 128 entries, W_N^k = e^{-j2πk/N}
//   Stage s: stride = 128 >> s, tw_idx = offset × stride
// ============================================================================

module fft_r2dit #(
    parameter N    = 256,
    parameter DW   = 16,
    parameter LOGN = 8
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Input: time-domain samples (real only, imag=0)
    input  wire signed [DW-1:0] x_re,
    input  wire                 x_valid,

    // Output: frequency bins (complex)
    output reg  signed [DW-1:0] y_re,
    output reg  signed [DW-1:0] y_im,
    output reg                  y_valid,
    output reg                  y_last    // high on final bin (255)
);

// ============================================================================
// BIT-REVERSE LUT
// ============================================================================
reg [LOGN-1:0] bit_rev [0:N-1];
integer r, b;
initial begin
    for (r = 0; r < N; r = r + 1) begin
        bit_rev[r] = 0;
        for (b = 0; b < LOGN; b = b + 1)
            bit_rev[r][b] = (r >> (LOGN-1-b)) & 1;
    end
end

// ============================================================================
// SAMPLE RAM
// ============================================================================
reg signed [DW-1:0] ram_re [0:N-1];
reg signed [DW-1:0] ram_im [0:N-1];

// ============================================================================
// TWIDDLE ROM
// ============================================================================
reg signed [DW-1:0] tw_re_rom [0:N/2-1];
reg signed [DW-1:0] tw_im_rom [0:N/2-1];
integer tw;
real pi_val;
initial begin
    pi_val = 3.14159265358979;
    for (tw = 0; tw < N/2; tw = tw + 1) begin
        tw_re_rom[tw] = $rtoi($cos(2.0 * pi_val * tw / N) * 32767.0);
        tw_im_rom[tw] = $rtoi(-$sin(2.0 * pi_val * tw / N) * 32767.0);
    end
end

// ============================================================================
// STATE MACHINE
// ============================================================================
localparam ST_IDLE    = 2'd0;
localparam ST_LOAD    = 2'd1;
localparam ST_COMPUTE = 2'd2;
localparam ST_OUTPUT  = 2'd3;

reg [1:0]       state;
reg [LOGN-1:0]  load_cnt;
reg [LOGN-1:0]  stage;
reg [LOGN-1:0]  bfly_cnt;
reg [LOGN-1:0]  out_cnt;

// ============================================================================
// BUTTERFLY INDEX DECODE (combinational)
// ============================================================================
reg  [LOGN-1:0] idx_a, idx_b, tw_idx;
reg  [LOGN-1:0] half_size, group, offset;

always @(*) begin
    half_size = 1 << stage;                      // 2^stage
    group     = bfly_cnt >> stage;               // which group
    offset    = bfly_cnt & (half_size - 1);      // position within group
    idx_a     = (group << (stage+1)) + offset;   // = group * 2*half + offset
    idx_b     = idx_a + half_size;
    tw_idx    = offset * ((N/2) >> stage);       // offset * stride
end

// ============================================================================
// COMBINATIONAL BUTTERFLY
// ============================================================================
wire signed [DW-1:0]     a_re_w = ram_re[idx_a];
wire signed [DW-1:0]     a_im_w = ram_im[idx_a];
wire signed [DW-1:0]     b_re_w = ram_re[idx_b];
wire signed [DW-1:0]     b_im_w = ram_im[idx_b];
wire signed [DW-1:0]     w_re_w = tw_re_rom[tw_idx];
wire signed [DW-1:0]     w_im_w = tw_im_rom[tw_idx];

wire signed [2*DW-1:0]   wb_re_full = (w_re_w * b_re_w) - (w_im_w * b_im_w);
wire signed [2*DW-1:0]   wb_im_full = (w_re_w * b_im_w) + (w_im_w * b_re_w);
wire signed [DW-1:0]     wb_re      = $signed(wb_re_full) >>> 15;
wire signed [DW-1:0]     wb_im      = $signed(wb_im_full) >>> 15;

// Scale by >>1 each stage to prevent overflow across 8 stages (total /256)
// Without scaling: max growth = 2^8 = 256x -> overflows 16-bit
// With scaling: output magnitude = input amplitude (no net growth)
wire signed [DW-1:0]     p_re = (a_re_w + wb_re) >>> 1;
wire signed [DW-1:0]     p_im = (a_im_w + wb_im) >>> 1;
wire signed [DW-1:0]     q_re = (a_re_w - wb_re) >>> 1;
wire signed [DW-1:0]     q_im = (a_im_w - wb_im) >>> 1;

// ============================================================================
// FSM
// ============================================================================
integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state    <= ST_IDLE;
        load_cnt <= 0;
        stage    <= 0;
        bfly_cnt <= 0;
        out_cnt  <= 0;
        y_valid  <= 0;
        y_last   <= 0;
        y_re     <= 0;
        y_im     <= 0;
        for (i = 0; i < N; i = i + 1) begin
            ram_re[i] <= 0;
            ram_im[i] <= 0;
        end
    end else begin
        y_valid <= 0;
        y_last  <= 0;

        case (state)

        // Wait for first sample
        ST_IDLE: begin
            if (x_valid) begin
                ram_re[bit_rev[0]] <= x_re;
                ram_im[bit_rev[0]] <= 0;
                load_cnt <= 1;
                state    <= ST_LOAD;
            end
        end

        // Load 256 samples in bit-reversed order
        ST_LOAD: begin
            if (x_valid) begin
                ram_re[bit_rev[load_cnt]] <= x_re;
                ram_im[bit_rev[load_cnt]] <= 0;
                if (load_cnt == N-1) begin
                    stage    <= 0;
                    bfly_cnt <= 0;
                    state    <= ST_COMPUTE;
                end else begin
                    load_cnt <= load_cnt + 1;
                end
            end
        end

        // Compute: one butterfly per clock
        ST_COMPUTE: begin
            ram_re[idx_a] <= p_re;
            ram_im[idx_a] <= p_im;
            ram_re[idx_b] <= q_re;
            ram_im[idx_b] <= q_im;

            if (bfly_cnt == N/2 - 1) begin
                bfly_cnt <= 0;
                if (stage == LOGN-1) begin
                    out_cnt <= 0;
                    state   <= ST_OUTPUT;
                end else begin
                    stage <= stage + 1;
                end
            end else begin
                bfly_cnt <= bfly_cnt + 1;
            end
        end

        // Stream output bins 0..255
        ST_OUTPUT: begin
            y_re    <= ram_re[out_cnt];
            y_im    <= ram_im[out_cnt];
            y_valid <= 1;
            y_last  <= (out_cnt == N-1);

            if (out_cnt == N-1) begin
                load_cnt <= 0;
                state    <= ST_IDLE;
            end else begin
                out_cnt <= out_cnt + 1;
            end
        end

        endcase
    end
end

endmodule