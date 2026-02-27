// ============================================================================
// cic_decimator.v  —  CIC (Cascaded Integrator-Comb) Decimation Filter
// ----------------------------------------------------------------------------
// Converts high sample rate I2S audio (e.g. 3.072 MHz) down to 16 kHz
// for the beamformer. No multipliers — pure adders and registers.
//
// CIC THEORY (read this — it's on your FPGA, not a black box):
//   A CIC filter has two sections:
//   1. INTEGRATOR stage (runs at HIGH rate): y[n] = y[n-1] + x[n]
//      This is just a running accumulator. M integrators in series.
//   2. COMB stage (runs at LOW rate, after decimation): y[n] = x[n] - x[n-M]
//      This is a differentiator with M-sample delay.
//
//   For M=3 stages, decimation ratio R=192:
//     Input:  3.072 MHz (from I2S BCLK / 1)  → 3,072,000 Hz
//     Output: 16,000 Hz = 3,072,000 / 192
//
//   Bit growth: CIC adds log2(R^M) bits of headroom = log2(192^3) ≈ 22 bits
//   So input is 24-bit, internal registers need 24+22 = 46 bits minimum.
//   We use 48-bit internal width (power of 2, matches DSP slice widths).
//
// FREQUENCY RESPONSE:
//   CIC has a sinc^M rolloff in the passband. This is acceptable for audio
//   since the rolloff at 8kHz (Nyquist for 16kHz output) is only ~3dB.
//   A short FIR compensation filter after CIC corrects the passband droop.
//   (FIR compensator is a separate module — cic_comp_fir.v)
//
// FPGA RESOURCE USAGE (estimated for 4 channels):
//   Registers:  ~4 × (M integrators + M combs) × 48 bits ≈ 1152 FFs
//   DSP blocks: 0 (no multipliers)
//   Block RAMs:  0
// ============================================================================

module cic_decimator #(
    parameter INPUT_WIDTH  = 24,    // Input sample width (from I2S RX)
    parameter OUTPUT_WIDTH = 16,    // Output sample width (to beamformer)
    parameter M_STAGES     = 3,     // Number of integrator/comb stages
    parameter R_DECIMATE   = 192,   // Decimation ratio (3.072MHz / 192 = 16kHz)
    parameter INTERNAL_W   = 48    // Internal accumulator width (see bit growth note)
)(
    input  wire                    clk,        // High-rate clock (system clock)
    input  wire                    rst_n,
    input  wire [INPUT_WIDTH-1:0]  x_in,       // Input sample (high rate)
    input  wire                    x_valid,    // Input valid strobe (high rate)
    output reg  [OUTPUT_WIDTH-1:0] y_out,      // Output sample (low rate = 16kHz)
    output reg                     y_valid     // Output valid strobe (16kHz)
);

// ============================================================================
// INTEGRATOR SECTION (runs at high input rate)
// Each stage: acc[k] = acc[k] + acc[k-1]   (or input for stage 0)
// ============================================================================
reg signed [INTERNAL_W-1:0] int_stage [0:M_STAGES-1];

integer s;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (s = 0; s < M_STAGES; s = s+1)
            int_stage[s] <= 0;  
    end else if (x_valid) begin
        // Stage 0: integrate input
        int_stage[0] <= int_stage[0] + {{(INTERNAL_W-INPUT_WIDTH){x_in[INPUT_WIDTH-1]}}, x_in};
        // Stages 1..M-1: integrate previous stage output
        for (s = 1; s < M_STAGES; s = s+1)
            int_stage[s] <= int_stage[s] + int_stage[s-1];
    end
end

// ============================================================================
// DECIMATION COUNTER
// Generates a strobe every R_DECIMATE input samples
// ============================================================================
reg [$clog2(R_DECIMATE)-1:0] decim_count;
reg                           decim_strobe;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        decim_count  <= 0;
        decim_strobe <= 1'b0;
    end else begin
        decim_strobe <= 1'b0;
        if (x_valid) begin
            if (decim_count == R_DECIMATE - 1) begin
                decim_count  <= 0;
                decim_strobe <= 1'b1;  // Fire once every R samples
            end else begin
                decim_count <= decim_count + 1'b1;
            end
        end
    end
end

// ============================================================================
// COMB SECTION (runs at low output rate, triggered by decim_strobe)
// Each stage: diff[k] = acc[k] - acc[k-1 delayed by 1 output sample]
// ============================================================================
reg signed [INTERNAL_W-1:0] comb_in  [0:M_STAGES-1];
reg signed [INTERNAL_W-1:0] comb_dly [0:M_STAGES-1];  // 1-sample delay
reg signed [INTERNAL_W-1:0] comb_out [0:M_STAGES-1];

integer c;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (c = 0; c < M_STAGES; c = c+1) begin
            comb_in[c]  <= 0;
            comb_dly[c] <= 0;
            comb_out[c] <= 0;
        end
        y_out   <= 0;
        y_valid <= 1'b0;
    end else begin
        y_valid <= 1'b0;

        if (decim_strobe) begin
            // Stage 0 input: last integrator output
            comb_in[0]  <= int_stage[M_STAGES-1];
            comb_dly[0] <= comb_in[0];
            comb_out[0] <= comb_in[0] - comb_dly[0];

            // Stages 1..M-1
            for (c = 1; c < M_STAGES; c = c+1) begin
                comb_in[c]  <= comb_out[c-1];
                comb_dly[c] <= comb_in[c];
                comb_out[c] <= comb_in[c] - comb_dly[c];
            end

            // Output: truncate to OUTPUT_WIDTH
            // We discard the lower bits (quantization noise) and sign-extend
            // The gain of a CIC is R^M = 192^3 ≈ 7e6, so the top OUTPUT_WIDTH
            // bits represent the normalized output.
            // Bit [INTERNAL_W-1] is sign. We take bits [INTERNAL_W-2 : INTERNAL_W-OUTPUT_WIDTH-1]
            y_out   <= comb_out[M_STAGES-1][INTERNAL_W-2 : INTERNAL_W-OUTPUT_WIDTH-1];
            y_valid <= 1'b1;
        end
    end
end

endmodule
