// ============================================================================
// twiddle_rom.v  —  Twiddle Factor ROM for 256-point FFT
// ----------------------------------------------------------------------------
// Stores W_N^k = e^(-j*2*pi*k/N) for k = 0..N/2-1 (128 entries)
//
// FORMAT: Q1.15 fixed point
//   wr[k] = round(cos(2*pi*k/256) * 32767)
//   wi[k] = round(-sin(2*pi*k/256) * 32767)
//
// In synthesis: This becomes a Block RAM or distributed ROM.
// In simulation: Initialized via $cos/$sin in initial block.
//
// USAGE:
//   The FFT stage s uses twiddle W_N^(k * 2^(S-s-1))
//   where S = log2(N) = 8 for N=256
//   Address = (butterfly_index * stride) mod 128
// ============================================================================

module twiddle_rom #(
    parameter N     = 256,   // FFT size
    parameter DW    = 16     // Data width
)(
    input  wire                    clk,
    input  wire [$clog2(N/2)-1:0]  addr,   // 0..127
    output reg  signed [DW-1:0]    wr,     // cos term
    output reg  signed [DW-1:0]    wi      // -sin term
);

localparam HALF_N = N/2;  // 128 entries

reg signed [DW-1:0] rom_r [0:HALF_N-1];
reg signed [DW-1:0] rom_i [0:HALF_N-1];

// Initialize ROM with twiddle factors
integer k;
real pi_val;
initial begin
    pi_val = 3.14159265358979;
    for (k = 0; k < HALF_N; k = k + 1) begin
        rom_r[k] = $rtoi($cos(2.0 * pi_val * k / N) * 32767.0);
        rom_i[k] = $rtoi(-$sin(2.0 * pi_val * k / N) * 32767.0);
    end
end

// Registered read (1-cycle latency — matches butterfly pipeline)
always @(posedge clk) begin
    wr <= rom_r[addr];
    wi <= rom_i[addr];
end

endmodule
