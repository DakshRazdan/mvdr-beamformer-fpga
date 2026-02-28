// ============================================================================
// butterfly.v  —  Single Radix-2 DIT Butterfly Unit
// ----------------------------------------------------------------------------
// The fundamental building block of the FFT.
// One butterfly computes:
//   P = A + W*B
//   Q = A - W*B
//
// Where W = twiddle factor = e^(-j*2*pi*k/N)
// Represented as fixed-point: Wr = cos, Wi = -sin (Q1.15 format)
//
// DATA FORMAT: Q1.15 fixed point
//   16-bit signed: 1 sign bit + 15 fractional bits
//   Range: -1.0 to +0.99997
//   1.0 = 32767, -1.0 = -32768
//
// PIPELINE: 3 clock cycles (registered multiply-accumulate)
// ============================================================================

module butterfly #(
    parameter DW = 16   // Data width (16-bit fixed point)
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                valid_in,

    // Input A and B (complex)
    input  wire signed [DW-1:0] ar, ai,   // A real, imag
    input  wire signed [DW-1:0] br, bi,   // B real, imag

    // Twiddle factor W (complex, Q1.15)
    input  wire signed [DW-1:0] wr, wi,   // W real, imag

    // Outputs P = A + W*B, Q = A - W*B
    output reg  signed [DW-1:0] pr, pi,   // P real, imag
    output reg  signed [DW-1:0] qr, qi,   // Q real, imag
    output reg                  valid_out
);

// Stage 1: Compute W*B (complex multiply)
// W*B = (wr + j*wi)(br + j*bi)
//     = (wr*br - wi*bi) + j*(wr*bi + wi*br)
reg signed [2*DW-1:0] wrbr, wibi, wrbi, wibr;
reg valid_s1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wrbr <= 0; wibi <= 0;
        wrbi <= 0; wibr <= 0;
        valid_s1 <= 0;
    end else begin
        wrbr     <= wr * br;
        wibi     <= wi * bi;
        wrbi     <= wr * bi;
        wibr     <= wi * br;
        valid_s1 <= valid_in;
    end
end

// Stage 2: Add/subtract to get W*B, then compute P and Q
reg signed [DW-1:0] wbr, wbi_out;  // W*B real and imag
reg valid_s2;
reg signed [DW-1:0] ar_r, ai_r;    // Delayed A

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wbr <= 0; wbi_out <= 0;
        valid_s2 <= 0;
        ar_r <= 0; ai_r <= 0;
    end else begin
        // Take bits [30:15] — Q1.15 * Q1.15 = Q2.30, we want Q1.15
        wbr      <= (wrbr - wibi) >>> 15;
        wbi_out  <= (wrbi + wibr) >>> 15;
        valid_s2 <= valid_s1;
        // Delay A by one cycle to match pipeline
        ar_r <= ar;
        ai_r <= ai;
    end
end

// Stage 3: P = A + WB, Q = A - WB
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pr <= 0; pi <= 0;
        qr <= 0; qi <= 0;
        valid_out <= 0;
    end else begin
        pr        <= ar_r + wbr;
        pi        <= ai_r + wbi_out;
        qr        <= ar_r - wbr;
        qi        <= ai_r - wbi_out;
        valid_out <= valid_s2;
    end
end

endmodule
