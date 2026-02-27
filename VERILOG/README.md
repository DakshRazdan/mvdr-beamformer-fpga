# MVDR Beamformer — Cognitive-Assistive Hearing Device

**4-microphone Minimum Variance Distortionless Response (MVDR) spatial filter for ADHD users with sensorineural hearing loss**

MATLAB simulation validated → FPGA Verilog implementation in progress

---

## Project Overview

Standard hearing aids use 2-microphone delay-and-sum beamforming (+3–4 dB SNR improvement). This project implements frequency-domain MVDR beamforming with a 4-mic INMP441 I2S MEMS array, achieving **+14 dB SNR improvement** — more than 3× better than delay-and-sum.

The system targets the Lattice iCE40UP5K FPGA for eventual miniaturization into a wearable form factor.

---

## Key Results (MATLAB Simulation)

| Configuration | SNR Improvement |
|---|---|
| Raw mic (baseline) | 0.0 dB |
| 2-mic Delay-and-Sum | +2.2 dB |
| 4-mic Delay-and-Sum | +3.9 dB |
| 2-mic MVDR | +6.0 dB |
| **4-mic MVDR** | **+14.1 dB** |

MVDR beam pattern: deep nulls (~55 dB) at noise source angles (-45° and 60°), 0 dB at target (0°).

---

## System Architecture

```
4× INMP441 I2S MEMS Mics
        ↓
I2S RX Controller (i2s_rx.v)        ← VERIFIED ✅
        ↓
CIC Decimation Filter (cic_decimator.v)  ← VERIFIED ✅
   3.072 MHz → 16 kHz, no multipliers
        ↓
256-point FFT Engine (fft_r2dit.v)   ← IN PROGRESS
        ↓
Covariance Matrix Estimator          ← PENDING
   R(k) = α·R(k) + x(k)·x(k)^H
        ↓
MVDR Weight Computation              ← PENDING
   w(k) = R^{-1}·d / (d^H·R^{-1}·d)
        ↓
Beamformer Apply + IFFT              ← PENDING
        ↓
I2S TX → PAM8403 → Bone Conduction
```

---

## Design Parameters (from MATLAB sweep)

| Parameter | Value | Notes |
|---|---|---|
| Sample rate | 16 kHz | Post-CIC decimation |
| FFT size | 256 points | 16 ms frame, 62.5 Hz/bin |
| Overlap | 50% (128-sample hop) | Hann window (periodic) |
| Mic spacing | 2.0 cm | Max unambiguous freq: 8.5 kHz |
| Forgetting factor α | 0.95 | From parameter sweep |
| Diagonal loading δ | 0.5 | Robustness to mic mismatch |
| Fixed-point width | 16-bit | Safe margin above 1dB degradation |
| Noise sources | -45°, +60° | Dual interference |

---

## Repository Structure

```
mvdr-beamformer-fpga/
│
├── matlab/                   # MATLAB simulation (Phase 1 — COMPLETE)
│   ├── main_simulation.m     # Entry point — run this
│   ├── generate_array_signals.m
│   ├── mvdr_beamformer.m     # Core MVDR algorithm
│   ├── delayandsum_beamformer.m
│   ├── evaluate_performance.m
│   └── run_parameter_sweeps.m
│
├── verilog/                  # FPGA implementation (Phase 2 — IN PROGRESS)
│   ├── i2s_master_clk.v      # BCLK + WS generation
│   ├── i2s_rx.v              # 4-channel I2S capture ✅
│   ├── i2s_rx_tb.v           # Testbench — 4/4 PASS ✅
│   ├── cic_decimator.v       # 3-stage CIC, R=192 ✅
│   ├── cic_decimator_tb.v    # Testbench — 2/2 PASS ✅
│   └── fft_r2dit.v           # Radix-2 DIT FFT (in progress)
│
└── docs/                     # Reports and figures
```

---

## How to Simulate (MATLAB)

```matlab
cd matlab/
main_simulation.m   % Runs full pipeline + parameter sweeps
```

Requirements: MATLAB R2020+ with Signal Processing Toolbox

## How to Simulate (Verilog)

```powershell
# CIC Decimator
iverilog -o a.out cic_decimator_tb.v && vvp a.out

# I2S Receiver
iverilog -o a.out i2s_rx_tb.v && vvp a.out

# View waveforms
gtkwave cic_decimator.vcd
```

Requirements: [iverilog](http://iverilog.icarus.com/) + [GTKWave](http://gtkwave.sourceforge.net/)

---

## Hardware (Planned)

| Component | Part | Cost |
|---|---|---|
| 4× MEMS microphone | INMP441 | ₹600 |
| I2S DAC | PCM5102 | ₹250 |
| I2S Amplifier | MAX98357A | ₹200 |
| Bone conduction transducer | — | ₹300 |
| FPGA (prototype) | Arty A7-35T | — |
| FPGA (miniaturized) | Lattice iCE40UP5K | — |

---

## Novel Contributions

1. 4-mic MVDR with VAD-gated covariance estimation (not 2-mic delay-and-sum)
2. On-FPGA CIC decimation of I2S MEMS PDM (no analog front-end)
3. Per-bin frequency-domain weight update (frequency-invariant beam pattern)
4. Diagonal-loaded matrix inversion (robust to real-world mic mismatch)
5. Direct iCE40UP5K miniaturization path (wearable product trajectory)

---

## Authors

DakshRazdan — DLD Project, 2026

