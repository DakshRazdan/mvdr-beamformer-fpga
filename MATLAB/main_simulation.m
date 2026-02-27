%% =========================================================================
%  MVDR BEAMFORMER - MAIN SIMULATION
%  Cognitive-Assistive Hearing Device | Phase 1: Floating Point Validation
%
%  Run this file. Everything else is called from here.
%  Read every comment. This is your understanding, not just your code.
% =========================================================================
clear; clc; close all;

%% -------------------------------------------------------------------------
%  STEP 0: SYSTEM PARAMETERS
%  These values are not arbitrary. Each one has a hardware consequence.
% --------------------------------------------------------------------------
params.fs         = 16000;    % Sample rate (Hz). Matches INMP441 target output
                              % after CIC decimation. Speech band is 300-3400Hz,
                              % so 16kHz gives comfortable margin (Nyquist = 8kHz).

params.N_fft      = 256;      % FFT size. At 16kHz -> frame = 16ms. 
                              % Long enough for frequency resolution (62.5Hz/bin),
                              % short enough for <10ms latency after overlap.

params.overlap    = 0.5;      % 50% overlap (overlap-save method).
params.hop        = params.N_fft * (1 - params.overlap);  % = 128 samples

params.N_mics     = 4;        % Linear array. 4 mics = 3 degrees of freedom.
params.d          = 0.02;     % Mic spacing = 2cm. 
                              % Max unambiguous freq = c/(2d) = 343/(2*0.02) = 8575Hz
                              % Safely above our 8kHz Nyquist. No spatial aliasing.
params.c          = 343;      % Speed of sound (m/s) at ~20°C room temp

params.target_angle = 0;      % Desired speaker direction (degrees). 
                              % 0° = broadside (directly in front of array).

params.noise_angles = [60, -45];  % Two simultaneous noise sources (degrees).
                                  % MVDR will null these automatically.

params.alpha      = 0.97;     % Covariance matrix forgetting factor.
                              % 0.97 = ~33 frame memory (~500ms). 
                              % Tune this in sweep below.

params.delta_load = 0.5;     % Diagonal loading factor.
                              % Prevents matrix inversion instability from
                              % cheap mic mismatch. Critical for real hardware.

params.SNR_input_dB = 0;      % Input SNR (dB). 0dB = noise as loud as speech.
                              % Typical cafeteria scenario for ADHD users.

params.duration   = 3.0;      % Simulation duration (seconds)

fprintf('=== MVDR Beamformer Simulation ===\n');
fprintf('Array: %d mics, %.0fcm spacing\n', params.N_mics, params.d*100);
fprintf('Target: %d deg | Noise: %s deg\n', params.target_angle, ...
        num2str(params.noise_angles));
fprintf('Input SNR: %d dB | alpha: %.3f | delta: %.0e\n\n', ...
        params.SNR_input_dB, params.alpha, params.delta_load);

%% -------------------------------------------------------------------------
%  STEP 1: GENERATE SYNTHETIC MULTICHANNEL AUDIO
%  We use synthetic signals first because:
%  (a) we know the ground truth exactly
%  (b) we can measure SNR precisely
%  (c) no dependency on external audio files
% --------------------------------------------------------------------------
fprintf('Generating multichannel audio...\n');
[mic_signals, speech_clean, params] = generate_array_signals(params);
% mic_signals: [N_samples x N_mics] — what each mic captures
% speech_clean: [N_samples x 1]     — ground truth, for evaluation only

%% -------------------------------------------------------------------------
%  STEP 2: RUN MVDR BEAMFORMER
%  Core algorithm. Frequency-domain, per-bin weight computation.
% --------------------------------------------------------------------------
fprintf('Running MVDR beamformer...\n');
[output_mvdr, weights_log] = mvdr_beamformer(mic_signals, params);

%% -------------------------------------------------------------------------
%  STEP 3: RUN DELAY-AND-SUM FOR COMPARISON
%  This is what your original report proposed. We compare directly.
%  You need this to show why MVDR is better — not just claim it.
% --------------------------------------------------------------------------
fprintf('Running Delay-and-Sum for comparison...\n');
output_das = delayandsum_beamformer(mic_signals, params);

%% -------------------------------------------------------------------------
%  STEP 4: EVALUATE PERFORMANCE
%  Quantitative metrics. These go directly into your report.
% --------------------------------------------------------------------------
fprintf('\nEvaluating performance...\n');
evaluate_performance(speech_clean, mic_signals(:,1), output_mvdr, output_das, params);

%% -------------------------------------------------------------------------
%  STEP 5: PARAMETER SWEEPS
%  This is what separates analysis from just running code.
%  You are mapping the design space — required for fixed-point decisions later.
% --------------------------------------------------------------------------
fprintf('\nRunning parameter sweeps...\n');
run_parameter_sweeps(mic_signals, speech_clean, params);

fprintf('\n=== Simulation Complete ===\n');
