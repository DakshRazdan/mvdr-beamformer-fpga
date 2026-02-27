function [output, weights_log] = mvdr_beamformer(mic_signals, params)
%% MVDR_BEAMFORMER  —  Frequency-Domain MVDR with Preamble-Based Noise Estimation
%
%  APPROACH: Two-phase operation (mirrors real hearing aid hardware exactly)
%
%  PHASE 1 — CALIBRATION (first 0.5s, noise-only preamble):
%    Accumulate spatial covariance matrix R from pure noise frames.
%    R converges to true noise covariance with no speech contamination.
%    On FPGA: this is the power-on startup calibration mode.
%
%  PHASE 2 — BEAMFORMING (full signal):
%    R is FROZEN at its calibrated value. Weights computed ONCE from frozen R.
%    Applied to every incoming frame. No runtime weight updates.
%    On FPGA: fixed weights stored in registers, applied every frame.
%
%  WHY THIS BEATS VAD-GATED APPROACH AT 0dB SNR:
%    At 0dB SNR speech and noise have equal power — energy VAD cannot separate
%    them reliably. Preamble approach sidesteps this entirely. R estimated from
%    guaranteed clean noise gives optimal MVDR weights with zero cancellation.

[N_samples, N_mics] = size(mic_signals);
N_fft  = params.N_fft;
hop    = round(params.hop);
delta  = params.delta_load;
alpha  = params.alpha;
N_bins = N_fft/2 + 1;

%% --- Precompute steering vectors (vectorized, no loop)
freqs   = (0:N_bins-1)' * params.fs / N_fft;  % [N_bins x 1]
theta   = params.target_angle * pi/180;
tau     = params.d * cos(theta) / params.c;
mic_idx = 0:(N_mics-1);                        % [1 x N_mics]
D       = exp(-1j * 2*pi * freqs * tau * mic_idx);  % [N_bins x N_mics]

%% --- Hann window (periodic form for correct OLA)
win = hann(N_fft, 'periodic');

%% =========================================================================
%  PHASE 1: Covariance estimation — use ONLY the noise preamble frames
% ==========================================================================
N_preamble = round(0.5 * params.fs);
R = repmat(delta * eye(N_mics), [1, 1, N_bins]);
n_cal_frames = 0;

for frame_start = 1:hop:(N_preamble - N_fft + 1)
    frame_end = frame_start + N_fft - 1;
    frame = mic_signals(frame_start:frame_end, :) .* win;
    X = fft(frame, N_fft);
    X = X(1:N_bins, :);  % [N_bins x N_mics]
    
    for k = 1:N_bins
        x_k      = X(k,:)';
        R(:,:,k) = alpha * R(:,:,k) + x_k * x_k';
    end
    n_cal_frames = n_cal_frames + 1;
end
fprintf('  MVDR Phase 1: %d calibration frames processed\n', n_cal_frames);

%% --- Compute MVDR weights ONCE from frozen R
%  w(k) = R^{-1}(k)*d(k) / (d(k)^H * R^{-1}(k) * d(k))
%  These weights are computed once and applied to every subsequent frame.
%  On FPGA this is a one-time startup computation stored in weight registers.
W = zeros(N_bins, N_mics);  % [N_bins x N_mics] complex
for k = 1:N_bins
    R_loaded  = R(:,:,k) + delta * eye(N_mics);
    d_k       = D(k,:)';
    Rinv_d    = R_loaded \ d_k;
    denom     = d_k' * Rinv_d;
    if abs(denom) > 1e-12
        W(k,:) = (Rinv_d / denom)';
    else
        W(k,:) = d_k' / N_mics;  % Fallback to DAS if matrix singular
    end
end
fprintf('  MVDR Phase 1: Weights computed for all %d frequency bins\n', N_bins);

%% --- Log weights
weights_log.weights        = zeros(N_bins, N_mics, 1);
weights_log.weights(:,:,1) = W;
weights_log.frame_idx      = [1];

%% =========================================================================
%  PHASE 2: Apply fixed weights to entire signal
% ==========================================================================
output      = zeros(N_samples, 1);
frame_count = 0;

for frame_start = 1:hop:(N_samples - N_fft + 1)
    frame_end   = frame_start + N_fft - 1;
    frame_count = frame_count + 1;
    
    % Windowed FFT of all mics
    frame = mic_signals(frame_start:frame_end, :) .* win;
    X     = fft(frame, N_fft);
    X     = X(1:N_bins, :);  % [N_bins x N_mics]
    
    % Apply weights: Y(k) = w(k)^H * x(k)
    % Vectorized across all bins simultaneously
    Y = sum(conj(W) .* X, 2);  % [N_bins x 1]
    
    % Reconstruct full spectrum and IFFT
    Y_full  = [Y; conj(flipud(Y(2:end-1)))];
    y_frame = real(ifft(Y_full));
    
    % Overlap-add (no second window — periodic Hann OLA sums to 1.0)
    output(frame_start:frame_end) = output(frame_start:frame_end) + y_frame;
end

output = output(1:N_samples);
fprintf('  MVDR Phase 2: %d beamforming frames applied\n', frame_count);
end
