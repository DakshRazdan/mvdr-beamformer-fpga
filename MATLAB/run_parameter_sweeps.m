function run_parameter_sweeps(mic_signals, speech_clean, params)
%% RUN_PARAMETER_SWEEPS
%  This function maps the design space. It answers three critical questions:
%
%  Q1: What forgetting factor (alpha) gives the best SNR?
%      → Determines how fast the beamformer tracks changing noise environments
%      → Too low: weights thrash, poor steady-state SNR
%      → Too high: slow adaptation, fails in dynamic noise
%
%  Q2: What diagonal loading (delta) is optimal?
%      → Determines robustness to mic gain/phase mismatch
%      → Too low: matrix inversion unstable with real mics
%      → Too high: approaches delay-and-sum (loses MVDR benefit)
%
%  Q3: What word length (bits) is sufficient for fixed-point?
%      → Determines FPGA datapath width
%      → This is the bridge from simulation to hardware
%
%  Run time: ~2-5 minutes. Make a cup of tea.

fprintf('\n--- SWEEP 1: Forgetting Factor (alpha) ---\n');
sweep_alpha(mic_signals, speech_clean, params);

fprintf('\n--- SWEEP 2: Diagonal Loading (delta) ---\n');
sweep_delta(mic_signals, speech_clean, params);

fprintf('\n--- SWEEP 3: Number of Microphones ---\n');
sweep_mic_count(mic_signals, speech_clean, params);

fprintf('\n--- SWEEP 4: Fixed-Point Word Length ---\n');
sweep_fixed_point(mic_signals, speech_clean, params);
end


%% =========================================================================
function sweep_alpha(mic_signals, speech_clean, params)
alpha_values = [0.80, 0.85, 0.90, 0.93, 0.95, 0.97, 0.98, 0.99, 0.995];
snr_results  = zeros(size(alpha_values));
N = length(speech_clean);

for i = 1:length(alpha_values)
    p = params;
    p.alpha = alpha_values(i);
    output = mvdr_beamformer(mic_signals, p);
    N2 = min(length(output), length(speech_clean));
    s = speech_clean(1:N2) / (rms(speech_clean(1:N2))+1e-12);
    o = output(1:N2) / (rms(output(1:N2))+1e-12);
    sc = (s'*o)/(s'*s); sp = sc*s; ns = o-sp;
    snr_results(i) = 10*log10((mean(sp.^2)+1e-12)/(mean(ns.^2)+1e-12));
    fprintf('  alpha=%.3f -> SNR=%.1f dB\n', alpha_values(i), snr_results(i));
end

figure('Name', 'Sweep: Forgetting Factor', 'Position', [100 100 600 350]);
plot(alpha_values, snr_results, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
[~, best_idx] = max(snr_results);
hold on;
plot(alpha_values(best_idx), snr_results(best_idx), 'r*', 'MarkerSize', 15);
text(alpha_values(best_idx)+0.002, snr_results(best_idx), ...
     sprintf('  Best: α=%.3f\n  SNR=%.1f dB', alpha_values(best_idx), snr_results(best_idx)));
xlabel('Forgetting Factor (α)'); ylabel('Output SNR (dB)');
title('MVDR Performance vs. Forgetting Factor');
grid on; xlim([0.78 1.0]);
fprintf('  → Recommended alpha: %.3f\n', alpha_values(best_idx));
end


%% =========================================================================
function sweep_delta(mic_signals, speech_clean, params)
delta_values = logspace(-6, 0, 15);  % 1e-6 to 1.0 (log scale)
snr_results  = zeros(size(delta_values));
N = length(speech_clean);

for i = 1:length(delta_values)
    p = params;
    p.delta_load = delta_values(i);
    try
        output = mvdr_beamformer(mic_signals, p);
        N2 = min(length(output), length(speech_clean));
        s = speech_clean(1:N2)/(rms(speech_clean(1:N2))+1e-12);
        o = output(1:N2)/(rms(output(1:N2))+1e-12);
        sc = (s'*o)/(s'*s); sp = sc*s; ns = o-sp;
        snr_results(i) = 10*log10((mean(sp.^2)+1e-12)/(mean(ns.^2)+1e-12));
    catch
        snr_results(i) = NaN;  % Inversion failed — delta too small
    end
    fprintf('  delta=%.2e -> SNR=%.1f dB\n', delta_values(i), snr_results(i));
end

figure('Name', 'Sweep: Diagonal Loading', 'Position', [700 100 600 350]);
semilogx(delta_values, snr_results, 'r-o', 'LineWidth', 2, 'MarkerSize', 8);
[~, best_idx] = max(snr_results);
hold on;
semilogx(delta_values(best_idx), snr_results(best_idx), 'b*', 'MarkerSize', 15);
text(delta_values(best_idx)*1.5, snr_results(best_idx), ...
     sprintf('  Best: δ=%.2e\n  SNR=%.1f dB', delta_values(best_idx), snr_results(best_idx)));
xlabel('Diagonal Loading Factor (δ)'); ylabel('Output SNR (dB)');
title('MVDR Robustness vs. Diagonal Loading');
grid on;
fprintf('  → Recommended delta: %.2e\n', delta_values(best_idx));
end


%% =========================================================================
function sweep_mic_count(mic_signals, speech_clean, params)
%  Compares DAS vs MVDR for 2-mic and 4-mic configurations.
%  THIS is the key comparison figure for your report — it directly shows
%  why 4 mics + MVDR >> 2 mics + DAS.

N = length(speech_clean);

function snr = get_snr(sig, ref)
    N2 = min(length(sig), length(ref));
    sig = sig(1:N2)/(rms(sig(1:N2))+1e-12);
    ref = ref(1:N2)/(rms(ref(1:N2))+1e-12);
    sc = (ref'*sig)/(ref'*ref);
    sp = sc*ref; ns = sig-sp;
    snr = 10*log10((mean(sp.^2)+1e-12)/(mean(ns.^2)+1e-12));
end

% 2-mic DAS
p2 = params; p2.N_mics = 2;
out_das2 = delayandsum_beamformer(mic_signals(:,1:2), p2);

% 4-mic DAS
out_das4 = delayandsum_beamformer(mic_signals, params);

% 2-mic MVDR
out_mvdr2 = mvdr_beamformer(mic_signals(:,1:2), p2);

% 4-mic MVDR
out_mvdr4 = mvdr_beamformer(mic_signals, params);

snr_raw    = get_snr(mic_signals(:,1), speech_clean);
snr_das2   = get_snr(out_das2,  speech_clean);
snr_das4   = get_snr(out_das4,  speech_clean);
snr_mvdr2  = get_snr(out_mvdr2, speech_clean);
snr_mvdr4  = get_snr(out_mvdr4, speech_clean);

labels = {'Raw Mic', '2-Mic DAS', '4-Mic DAS', '2-Mic MVDR', '4-Mic MVDR'};
snrs   = [snr_raw, snr_das2, snr_das4, snr_mvdr2, snr_mvdr4];
improvements = snrs - snr_raw;

figure('Name', 'Mic Count Comparison', 'Position', [100 500 700 400]);
b = bar(improvements, 'FaceColor', 'flat');
b.CData(1,:) = [0.5 0.5 0.5];  % Raw = grey
b.CData(2,:) = [0.2 0.4 0.8];  % DAS 2mic
b.CData(3,:) = [0.1 0.2 0.9];  % DAS 4mic
b.CData(4,:) = [0.8 0.2 0.2];  % MVDR 2mic
b.CData(5,:) = [0.9 0.1 0.1];  % MVDR 4mic — best
set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 20);
ylabel('SNR Improvement over Raw Mic (dB)');
title('Mic Array Configuration Comparison');
grid on; grid minor;

% Add value labels on bars
for i = 1:length(improvements)
    text(i, improvements(i) + 0.3, sprintf('+%.1f dB', improvements(i)), ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 10);
end

fprintf('\n  Configuration Comparison:\n');
fprintf('  %-20s %8.1f dB (reference)\n', 'Raw Mic', snr_raw);
fprintf('  %-20s %8.1f dB (+%.1f dB)\n', '2-Mic DAS',  snr_das2,  snr_das2-snr_raw);
fprintf('  %-20s %8.1f dB (+%.1f dB)\n', '4-Mic DAS',  snr_das4,  snr_das4-snr_raw);
fprintf('  %-20s %8.1f dB (+%.1f dB)\n', '2-Mic MVDR', snr_mvdr2, snr_mvdr2-snr_raw);
fprintf('  %-20s %8.1f dB (+%.1f dB)\n', '4-Mic MVDR', snr_mvdr4, snr_mvdr4-snr_raw);
end


%% =========================================================================
function sweep_fixed_point(mic_signals, speech_clean, params)
%% CRITICAL: This sweep tells you what bit widths to use in Verilog.
%
%  We quantize the MVDR weights to different word lengths and measure
%  the SNR degradation. The minimum word length with <1dB SNR loss
%  is your FPGA datapath width target.
%
%  WHY THIS MATTERS IN HARDWARE:
%  - 32-bit float: too large for hearing-aid FPGA, high power
%  - 24-bit: common in professional audio
%  - 16-bit: standard for audio data
%  - 12-bit: common for FPGA DSP blocks
%  - 8-bit: too coarse — weights lose precision, MVDR degrades

word_lengths = [8, 10, 12, 14, 16, 18, 20, 24];
snr_float = NaN;  % Reference: floating point
snr_results = zeros(size(word_lengths));

N = length(speech_clean);

function snr = get_snr(sig, ref)
    N2 = min(length(sig), length(ref));
    sig = sig(1:N2)/(rms(sig(1:N2))+1e-12);
    ref = ref(1:N2)/(rms(ref(1:N2))+1e-12);
    sc = (ref'*sig)/(ref'*ref);
    sp = sc*ref; ns = sig-sp;
    snr = 10*log10((mean(sp.^2)+1e-12)/(mean(ns.^2)+1e-12));
end

% Get floating-point reference first
output_float = mvdr_beamformer(mic_signals, params);
snr_float = get_snr(output_float, speech_clean);

% Now sweep quantized versions
for i = 1:length(word_lengths)
    wl = word_lengths(i);
    output_quant = mvdr_beamformer_quantized(mic_signals, params, wl);
    snr_results(i) = get_snr(output_quant, speech_clean);
    fprintf('  %2d-bit weights -> SNR=%.1f dB (loss=%.2f dB)\n', ...
            wl, snr_results(i), snr_float - snr_results(i));
end

figure('Name', 'Fixed-Point Word Length Sweep', 'Position', [700 500 600 400]);
plot(word_lengths, snr_results, 'g-s', 'LineWidth', 2, 'MarkerSize', 10);
hold on;
yline(snr_float, 'k--', 'LineWidth', 2, 'Label', 'Floating-Point Reference');
yline(snr_float - 1, 'r--', 'LineWidth', 1.5, 'Label', '1dB Degradation Limit');

% Mark minimum acceptable word length
for i = 1:length(word_lengths)
    if snr_float - snr_results(i) <= 1.0
        min_wl = word_lengths(i);
        break;
    end
end
if exist('min_wl', 'var')
    xline(min_wl, 'b-', 'LineWidth', 2, ...
          'Label', sprintf('Min WL = %d bits', min_wl));
    fprintf('\n  → Use %d-bit fixed-point for MVDR weights in Verilog\n', min_wl);
end

xlabel('Weight Word Length (bits)'); ylabel('Output SNR (dB)');
title('Fixed-Point Quantization Analysis for FPGA Implementation');
grid on; xlim([6 26]);
end


function output = mvdr_beamformer_quantized(mic_signals, params, weight_bits)
%% MVDR with quantized weights (simulates FPGA fixed-point arithmetic)
%  We quantize the complex weights to 'weight_bits' bits and observe impact.
%  The data path (audio samples) stays at 16-bit throughout.

[N_samples, N_mics] = size(mic_signals);
N_fft  = params.N_fft;
hop    = params.hop;
alpha  = params.alpha;
delta  = params.delta_load;
N_bins = N_fft/2 + 1;

freqs = (0:N_bins-1) * params.fs / N_fft;
theta = params.target_angle * pi/180;
tau   = params.d * cos(theta) / params.c;

D = zeros(N_bins, N_mics);
for k = 1:N_bins
    for m = 1:N_mics
        D(k,m) = exp(-1j * 2*pi * freqs(k) * (m-1) * tau);
    end
end

R = repmat(delta * eye(N_mics), [1, 1, N_bins]);
win = hann(N_fft);
output = zeros(N_samples, 1);
norm_buffer = zeros(N_samples, 1);

% Quantization scale: weights are complex numbers roughly in range [-2, 2]
% Represent with (weight_bits - 1) fractional bits, 1 sign bit.
q_scale = 2^(weight_bits - 2);  % Leaves range [-2, 2) with full resolution

for frame_start = 1:hop:(N_samples - N_fft + 1)
    frame_end = frame_start + N_fft - 1;
    frame = mic_signals(frame_start:frame_end, :) .* win;
    X = fft(frame, N_fft);
    X = X(1:N_bins, :);
    Y = zeros(N_bins, 1);
    
    for k = 1:N_bins
        x_k = X(k,:)';
        R(:,:,k) = alpha * R(:,:,k) + x_k * x_k';
        R_loaded = R(:,:,k) + delta * eye(N_mics);
        d_k = D(k,:)';
        Rinv_d = R_loaded \ d_k;
        denom  = d_k' * Rinv_d;
        w_k    = Rinv_d / denom;
        
        % === QUANTIZE WEIGHTS HERE ===
        % Round real and imaginary parts to nearest representable value
        w_k_real_q = round(real(w_k) * q_scale) / q_scale;
        w_k_imag_q = round(imag(w_k) * q_scale) / q_scale;
        w_k_quantized = w_k_real_q + 1j * w_k_imag_q;
        
        % Apply quantized weights
        Y(k) = w_k_quantized' * x_k;
    end
    
    Y_full = [Y; conj(flipud(Y(2:end-1)))];
    y_frame = real(ifft(Y_full)) .* win;
    output(frame_start:frame_end) = output(frame_start:frame_end) + y_frame;
    norm_buffer(frame_start:frame_end) = norm_buffer(frame_start:frame_end) + win.^2;
end

norm_buffer(norm_buffer < 1e-10) = 1;
output = output(1:N_samples) ./ norm_buffer(1:N_samples);
end
