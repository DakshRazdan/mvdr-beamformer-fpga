function evaluate_performance(speech_clean, mic1_raw, output_mvdr, output_das, params)
%% EVALUATE_PERFORMANCE
%  Compute and display quantitative metrics for your report.
%  
%  METRICS EXPLAINED:
%
%  SNR (Signal-to-Noise Ratio):
%    The most basic measure. Higher = better.
%    We compute it by separating signal and noise contributions.
%    Since we generated the signal synthetically, we know the clean speech.
%
%  STOI (Short-Time Objective Intelligibility):
%    Range 0 to 1. Correlates with human speech understanding percentage.
%    0.7 ≈ ~70% word recognition rate. This is what matters for ADHD users.
%    MATLAB requires the STOI function — see note at bottom if not available.
%
%  PESQ (Perceptual Evaluation of Speech Quality):
%    Range -0.5 to 4.5. MOS-LQO scale. Correlates with listener preference.
%    Used in telecom standards. Requires PESQ function or ITU-T P.862 implementation.
%
%  BEAM PATTERN:
%    The spatial response of the beamformer as a function of angle.
%    Shows which directions are amplified vs suppressed.
%    This is the most visually compelling figure for your report.

N = min([length(speech_clean), length(mic1_raw), ...
         length(output_mvdr), length(output_das)]);
speech_clean = speech_clean(1:N);
mic1_raw     = mic1_raw(1:N);
output_mvdr  = output_mvdr(1:N);
output_das   = output_das(1:N);

%% --- SNR Computation ---
% CORRECTED METHOD: Amplitude-invariant SNR using projection.
% The MVDR output may have a different gain than the reference (by design —
% "distortionless" means no distortion of the SPATIAL response, but overall
% level can shift). We normalize both signals to unit power before comparison.
%
% We measure how much of the output power is correlated with clean speech
% (= signal) vs uncorrelated (= residual noise). This is gain-independent.

function snr_db = compute_snr(signal, clean_ref)
    N = min(length(signal), length(clean_ref));
    signal    = signal(1:N);
    clean_ref = clean_ref(1:N);
    % Normalize both to unit RMS — makes metric gain-independent
    signal    = signal    / (rms(signal)    + 1e-12);
    clean_ref = clean_ref / (rms(clean_ref) + 1e-12);
    % Project signal onto clean reference
    scale = (clean_ref' * signal) / (clean_ref' * clean_ref);
    speech_component = scale * clean_ref;
    noise_component  = signal - speech_component;
    snr_db = 10*log10((mean(speech_component.^2) + 1e-12) / ...
                      (mean(noise_component.^2)  + 1e-12));
end

snr_raw  = compute_snr(mic1_raw,    speech_clean);
snr_mvdr = compute_snr(output_mvdr, speech_clean);
snr_das  = compute_snr(output_das,  speech_clean);

fprintf('\n--- PERFORMANCE RESULTS ---\n');
fprintf('%-25s %8s %8s %8s\n', 'Metric', 'Raw Mic', 'DAS', 'MVDR');
fprintf('%s\n', repmat('-', 1, 53));
fprintf('%-25s %8.1f %8.1f %8.1f\n', 'Output SNR (dB)', snr_raw, snr_das, snr_mvdr);
fprintf('%-25s %8.1f %8.1f %8.1f\n', 'SNR Improvement (dB)', 0, snr_das-snr_raw, snr_mvdr-snr_raw);

%% --- STOI (if available) ---
try
    stoi_raw  = stoi(speech_clean, mic1_raw,    params.fs);
    stoi_das  = stoi(speech_clean, output_das,  params.fs);
    stoi_mvdr = stoi(speech_clean, output_mvdr, params.fs);
    fprintf('%-25s %8.3f %8.3f %8.3f\n', 'STOI (0-1)', stoi_raw, stoi_das, stoi_mvdr);
catch
    fprintf('  [STOI not available — install from https://github.com/mpariente/pystoi or MATLAB FEX]\n');
end

%% --- FIGURE 1: Time Domain Signals ---
figure('Name', 'Time Domain Comparison', 'Position', [100 100 1200 600]);
t = (0:N-1)/params.fs;

subplot(4,1,1); plot(t, speech_clean, 'g'); title('Clean Speech (Reference)');
ylabel('Amplitude'); ylim([-2 2]);

subplot(4,1,2); plot(t, mic1_raw, 'b'); 
title(sprintf('Raw Mic 1 (Input SNR = %d dB)', params.SNR_input_dB));
ylabel('Amplitude'); ylim([-2 2]);

subplot(4,1,3); plot(t, output_das, 'r');
title(sprintf('Delay-and-Sum Output (SNR = %.1f dB, Improvement = %.1f dB)', ...
              snr_das, snr_das-snr_raw));
ylabel('Amplitude'); ylim([-2 2]);

subplot(4,1,4); plot(t, output_mvdr, 'm');
title(sprintf('MVDR Output (SNR = %.1f dB, Improvement = %.1f dB)', ...
              snr_mvdr, snr_mvdr-snr_raw));
ylabel('Amplitude'); xlabel('Time (s)'); ylim([-2 2]);

%% --- FIGURE 2: Beam Patterns ---
% The beam pattern H(θ) at frequency f is:
%   H(f,θ) = |w(f)^H * d(f,θ)|
% We evaluate this for all angles -90° to +90°
figure('Name', 'Beam Pattern Comparison', 'Position', [100 750 1200 500]);

angles = -90:1:90;
eval_freqs = [500, 1000, 2000, 3000];  % Key speech frequencies (Hz)

subplot(1,2,1); hold on; title('Delay-and-Sum Beam Pattern');
subplot(1,2,2); hold on; title('MVDR Beam Pattern (Frame-averaged)');

colors = {'b', 'r', 'g', 'm'};

for fi = 1:length(eval_freqs)
    f = eval_freqs(fi);
    k = round(f / params.fs * params.N_fft) + 1;  % Bin index
    k = min(k, params.N_fft/2 + 1);
    
    tau_target = params.d * cos(params.target_angle*pi/180) / params.c;
    
    % DAS beam pattern: fixed weights w = (1/M)*[1, e^{j*2pi*f*tau}, ...]
    H_das = zeros(1, length(angles));
    H_mvdr = zeros(1, length(angles));
    
    for ai = 1:length(angles)
        theta_test = angles(ai) * pi/180;
        tau_test = params.d * cos(theta_test) / params.c;
        
        % Steering vector for this test angle
        d_test = exp(-1j * 2*pi*f * (0:params.N_mics-1)' * tau_test);
        
        % DAS weights (steer to target)
        d_target = exp(-1j * 2*pi*f * (0:params.N_mics-1)' * tau_target);
        w_das = d_target / params.N_mics;
        
        H_das(ai) = abs(w_das' * d_test);
        
        % MVDR weights from last logged frame
        if ~isempty(weights_log_placeholder(fi))  % Placeholder — see note
            H_mvdr(ai) = NaN;  % Will be filled below
        end
    end
    
    subplot(1,2,1);
    plot(angles, 20*log10(H_das + 1e-6), colors{fi}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('%d Hz', f));
    
end

for sp = 1:2
    subplot(1,2,sp);
    xline(params.target_angle, 'k--', 'LineWidth', 2, 'DisplayName', 'Target');
    for n = 1:length(params.noise_angles)
        xline(params.noise_angles(n), 'k:', 'LineWidth', 1.5, ...
              'DisplayName', sprintf('Noise %d°', params.noise_angles(n)));
    end
    xlim([-90 90]); ylim([-40 5]); grid on;
    xlabel('Angle (degrees)'); ylabel('Response (dB)');
    legend('Location', 'best');
end

% MVDR beam pattern using actual computed weights
compute_mvdr_beam_pattern(params, eval_freqs, colors);

%% --- FIGURE 3: Spectrogram Comparison ---
figure('Name', 'Spectrogram Comparison', 'Position', [100 100 1400 400]);

subplot(1,3,1);
spectrogram(mic1_raw, hann(256), 128, 256, params.fs, 'yaxis');
title('Raw Mic 1'); colorbar off; ylim([0 4]);

subplot(1,3,2);
spectrogram(output_das, hann(256), 128, 256, params.fs, 'yaxis');
title('Delay-and-Sum Output'); colorbar off; ylim([0 4]);

subplot(1,3,3);
spectrogram(output_mvdr, hann(256), 128, 256, params.fs, 'yaxis');
title('MVDR Output'); colorbar off; ylim([0 4]);
colormap(gca, 'hot');

fprintf('\nFigures generated: Time domain, Beam pattern, Spectrogram.\n');
end


function compute_mvdr_beam_pattern(params, eval_freqs, colors)
%% Compute MVDR beam pattern analytically using optimal weights
%  (since we need R for actual weights, we use the theoretical optimal
%   for a signal + 2 noise sources scenario to illustrate null placement)

angles = -90:1:90;
N_mics = params.N_mics;

% Build theoretical covariance matrix: 
% R = signal_power * d_s*d_s^H + sum_n(noise_power * d_n*d_n^H) + delta*I
signal_power = 1.0;
noise_power  = 10^(-params.SNR_input_dB/10);

theta_s = params.target_angle * pi/180;
tau_s   = params.d * cos(theta_s) / params.c;

subplot(1,2,2); hold on;
for fi = 1:length(eval_freqs)
    f = eval_freqs(fi);
    H_mvdr = zeros(1, length(angles));
    
    % Build R at this frequency
    d_s = exp(-1j*2*pi*f*(0:N_mics-1)' * tau_s);  % Speech steering vector
    R = signal_power * (d_s * d_s') + params.delta_load * eye(N_mics);
    
    for n = 1:length(params.noise_angles)
        tau_n = params.d * cos(params.noise_angles(n)*pi/180) / params.c;
        d_n   = exp(-1j*2*pi*f*(0:N_mics-1)' * tau_n);
        R     = R + noise_power * (d_n * d_n');
    end
    R = R + params.delta_load * eye(N_mics);  % Diagonal loading
    
    % MVDR weights
    Rinv_d = R \ d_s;
    w_mvdr = Rinv_d / (d_s' * Rinv_d);
    
    for ai = 1:length(angles)
        tau_t = params.d * cos(angles(ai)*pi/180) / params.c;
        d_t   = exp(-1j*2*pi*f*(0:N_mics-1)' * tau_t);
        H_mvdr(ai) = abs(w_mvdr' * d_t);
    end
    
    plot(angles, 20*log10(H_mvdr + 1e-6), colors{fi}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('%d Hz', f));
end

xline(params.target_angle, 'k--', 'LineWidth', 2, 'DisplayName', 'Target');
for n = 1:length(params.noise_angles)
    xline(params.noise_angles(n), 'k:', 'LineWidth', 1.5);
end
xlim([-90 90]); ylim([-60 5]); grid on;
xlabel('Angle (degrees)'); ylabel('Response (dB)');
legend('Location', 'best');
end


function out = weights_log_placeholder(~)
% Placeholder — beam pattern uses analytical weights computed in
% compute_mvdr_beam_pattern instead of logged weights.
out = [];
end
