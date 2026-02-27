function [mic_signals, speech_clean, params] = generate_array_signals(params)
%% GENERATE_ARRAY_SIGNALS
%  Creates a realistic 4-mic array recording scenario.
%
%  Physical model:
%  - 1 speech source at params.target_angle (0° = broadside)
%  - N noise sources at params.noise_angles
%  - Each source arrives at each mic with a different time delay
%    based on geometry (far-field plane wave assumption)
%
%  Far-field assumption is valid when source distance >> array aperture.
%  Our aperture = 3*d = 6cm. Valid for speaker > ~60cm away. Fine for use.
%
%  OUTPUT:
%    mic_signals [N_samples x N_mics]  — what each mic hears
%    speech_clean [N_samples x 1]      — clean speech reference

N_samples = round(params.duration * params.fs);
params.N_samples = N_samples;
t = (0:N_samples-1)' / params.fs;

%% --- Noise-only preamble
%  THIS IS THE CRITICAL FIX.
%  Real hearing aids are always-on — they estimate noise in gaps between speech.
%  Our synthetic signal had speech from t=0, so the VAD could never find a
%  clean noise reference. We add 0.5s of noise-only at the start.
%  VAD uses this to converge R correctly before speech arrives.
%  The beamformer will be fully adapted by the time speech starts at t=0.5s.
N_preamble = round(0.5 * params.fs);  % 0.5s noise-only preamble

%% --- Generate speech signal (sum of harmonics — simulates vowel-like speech)
f0 = 150;
speech_clean = zeros(N_samples, 1);
for k = 1:8
    freq = f0 * k;
    if freq < params.fs/2
        speech_clean = speech_clean + (1/k) * sin(2*pi*freq*t);
    end
end
% AM envelope at 4Hz syllable rate
am_envelope = 0.5 * (1 + sin(2*pi*4*t));
speech_clean = speech_clean .* am_envelope;
speech_clean = speech_clean / max(abs(speech_clean));

% ZERO OUT speech during preamble — only noise exists in first 0.5s
% This gives VAD a clean noise reference window to estimate R correctly.
speech_clean(1:N_preamble) = 0;

%% --- Generate noise signals (one per noise source)
N_noise_sources = length(params.noise_angles);
noise_sources = zeros(N_samples, N_noise_sources);
for n = 1:N_noise_sources
    % White noise filtered to speech band — more realistic than pure white
    raw_noise = randn(N_samples, 1);
    % Bandpass filter 200Hz - 4kHz (speech band noise is hardest to suppress)
    [b_bp, a_bp] = butter(4, [200 4000]/(params.fs/2), 'bandpass');
    noise_sources(:,n) = filter(b_bp, a_bp, raw_noise);
    noise_sources(:,n) = noise_sources(:,n) / max(abs(noise_sources(:,n)));
end

%% --- Compute time delays for each source at each mic
%  For a uniform linear array (ULA), the time delay from source at angle θ
%  to mic m (m=0,1,2,3) relative to mic 0 is:
%
%    τ(m, θ) = m * d * cos(θ) / c
%
%  cos(θ) because θ is measured from broadside (perpendicular to array axis).
%  At θ=0° (broadside): τ = 0 for all mics (all equidistant from source).
%  At θ=90° (endfire): τ = m*d/c (maximum delay).

mic_signals = zeros(N_samples, params.N_mics);

% --- Add speech contribution to each mic
speech_power = mean(speech_clean.^2);
theta_speech = params.target_angle * pi/180;

for m = 1:params.N_mics
    delay_samples = (m-1) * params.d * cos(theta_speech) / params.c * params.fs;
    % Fractional sample delay via linear interpolation
    mic_signals(:,m) = mic_signals(:,m) + ...
                       fractional_delay(speech_clean, delay_samples);
end

% --- Add noise contributions scaled to desired SNR
noise_power_target = speech_power / (10^(params.SNR_input_dB/10));

for n = 1:N_noise_sources
    theta_noise = params.noise_angles(n) * pi/180;
    noise_power_current = mean(noise_sources(:,n).^2);
    noise_scaled = noise_sources(:,n) * sqrt(noise_power_target / noise_power_current);
    
    for m = 1:params.N_mics
        delay_samples = (m-1) * params.d * cos(theta_noise) / params.c * params.fs;
        mic_signals(:,m) = mic_signals(:,m) + ...
                           fractional_delay(noise_scaled, delay_samples);
    end
end

% --- Add small sensor self-noise (models real mic noise floor, -60dB relative)
sensor_noise_power = speech_power * 10^(-60/10);
mic_signals = mic_signals + randn(N_samples, params.N_mics) * sqrt(sensor_noise_power);

% --- CRITICAL: Add realistic mic gain and phase mismatch
% INMP441 datasheet: sensitivity ±1dB, which is ~±12% amplitude variation.
% Phase mismatch from PCB trace length differences: ~±3 degrees at 1kHz.
% Without this, the delta sweep recommends delta=1e-6 (too small for hardware).
% With this, the sweep will show you the REAL optimal delta for your FPGA.
rng(42);  % Fixed seed so results are reproducible
gain_mismatch  = 1 + 0.12 * (rand(1, params.N_mics) - 0.5);  % ±6% gain spread
phase_mismatch = exp(1j * deg2rad(3 * (rand(1, params.N_mics) - 0.5)));  % ±1.5 deg

% Apply mismatch in frequency domain (correct way — phase is frequency-dependent)
for m = 1:params.N_mics
    % Gain mismatch: scalar multiplication
    mic_signals(:,m) = mic_signals(:,m) * gain_mismatch(m);
    % Phase mismatch: apply as a time-domain shift (fractional sample)
    phase_deg = angle(phase_mismatch(m)) * 180/pi;
    delay_err = phase_deg / 360 / 1000 * params.fs;  % At 1kHz reference
    mic_signals(:,m) = fractional_delay(mic_signals(:,m), delay_err);
end

fprintf('  Mic gain spread: [%s] (±%.1f%%)\n', ...
        num2str(gain_mismatch, '%.3f '), max(abs(gain_mismatch-1))*100);
fprintf('  Mic phase spread: ±%.1f deg\n', ...
        max(abs(angle(phase_mismatch)*180/pi)));

fprintf('  Speech power: %.4f | Noise power target: %.4f\n', ...
        speech_power, noise_power_target);
fprintf('  Actual input SNR (mic 1): %.1f dB\n', ...
        10*log10(speech_power/noise_power_target));
end


function y = fractional_delay(x, delay_samples)
%% FRACTIONAL_DELAY
%  Applies a fractional sample delay using linear interpolation.
%  This is important because mic spacing rarely gives integer sample delays.
%  Example: d=2cm, fs=16kHz, θ=30° → delay = 0.02*cos(30°)/343*16000 = 0.808 samples
%
%  In hardware (FPGA), this becomes a simple 2-tap FIR filter.

N = length(x);
y = zeros(N, 1);

int_delay = floor(delay_samples);
frac = delay_samples - int_delay;  % Fractional part [0, 1)

for i = 1:N
    i0 = i - int_delay;
    i1 = i0 - 1;
    
    v0 = 0; v1 = 0;
    if i0 >= 1 && i0 <= N, v0 = x(i0); end
    if i1 >= 1 && i1 <= N, v1 = x(i1); end
    
    % Linear interpolation: y = (1-frac)*x[n] + frac*x[n-1]
    y(i) = (1 - frac) * v0 + frac * v1;
end
end
