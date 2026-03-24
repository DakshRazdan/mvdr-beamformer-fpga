% ============================================================
% lms_filter.m — LMS Adaptive Filter (Fixed for MVDR system)
% Compatible with: 16kHz, 128-sample hop, Q1.15 fixed point
% ============================================================
clc; clear; close all;

fs = 16000;          % Must match FPGA output (16kHz)
frame_len = 128;     % Match FPGA hop size (was 320, now 128)

%% Generate test signal (replace with real mic input later)
t = (0:fs-1)/fs;
speech   = 0.5 * sin(2*pi*1000*t);           % 1kHz target speech
noise    = 0.3 * randn(size(speech));          % White noise
noisy    = speech + noise;

%% LMS Parameters (DO NOT CHANGE — matched to Verilog)
M   = 32;            % Filter taps (fixed in hardware)
mu  = 0.001;         % Step size (Q1.15: mu*32767 = 32)
w   = zeros(M,1);    % Initial weights

N      = length(noisy);
output = zeros(N,1); % Noise estimate
error  = zeros(N,1); % Denoised = noisy - noise_estimate

%% LMS Algorithm (real-time frame-by-frame, matches hardware)
for n = M:N
    x      = noisy(n:-1:n-M+1);   % Tap delay line
    y      = w' * x;               % Filter output (noise estimate)
    output(n) = y;
    error(n)  = noisy(n) - y;      % Error = denoised signal
    w = w + mu * x * error(n);     % Weight update
end

%% SNR Measurement
signal_power = mean(speech(M:end).^2);
noise_power  = mean((error(M:end) - speech(M:end)).^2);
snr_out = 10*log10(signal_power / noise_power);
fprintf('Output SNR: %.2f dB\n', snr_out);

%% Plot
figure;
subplot(3,1,1); plot(t, noisy);  title('Noisy Input (from MVDR output)');
subplot(3,1,2); plot(t, output); title('Estimated Noise');
subplot(3,1,3); plot(t, error);  title('Denoised Output (LMS)'); xlabel('Time (s)');