% ============================================================
% vad_out.m — Voice Activity Detection (Fixed for MVDR system)
% Compatible with: 16kHz, 128-sample frames
% ============================================================
clc; clear; close all;

fs        = 16000;   % Must match FPGA (16kHz)
frame_len = 320;     % 20ms frame for vad_out (320 samples at 16kHz)
frame_shift = 128;   % Match FPGA hop size (was 160, now 128)

%% Generate test (replace with real beamformer output)
t = (0:fs*2-1)/fs;                          % 2 seconds
speech_seg = 0.5*sin(2*pi*1000*t);
noise_seg  = 0.05*randn(size(t));
% First 1s = noise only, second 1s = speech+noise
x = [noise_seg(1:fs), speech_seg(fs+1:end) + noise_seg(1:fs)];
x = x(:);

%% Frame blocking
num_frames = floor((length(x) - frame_len) / frame_shift) + 1;
energy = zeros(1, num_frames);
zcr    = zeros(1, num_frames);

for i = 1:num_frames
    idx   = (i-1)*frame_shift + 1;
    frame = x(idx : idx+frame_len-1);

    % Short-time energy (normalized)
    energy(i) = sum(frame.^2) / frame_len;

    % Zero crossing rate
    zcr(i) = sum(abs(diff(sign(frame)))) / (2*frame_len);
end

%% Thresholds (DO NOT CHANGE — matched to Verilog)
energy_th = 0.01;    % Q1.15 equivalent: 328 (was 0.1, lowered for 128-sample frames)
zcr_low   = 0.02;    % Q1.15: 655
zcr_high  = 0.20;    % Q1.15: 6554

%% vad_out Decision
vad_out = (energy > energy_th) & (zcr > zcr_low) & (zcr < zcr_high);

%% Detection rate
speech_frames = round(fs / frame_shift);  % approx frames in speech region
detected = sum(vad_out(end-speech_frames:end));
fprintf('Speech frames detected: %d / %d\n', detected, speech_frames);

%% Plot
figure;
subplot(3,1,1); plot(t, x); title('Input Signal (Beamformer Output)');
subplot(3,1,2); plot(energy); yline(energy_th,'r--'); title('Energy + Threshold');
subplot(3,1,3); stairs(vad_out); ylim([-0.2 1.2]); title('vad_out Output (1=Speech)');
xlabel('Frame Index');