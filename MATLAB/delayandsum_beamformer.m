function output = delayandsum_beamformer(mic_signals, params)
%% DELAYANDSUM_BEAMFORMER  —  Classical Broadside Beamformer
%
%  The simplest beamformer. For each mic, apply a time delay to align 
%  the target signal, then average across all mics.
%
%  Y(n) = (1/M) * sum_m { x_m(n - delay_m) }
%
%  WHY IT'S LIMITED:
%  - Fixed weights (1/M) — no optimization, no null steering
%  - Beam width depends on frequency: narrow at high freq, wide at low freq
%  - Noise suppression = 10*log10(M) = 6dB for 4 mics. That's it.
%  - Noise from any direction gets partially suppressed equally — no 
%    ability to place a null specifically toward a noise source.
%
%  MVDR, by contrast, adapts weights per-bin to place explicit nulls,
%  achieving 15-20dB suppression toward known noise directions.
%  This is the core argument for your report.

[N_samples, N_mics] = size(mic_signals);
theta = params.target_angle * pi/180;
aligned = zeros(N_samples, N_mics);

for m = 1:N_mics
    % Compute integer delay for alignment (approximate — DAS uses integer delays)
    delay_samples = (m-1) * params.d * cos(theta) / params.c * params.fs;
    int_delay = round(delay_samples);  % Integer approximation — this is the lossy part
    
    % Shift signal to align with mic 1
    if int_delay >= 0
        aligned(int_delay+1:end, m) = mic_signals(1:end-int_delay, m);
    else
        aligned(1:end+int_delay, m) = mic_signals(-int_delay+1:end, m);
    end
end

% Simple average — uniform weights
output = mean(aligned, 2);

fprintf('  DAS: Integer delays applied (max delay: %d samples)\n', ...
        round((N_mics-1) * params.d * cos(theta) / params.c * params.fs));
end
