fs = 44100;                 % Standard audio sampling rate (Hz)
duration = 1.5;             % Length of the tone (seconds)
%f = 440;                    % Target frequency in Hz (e.g., 440 for A4)

f = input('enter frequency/frequencies in Hz: ');

t = 0 : 1/fs : duration;    % Time vector

y = zeros(size(t));

for i=1:length(f)
    phase(i) = (rand-0.5)*2*pi;
    phase(i) = 0;
    y = y + sin(2 * pi * f(i) * t + phase(i));    % Pure sine wave
end

sound(y, fs);               % Play audio through default speakers

% 3. Time-domain plot
N = length(y);
t = (0:N-1) / fs;       % Time vector in seconds

figure;
subplot(2,1,1);
plot(t, y);
xlabel('Time (s)');
ylabel('Amplitude');
title('Time-Domain Signal');
grid on;

% 4. Frequency-domain plot (Single-Sided Amplitude Spectrum)
Y = fft(y);
P2 = abs(Y / N);
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2 * P1(2:end-1);
f = fs * (0:floor(N/2)) / N;

subplot(2,1,2);
plot(f, P1);
xlabel('Frequency (Hz)');
ylabel('|P1(f)|');
title('Single-Sided Amplitude Spectrum');
xlim([0 5000]);         % Zoom in on 0-5 kHz (adjust up to Fs/2 as needed)
grid on;
