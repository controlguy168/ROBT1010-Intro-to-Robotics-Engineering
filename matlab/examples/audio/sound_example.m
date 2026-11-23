% 1. Record audio
Fs = 44100;             % Sample rate (Hz)
duration = 5;           % Seconds
recObj = audiorecorder(Fs, 16, 1);

disp('Start speaking.');
recordblocking(recObj, duration);
disp('End of recording.');

y = getaudiodata(recObj);

% 2. Listen to the recorded signal
sound(y, Fs);           % Plays audio at original sample rate

% 3. Time-domain plot
N = length(y);
t = (0:N-1) / Fs;       % Time vector in seconds

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
f = Fs * (0:floor(N/2)) / N;

subplot(2,1,2);
plot(f, P1);
xlabel('Frequency (Hz)');
ylabel('|P1(f)|');
title('Single-Sided Amplitude Spectrum');
xlim([0 5000]);         % Zoom in on 0-5 kHz (adjust up to Fs/2 as needed)
grid on;
