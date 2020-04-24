clc; clearvars; close all;
Fs = 1000;            % Sampling frequency
T = 1/Fs;             % Sampling period
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector

% TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
S = 0.7 * sin(2 * pi * 77 * t) + 2 * sin(2 * pi * 43 * t);

% Corrupt the signal with noise
X = S + 2*randn(size(t));

% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t).
figure;
plot(1000*t(1:50) ,X(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% TODO : Compute the Fourier transform of the signal.
% Specify total number of FFT points - do 1024
N = 1024;
Xf = fft(X, N);

% TODO : Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
P1 = abs(Xf / N); % Usually normalise by the length of the signal
P1 = P1(1 : N/2 + 1);

% Plotting
figure;
f = Fs*(0:(N/2))/N; % Changed from L to N
plot(f,P1)
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')