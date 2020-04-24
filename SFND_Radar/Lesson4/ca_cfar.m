%%
clearvars; clc;
% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% Data_points
Ns = 1000;

% Generate random noise
s=abs(randn(Ns,1)); % Bug - the original template did not have abs
% Of course, we are simulating the FFT

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100 ,200, 300, 700])=[8 9 4 11];

%plot the output
figure;
plot(s);

% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
% 1b. Guard Cells
T = 12; G = 4;

% Offset : Adding room above noise threshold for desired SNR
offset=5; % Changed to 5 instead of 3

% More efficient
% Create convolution kernel that performs averaging of training
% blocks except for guard and centre
f = ones(2*(T + G) + 1, 1);
f(T + G : T + 2*G + 1) = 0;
f = f / sum(f);

% Perform convolution to perform averaging of each block to compute
% thresholds
threshold_cfar = conv(s, f, 'same') * offset;

% Copy signal over and any values in the signal less than the threshold,
% set to 0
signal_cfar = s;
signal_cfar(s < threshold_cfar) = 0;

% Plot - note that we don't need to circular shift anymore as the
% convolution with 'same' handles that for us
% Also got rid of the ridiculous use of cells - not needed
figure;
plot(signal_cfar,'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(s);
hold on, plot(threshold_cfar,'r--','LineWidth',2)
hold on, plot(signal_cfar,'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')

%%
% OLD INEFFICIENT CODE
% % Vector to hold threshold values
% threshold_cfar = [];
%
% %Vector to hold final signal after thresholding
% signal_cfar = [];
%
% % 2. Slide window across the signal length
% for i = 1:(Ns-(G+T))
%
%     % 2. - 5. Determine the noise threshold by measuring it within the training cells
%     max_index = min(i + 2*(T + G), Ns);
%     window = s(i : max_index);
%     max_index = min(T + 2*G + 1, numel(window));
%     window(T + G : max_index) = [];
%     avg = mean(window) * offset;
%     threshold_cfar{end+1} = avg;
%     % 6. Measuring the signal within the CUT
%     max_index = min(i + T + G, Ns);
%     signal = s(max_index);
%
%     % 8. Filter the signal above the threshold
%     if signal < avg
%         signal = 0;
%     end
%     signal_cfar = [signal_cfar, {signal}];
% end
%
%
% % plot the filtered signal - OLD CODE
% figure;
% plot(cell2mat(signal_cfar),'g--');
%
% % plot original sig, threshold and filtered signal within the same figure.
% figure,plot(s);
% hold on, plot(circshift(threshold_cfar,G),'r--','LineWidth',2)
% hold on, plot(circshift(signal_cfar,(T+G)),'g--','LineWidth',4);
% legend('Signal','CFAR Threshold','detection')