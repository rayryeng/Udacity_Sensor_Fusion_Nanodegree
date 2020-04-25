%%
clearvars; close all;
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
max_range = 200;
range_res = 1;
max_velocity = 100;
c = 3e8;
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
R = 110;
v = -20;
 


%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B = c / (2 * range_res);
Tchirp = 5.5 * 2 * max_range / c;
slope = B / Tchirp;

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
%Tx=zeros(1,length(t)); %transmitted signal
%Rx=zeros(1,length(t)); %received signal
%Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
%r_t=zeros(1,length(t));
%td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 
alpha = slope;
% Inefficient method with loops - please consult the vectorized version
% below which implements the loops but using vectorization
% MATLAB operates more efficiently when using vectorization
% However, this loop below is for self-containment in case what is
% being evaluated is the "loopy" version

% for i=1:length(t)         
%     % *%TODO* :
%     %For each time stamp update the Range of the Target for constant velocity. 
%     % Initial range is R
%     r_t(i) = R + v * t(i);
%     % *%TODO* :
%     %For each time sample we need update the transmitted and
%     %received signal. 
%     % Recall - signal trip time = 2 * range / c
%     td(i) = 2 * r_t(i) / c;
%     Tx(i) = cos(2*pi*(fc*t(i) + (alpha*(t(i)^2)/2)));
%     Rx(i) = cos(2*pi*(fc*(t(i) - td(i)) + (alpha*((t(i) - td(i))^2)/2)));
%     
%     % *%TODO* :
%     %Now by mixing the Transmit and Receive generate the beat signal
%     %This is done by element wise matrix multiplication of Transmit and
%     %Receiver Signal
%     Mix(i) = Tx(i) * Rx(i);
% end

% Using vectorization instead
r_t = R + v * t;
td = 2 * r_t / c;
Tx = cos(2*pi*(fc*t + (alpha*(t.^2)/2)));
Rx = cos(2*pi*(fc*(t - td) + (alpha*((t - td).^2)/2)));
Mix = Tx .* Rx;
%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix_matrix = reshape(Mix, [Nr, Nd]);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
Mix_FFT = fft(Mix_matrix, [], 1) / Nr;

 % *%TODO* :
% Take the absolute value of FFT output
Mix_FFT_mag = abs(Mix_FFT);

% Find the largest value for each row
Mix_FFT_mag = max(Mix_FFT_mag, [], 2);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
Mix_FFT_mag = Mix_FFT_mag(1 : Nr/2);

%plotting the range
figure ('Name','Range from First FFT')
%subplot(2,1,1)

 % *%TODO* :
 % plot FFT output
 plot(Mix_FFT_mag);
 xlabel('Range (ft)');
 ylabel('FFT Magnitude');

 
axis ([0 200 0 1]);
grid;



%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);
xlabel('Doppler velocity (m/s)');
ylabel('Range (m)');
zlabel('Log magnitude');

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 10;
Td = 8;
% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 4;
Gd = 4;
% *%TODO* :
% offset the threshold by SNR value in dB
offset = 6;
% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
%noise_level = zeros(1,1);


% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.

% Inefficient method -
% Taken from the video with slight modifications
% Please consult the method below that uses 2D convolution to achieve
% the weighted sums and logical operators for vectorized thresholding
% MATLAB operates more efficiently when using vectorization
% However, this loop below is for self-containment in case what is
% being evaluated is the "loopy" version

% RDM_copy = RDM;
% for i = Tr + Gr + 1 : Nr/2 - (Gr + Tr)
%     for j = Td + Gd + 1 : Nd - (Gd + Td)
%         noise_level = 0;
%         for p = i - (Tr + Gr) : i + Tr + Gr
%             for q = j - (Td + Gd) : j + Td + Gd
%                 if abs(i - p) > Gr || abs(j - q) > Gd
%                     noise_level = noise_level + db2pow(RDM_copy(p, q));
%                 end
%             end
%         end
%         threshold = pow2db(noise_level / ( ((2 * Tr + 2 * Gr + 1) * (2 * Td + 2 * Gd + 1)) - ((2 * Gr + 1)*(2 * Gd + 1))));
%         threshold = threshold + offset;
%         if RDM(i, j) < threshold
%             RDM(i, j) = 0;
%         else
%             RDM(i, j) = 1;
%         end
%     end
% end

% Define in case you're using Octave
pow2db = @(x) 10*log10(x);
db2pow = @(x) 10.^(x / 10);

% Vectorized form
% Use conv2 to do the averaging
% Define mask where guard bands + centre is set to 0
% Divide by the total number of non-zero elements
mask = ones(2 * Tr + 2 * Gr + 1, 2 * Td + 2 * Gd + 1);
centre_coord = [Tr + Gr + 1, Td + Gd + 1];
mask(centre_coord(1) - Gr : centre_coord(1) + Gr, centre_coord(2) - Gd : centre_coord(2) + Gd) = 0;
mask = mask / sum(mask(:));
% Perform 180 degree rotation as convolution will do this so rotating by
% 180 first, then letting conv2 do it will result in the original mask
mask = mask(end:-1:1, end:-1:1);
% Convolve, then convert back to dB to add the offset
% The convolution defines the threshold
threshold = conv2(db2pow(RDM), mask, 'same');
threshold = pow2db(threshold) + offset;

% Any values less than the threshold are 0, else 1
RDM(RDM < threshold) = 0;
RDM(RDM >= threshold) = 1;


   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR





% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0.
RDM(1 : Tr + Gr, :) = 0;
RDM(Nr/2 - (Gr + Tr) + 1 : end, :) = 0;
RDM(:, 1 : Td + Gd) = 0;
RDM(:, Nd - (Gd + Td) + 1 : end) = 0;







% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis, RDM);
colorbar;


 
 