clc; clearvars; close all;
% TODO : Find the Bsweep of chirp for 1 m resolution
% From Lesson 3 - Part 1
% dres = c / (2*Bsweep)
% so --> Bsweep  = dres*c/2
c = 3e8;
dres = 1; % range resolution
Bsweep = dres * c / 2;

% TODO : Calculate the chirp time based on the Radar's Max Range
Rmax = 300; % metres
%            (  Trip Time  )
Tchirp = 5.5 * 2 * Rmax / c;

% TODO : define the frequency shifts
% Beat frequencies
frequency_shifts = [0, 1.1e6, 13e6, 24e6];

% Calculate the ranges now
calculated_range = c * Tchirp * frequency_shifts / (2 * Bsweep);

% Display the calculated range
disp(calculated_range);