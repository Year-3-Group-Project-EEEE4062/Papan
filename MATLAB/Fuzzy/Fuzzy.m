% /////////////////////////////////////////////////////////////////////////
% Analyse collected data.
% /////////////////////////////////////////////////////////////////////////


clc; clear all; close all;


% /////////////////////////////////////////////////////////////////////////
% Load data.
% /////////////////////////////////////////////////////////////////////////
data = readmatrix('Fuzzy9990.csv'); 
% /////////////////////////////////////////////////////////////////////////
% Plot angle difference and time.
time = data(:, 1);
time = time - time(1);

angleDiff = data(:, 2);
leftMotor = data(:, 3);
rightMotor = data(:, 4);

figure(1);
plot(time, angleDiff, '-r');
xlabel('Time (seconds)');
ylabel('Angle Difference (Degree)');
title('Error Signal for Heading');

figure(2);
plot(time, leftMotor, '-r');
xlabel('Time (seconds)');
ylabel('Left Motor Speed (%)');
title('Left Motor Speed');

figure(3);
plot(time, rightMotor, '-r');
xlabel('Time (seconds)');
ylabel('Right Motor Speed (%)');
title('Right Motor Speed');