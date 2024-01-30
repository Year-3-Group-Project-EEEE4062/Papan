% /////////////////////////////////////////////////////////////////////////
% Path generation for testing purpose.
% /////////////////////////////////////////////////////////////////////////


clear all; close all; clc;


% 2D path.
x = [0 33 46.8 50 52];
y = [0 3 5 7.7 10.8];

% Waypoints.
waypoints = [x; y];

% Time at each waypoint.
timePoints = [0 5 8 10 12];

% Number of points.
numSamples = 100;

% Generates a minimum jerk polynomial trajectory. 
[q,qd,qdd,qddd,pp,tPoints,tSamples] = minjerkpolytraj(waypoints,timePoints,numSamples);

% Plot the graph displacement in 2D plane.
figure(1);
plot(q(1, :), q(2, :), 'r');
axis([-5 60 -5 60]);
title('2D Plot');
xlabel('x-axis (m)');
ylabel('y-axis (m)');
grid on;

hold on;
plot(55, 20, 'xb-');