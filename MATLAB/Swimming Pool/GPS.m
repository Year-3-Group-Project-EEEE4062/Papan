% /////////////////////////////////////////////////////////////////////////
% Analyse collected data.
% /////////////////////////////////////////////////////////////////////////


clc; clear all; close all;


% /////////////////////////////////////////////////////////////////////////
% Load data.
% /////////////////////////////////////////////////////////////////////////
data = readmatrix('autoModeSwimmingYx.csv');
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Analyse data for lake.
% /////////////////////////////////////////////////////////////////////////
time = data(:, 1);
time = time - time(1);
latitude = data(:, 4);
longitude = data(:, 5);
setLatitude = data(:, 7);
setLongitude = data(:, 8);
bearing = data(:, 9);
heading = data(:, 10);
angleDiff = data(:, 11);
leftMotor = data(:, 12);
rightMotor = data(:, 13);

% Initialize arrays to store converted coordinates.
x = zeros(size(latitude));
y = zeros(size(longitude));
distance = zeros(size(latitude));
% Convert latitude and longitude to meters.
for i = 2:length(latitude)
    [x(i), y(i), ~, ~] = convertTo2DPlane(latitude(i), longitude(i), latitude(1), longitude(1));
end
for i = 1:length(latitude)
    [~, ~, ~, distance(i)] = convertTo2DPlane(latitude(i), longitude(i), setLatitude(i), setLongitude(i));
end

% Array to indicate toggling waypoint.
toggleWP = [];

% Use the unique function to get unique values from latitude and longitude.
setLat = [];
setLon = [];
for i = 1:length(setLatitude)
   tmp1 = setLatitude(i);
   tmp2 = setLongitude(i);
   if sum(ismember(setLat, tmp1)) == 0 || sum(ismember(setLon, tmp2)) == 0
       setLat = [setLat; tmp1];
       setLon = [setLon; tmp2];
       toggleWP = [toggleWP; i];
   end
end


% Error metrics.
setLat = [latitude(1); setLat];
setLon = [longitude(1); setLon];
toggleWP = [toggleWP; 10000000000];

j=1;

error = zeros(size(latitude));

for i=1:length(setLatitude)
    if i == toggleWP(j)
        prevLatitude = setLat(j);
        prevLongitude = setLon(j);
        
        desiredLatitude = setLat(j+1);
        desiredLongitude = setLon(j+1);   
        
        j = j + 1;
    end
    error(i) = errorCalculate(prevLatitude, prevLongitude, latitude(i), longitude(i), desiredLatitude, desiredLongitude);
end


setLat(1) = [];
setLon(1) = [];	
% Plot 2D Plot of Latitude and Longitude.
figure(1);
plot(latitude, longitude, '-r'); hold on;
plot(setLat', setLon', 'xb');
xlabel('Latitude (Degrees)');
ylabel('Longitude (Degrees)');
pbaspect([1 1 1]);
hold on;
title('GPS Coordinate');

% Plot 2D Plot of x and y (in meters).
figure(2);
plot(x, y, '-r');  % Plot main trajectory.
hold on;

% Plot setpointLatitude points with crosses and circles.
for i = 1:length(setLat)
    [tmpX, tmpY, ~] = convertTo2DPlane(setLat(i), setLon(i), latitude(1), longitude(1));
    hold on;
    plot(tmpX, tmpY, 'xb');  % Plot crosses
    viscircles([tmpX, tmpY], 2.5, 'Color', 'b', 'LineWidth', 1, 'LineStyle', '--');  % Plot circles.
end

xlabel('North (meters)');
ylabel('East (meters)');
title('Cartesian Coordinate');
pbaspect([1 1 1]);

figure(3);
plot(time, bearing, '-r'); hold on;
plot(time, heading, '-b');
legend('Bearing', 'Heading')
xlabel('Time (Seconds)');
ylabel('Angles (Degree)');
title('Angles (Degree)');
pbaspect([1 1 1]);

figure(4);
plot(time, angleDiff, '-r');
xlabel('Time (Seconds)');
ylabel('Angle Difference (Degree)');
pbaspect([1 1 1]);
title('Angle Difference (Degree)');

figure(5);
plot(time, leftMotor, '-r');
xlabel('Time (Seconds)');
ylabel('Left Motor Speed');
title('Left Motor Speed');
pbaspect([1 1 1]);

figure(6);
plot(time, rightMotor, '-r');
xlabel('Time (Seconds)');
ylabel('Right Motor Speed');
title('Right Motor Speed');
pbaspect([1 1 1]);

figure(7);
plot(time, distance, '-r');
xlabel('Time (Seconds)');
ylabel('Distance (m)');
title('Distance against time');
pbaspect([1 1 1]);

figure(8);
plot(time, error, '-r');
xlabel('Time (Seconds)');
ylabel('Error (m)');
title('Error against time');
pbaspect([1 1 1]);
% /////////////////////////////////////////////////////////////////////////







