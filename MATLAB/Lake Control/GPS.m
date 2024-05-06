% /////////////////////////////////////////////////////////////////////////
% Analyse collected data.
% /////////////////////////////////////////////////////////////////////////


clc; clear all; close all;


% /////////////////////////////////////////////////////////////////////////
% Load data.
% /////////////////////////////////////////////////////////////////////////
data = readmatrix('autoMode9993.csv');
lakeBoundary = readmatrix('LakeBoundary.xlsx');
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Analyse data for lake.
% /////////////////////////////////////////////////////////////////////////
lakeBoundaryLat = lakeBoundary(:, 2);
lakeBoundaryLon = lakeBoundary(:, 3);

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
plot(longitude, latitude, '-r'); hold on;
plot(setLon', setLat', 'xb');
xlabel('Longitude (Degrees)');
ylabel('Latitude (Degrees)');
pbaspect([1 1 1]);
hold on;
plot(lakeBoundaryLon, lakeBoundaryLat, '-k');
title('GPS Coordinate');
% Label the origin with text
text(longitude(1), latitude(1), 'Starting Point', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom')
text(longitude(length(longitude)), latitude(length(latitude)), 'Ending Point', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top')


% Plot 2D Plot of x and y (in meters).
figure(2);
plot(y, x, '-r');  % Plot main trajectory.

hold on;

% Plot setpointLatitude points with crosses and circles.
for i = 1:length(setLat)
    [tmpX, tmpY, ~] = convertTo2DPlane(setLat(i), setLon(i), latitude(1), longitude(1));
    hold on;
    plot(tmpY, tmpX, 'xb');  % Plot crosses
    viscircles([tmpY, tmpX], 2.5, 'Color', 'b', 'LineWidth', 1, 'LineStyle', '--');  % Plot circles.
end

xlabel('East (meters)');
ylabel('North (meters)');
pbaspect([1 1 1]);
title('Cartesian Coordinate');
text(y(1), x(1), 'Starting Point', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom')
text(y(length(y)), x(length(x)), 'Ending Point', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top')



figure(3);
plot(time, bearing, '-r'); hold on;
plot(time, heading, '-b');
legend('Desired Heading', 'Actual Heading')
xlabel('Time (Seconds)');
ylabel('Angles (Degree)');
pbaspect([1 1 1]);
title('Angles (Degree)');

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
pbaspect([1 1 1]);
title('Left Motor Speed');

figure(6);
plot(time, rightMotor, '-r');
xlabel('Time (Seconds)');
ylabel('Right Motor Speed');
pbaspect([1 1 1]);
title('Right Motor Speed');

figure(7);
plot(time, distance, '-r');
xlabel('Time (Seconds)');
ylabel('Distance (m)');
pbaspect([1 1 1]);
title('Distance against time');

figure(8);
plot(time, error, '-r');
xlabel('Time (Seconds)');
ylabel('Error (m)');
pbaspect([1 1 1]);
title('Error against time');
% /////////////////////////////////////////////////////////////////////////