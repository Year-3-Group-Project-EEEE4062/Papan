% /////////////////////////////////////////////////////////////////////////
% Analyse collected data.
% /////////////////////////////////////////////////////////////////////////


clc; clear all; close all;


% /////////////////////////////////////////////////////////////////////////
% Load data.
% /////////////////////////////////////////////////////////////////////////
boat_data1 = readmatrix('boat_data1.csv'); 
boat_data2 = readmatrix('boat_data2.csv');
sine = readmatrix('sine.csv'); 
walk_home = readmatrix('walk_home.csv'); 
LakeBoundary = readmatrix('LakeBoundary.xlsx'); 
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Analyse data for lake.
% /////////////////////////////////////////////////////////////////////////
latitude1 = boat_data1(:, 3);
longitude1 = boat_data1(:, 4);
bearing1 = boat_data1(:, 5);

% Initialize arrays to store converted coordinates.
x = zeros(size(latitude1));
y = zeros(size(longitude1));

% Convert latitude and longitude to meters.
for i = 2:length(latitude1)
    [x(i), y(i), ~] = convertTo2DPlane(latitude1(i), longitude1(i), latitude1(1), longitude1(1));
end

figure(1);

% Plot 2D Plot of Latitude and Longitude.
subplot(2,1,1);
plot(longitude1, latitude1, '-r');
pbaspect([1 1 1]);
xlabel('Longitude');
ylabel('Latitude');
title('2D Plot of Latitude and Longitude');

% Plot 2D Plot of x and y (in meters).
subplot(2,1,2);
plot(y, x, '-r');
pbaspect([1 1 1]);
xlabel('Y (meters)');
ylabel('X (meters)');
title('2D Plot of x and y (in meters)');

figure(2);
x = 1:length(bearing1);
plot(x, bearing1, '-r');
xlabel('Number of Samples');
ylabel('Heading (Degree)');
title('Heading (Degree)');
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Analyse data for lake.
% /////////////////////////////////////////////////////////////////////////
latitude2 = boat_data2(:, 3);
longitude2 = boat_data2(:, 4);
bearing2 = boat_data2(:, 5);

% Initialize arrays to store converted coordinates.
x = zeros(size(latitude2));
y = zeros(size(longitude2));

% Convert latitude and longitude to meters.
for i = 2:length(latitude2)
    [x(i), y(i), ~] = convertTo2DPlane(latitude2(i), longitude2(i), latitude2(1), longitude2(1));
end

figure(3);

% Plot 2D Plot of Latitude and Longitude.
subplot(2,1,1);
plot(longitude2, latitude2, '-r');
pbaspect([1 1 1]);
xlabel('Longitude');
ylabel('Latitude');
title('2D Plot of Latitude and Longitude');

% Plot 2D Plot of x and y (in meters).
subplot(2,1,2);
plot(y, x, '-r');
pbaspect([1 1 1]);
xlabel('Y (meters)');
ylabel('X (meters)');
title('2D Plot of x and y (in meters)');

figure(4);
x = 1:length(bearing2);
plot(x, bearing2, '-r');
xlabel('Number of Samples');
ylabel('Heading (Degree)');
title('Heading (Degree)');
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Analyse data for lake.
% /////////////////////////////////////////////////////////////////////////
latitude3 = sine(:, 3);
longitude3 = sine(:, 4);
bearing3 = sine(:, 5);

% Initialize arrays to store converted coordinates.
x = zeros(size(latitude3));
y = zeros(size(longitude3));

% Convert latitude and longitude to meters.
for i = 2:length(latitude3)
    [x(i), y(i), ~] = convertTo2DPlane(latitude3(i), longitude3(i), latitude3(1), longitude3(1));
end

figure(4);

% Plot 2D Plot of Latitude and Longitude.
subplot(2,1,1);
plot(longitude3, latitude3, '-r');
pbaspect([1 1 1]);
xlabel('Longitude');
ylabel('Latitude');
title('2D Plot of Latitude and Longitude');

% Plot 2D Plot of x and y (in meters).
subplot(2,1,2);
plot(y, x, '-r');
pbaspect([1 1 1]);
xlabel('Y (meters)');
ylabel('X (meters)');
title('2D Plot of x and y (in meters)');

figure(5);
x = 1:length(bearing3);
plot(x, bearing3, '-r');
xlabel('Number of Samples');
ylabel('Heading (Degree)');
title('Heading (Degree)');
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Comparison.
% /////////////////////////////////////////////////////////////////////////
figure(6);
plot(longitude1, latitude1, '-r', longitude2, latitude2, '-b', longitude3, latitude3, '-g');
pbaspect([1 1 1]);
legend('Reading 1', 'Reading 2', 'Reading 3')
xlabel('Longitude');
ylabel('Latitude');
title('Comparison for 2D Plot of Latitude and Longitude');
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Lake boundary.
% /////////////////////////////////////////////////////////////////////////
figure(7);
lat = boat_data1(:, 3);
lon = boat_data1(:, 4);

plot(lon, lat, '-r');
pbaspect([1 1 1]);
xlabel('Longitude');
ylabel('Latitude');
title('2D Plot of Latitude and Longitude');
% /////////////////////////////////////////////////////////////////////////


function [x, y, bearing, distance] = convertTo2DPlane(lat2, lon2, reference_lat, reference_lon)
    % Earth radius in meters.
    EARTH_RADIUS = 6371000.0;

    % Convert degrees to radians.
    reference_lat = deg2rad(reference_lat);
    reference_lon = deg2rad(reference_lon);
    lat2 = deg2rad(lat2);
    lon2 = deg2rad(lon2);

    % Calculate differences in coordinates.
    deltaLat = lat2 - reference_lat;
    deltaLon = lon2 - reference_lon;

    % Calculate distance between the two points.
    a = sin(deltaLat / 2)^2 + cos(reference_lat) * cos(lat2) * sin(deltaLon / 2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    distance = EARTH_RADIUS * c;

    % Calculate bearing.
    angle = atan2(sin(deltaLon) * cos(lat2), cos(reference_lat) * sin(lat2) - sin(reference_lat) * cos(lat2) * cos(deltaLon));
    bearing = rad2deg(angle);

    % Convert distance and bearing to x-y coordinates.
    x = distance * cos(angle);
    y = distance * sin(angle);
end