% /////////////////////////////////////////////////////////////////////////
% Analyse collected data.
% /////////////////////////////////////////////////////////////////////////


clc; clear all; close all;


% /////////////////////////////////////////////////////////////////////////
% Load data.
% /////////////////////////////////////////////////////////////////////////
data = readmatrix('boatFriday2.csv'); 
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Analyse data for lake.
% /////////////////////////////////////////////////////////////////////////
latitude = data(:, 3);
longitude = data(:, 4);
setLatitude = data(:, 5);
setLongitude = data(:, 6);
leftMotor = data(:, 10)*25;
rightMotor = data(:, 11)*25;

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
for i=1:length(setLatitude)
   tmp1 = setLatitude(i);
   tmp2 = setLongitude(i);
   if sum(ismember(setLat, tmp1)) == 0 || sum(ismember(setLon, tmp2)) == 0
       setLat = [setLat; tmp1];
       setLon = [setLon; tmp2];
       toggleWP = [toggleWP; i];
   end
end


% Plot 2D Plot of Latitude and Longitude.
figure(1);
plot(longitude, latitude, 'xr'); hold on;
plot(setLon', setLat', 'xb');
pbaspect([1 1 1]);
xlabel('Longitude');
ylabel('Latitude');
title('GPS Coordinate');

% Plot 2D Plot of x and y (in meters).
figure(2);
plot(y, x, '-r');  % Plot main trajectory.
hold on;

% Plot Initial Latitude points with crosses and circles.
hold on;
plot(y(1), x(1), 'xb');  % Plot crosses
handles = viscircles([y(1), x(1)], 1.5, 'Color', 'b', 'LineWidth', 1, 'LineStyle', '--');  % Plot circles.

xlabel('East (meters)');
ylabel('North (meters)');
pbaspect([1 1 1]);
title('Cartesian Coordinate');

labels = sprintf('1.5m Boundary'); % Store label for legend.


% Add legend with labeled circles.
legend(handles, labels);

% Define the coordinates of the origin
x_origin = x(1);
y_origin = y(1);

% Label the origin with text
text(x_origin, y_origin, 'Starting Point', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top')

figure(3);
x= 1:length(latitude);
plot(x, leftMotor, '-r');
xlabel('Number of Samples');
ylabel('Left Motor Speed (%)');
title('Left Motor Speed');

figure(4);
plot(x, rightMotor, '-r');
xlabel('Number of Samples');
ylabel('Right Motor Speed (%)');
title('Right Motor Speed');

trigger = zeros(size(distance));
for i=1:length(distance)
    if (distance(i) > 1.5)
        trigger(i) = 1;
    else
        trigger(i) = 0;
    end
end

figure(5);
% Plot the first subplot (distance against time).
subplot(2,1,1); % Create a subplot grid of 2 rows and 1 column, and select the first subplot.
plot(x, distance, '-r');
xlabel('Number of Samples');
ylabel('Distance (m)');
title('Distance against Time');

% Plot the second subplot (trigger against time).
subplot(2,1,2); % Select the second subplot.
plot(x, trigger, '-b');
xlabel('Number of Samples');
ylabel('Logic Signal (Boolean)');
ylim([0 1.1])
title('Logic Signal against Time');
% /////////////////////////////////////////////////////////////////////////