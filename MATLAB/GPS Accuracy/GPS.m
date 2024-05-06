% /////////////////////////////////////////////////////////////////////////
% Analyse collected data.
% /////////////////////////////////////////////////////////////////////////


clc; clear all; close all;


% /////////////////////////////////////////////////////////////////////////
% Load data.
% /////////////////////////////////////////////////////////////////////////
gps = readmatrix('TablePurple.csv'); 
% /////////////////////////////////////////////////////////////////////////


% /////////////////////////////////////////////////////////////////////////
% Analyse data for lake.
% /////////////////////////////////////////////////////////////////////////
latitude = gps(:, 3);
longitude = gps(:, 4);

% Initialize arrays to store converted coordinates.
x = zeros(size(latitude));
y = zeros(size(longitude));

% Convert latitude and longitude to meters.
for i = 2:length(latitude)
    [x(i), y(i), ~] = convertTo2DPlane(latitude(i), longitude(i), latitude(1), longitude(1));
end
fprintf('Mean X = %.2f.\n', mean(x));
fprintf('Mean Y = %.2f.\n', mean(y));
x = x - mean(x);
y = y - mean(y);

D = sqrt(x.^2+y.^2);
r1 = 0;
r2 = 0;
r3 = 0;

for i = 1:length(D)
    if D(i) <= 2.5
        r3 = r3 + 1;
    elseif D(i) <= 5.0
        r2 = r2 + 1;
    elseif D(i) <= 7.5
        r1 = r1 + 1;
    end
end

fprintf('r1 = %d, r2 = %d, r3 = %d.\n', r1, r2, r3);
fprintf('r1 = %.2f, r2 = %.2f, r3 = %.2f.\n', r1/(r1+r2+r3), r2/(r1+r2+r3), r3/(r1+r2+r3));
maxD = max(D);
fprintf('Maximum distance from origin: %.2f.\n', maxD);
meanD = mean(D);
fprintf('Mean distance from origin: %.2f.\n', meanD);
fprintf('SDX: %.2f, SDY: %.2f.\n', sqrt(var(x)), sqrt(var(y)));
fprintf('Mean X: %.2f, Mean Y: %.2f.\n', mean(x), mean(y));

% Plot 2D Plot of Latitude and Longitude.
figure(1)
plot(longitude, latitude, 'xr');
pbaspect([1 1 1]);
xlabel('Longitude');
ylabel('Latitude');
title('GPS Coordinate');

% Plot 2D Plot of x and y (in meters).
figure(2);
plot(y, x, 'xr');
pbaspect([1 1 1]);
axis([-10 10 -10 10])
xlabel('East (meters)');
ylabel('North (meters)');
title('Cartesian Coordinate');

hold on; % To retain the current plot while adding the circle.

% Define the radius of the circle.
radius1 = 2.5;
radius2 = 5.0;
radius3 = 7.5;

% Define the center coordinates of the circle.
center_x = 0;
center_y = 0;

% Define the radius of the circles.
radii = [radius1, radius2, radius3];
colors = {'r', 'g', 'b'};
labels = cell(1, numel(radii)); % Initialize cell array for legend labels.
handles = zeros(1, numel(radii)); % Initialize array for circle handles.

for i = 1:numel(radii)
    handles(i) = viscircles([center_x, center_y], radii(i), 'Color', colors{i}, 'LineWidth', 2);
    labels{i} = sprintf('Radius %.1f', radii(i)); % Store label for legend.
end

% Add legend with labeled circles.
legend(handles, labels);

hold off; % Release the hold on the plot.
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