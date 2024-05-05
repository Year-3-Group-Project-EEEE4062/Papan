% /////////////////////////////////////////////////////////////////////////
% Look up table for left and right motor.
% /////////////////////////////////////////////////////////////////////////


% Load data.
data = readmatrix('boat_data.csv'); % Load data from CSV file into a matrix.


t = data(:, 1); % Extract time values from the first column.
angle = data(:, 2); % Extract angle values from the second column.
left_speed = data(:, 3); % Extract left motor speed values from the third column.
right_speed = data(:, 4); % Extract right motor speed values from the fourth column.


% Plot the angle over time.
figure(1)
subplot(2,1,1); % Create the first subplot.
plot(t, angle)
title('Angle Over Time') % Title of the plot.
xlabel('Time') % Label for the x-axis.
ylabel('Angle') % Label for the y-axis.


% Plot the motor speeds over time.
subplot(2,1,2); % Create the second subplot.
plot(t, left_speed, t, right_speed)
title('Motor Speeds Over Time') % Title of the plot.
xlabel('Time') % Label for the x-axis.
ylabel('Speed') % Label for the y-axis.
legend('Left Motor Speed', 'Right Motor Speed') % Add legend.

