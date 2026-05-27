% ────────────────────────────────────────────────────────────────────────%
%                     ┬─┐┌─┐┌┐ ┌─┐┌┬┐  ┬  ┌─┐┌┐                           %
%                     ├┬┘│ │├┴┐│ │ │   │  ├─┤├┴┐                          %
%                     ┴└─└─┘└─┘└─┘ ┴   ┴─┘┴ ┴└─┘                          %
% ────────────────────────────────────────────────────────────────────────%
% Project Title: TurtleBot Simulation
% Author: Juan Lopez Muro, Shreya Srikanth
% Date: 01-31-2025
% Version 1.1
% Description: In this class, we will apply the concepts studied to an
% example. In the first stage, we will work in a simulated environment.
% Later, we will work on an actual experiment in the lab.
%
% The following files need to be all in the same folder, for this mainStd.m script to run:
%                 mainStd.m
%                 Turtlebot.p
%                 way_points_ext.txt
%                 way_points_int.txt
%                 way_points_mid.txt
%% Clear workspace and command window
close all;
clear;
clc;
% Before going further, scan the whole script to get an idea of what we'll do!
%% Section 0: Data Preparation
obj = Turtlebot();   % Create a Turtlebot object! I named it "obj" because
% I am boring, you can call it something else :). Do not worry about it, it will just simulate the robot dynamics.
% Set the initial position and orientation. This is the location where you want to place the robor. For now, set them to zero!
x0 = 0;     %-2;    %<-- use later!
y0 = 0;     %-0.8750;   %<-- use later!
psi0 = 0;   %pi/2;    %<-- use later!
obj.set_states(x0, y0, psi0)
% Set the sampling time TS we'll use in this simulation, 0.01 will do!
obj.set_TS(0.01);

%% Defining global integral and derivative helper variables and saturation limits
global x_error;
global y_error;
global yaw_error;
global linear_speed_sat;
global angular_speed_sat;
global net_count;
global sample_queue_x;
global sample_queue_y;
global sample_queue_yaw;
global x_error_sum;
global y_error_sum;
global yaw_error_sum;
global linear_distance;
global angular_distance;

% Initialize global variables
x_error = 0;
y_error = 0;
yaw_error = 0;
net_count = 0;
sample_queue_x = {};
sample_queue_y = {};
sample_queue_yaw = {};
x_error_sum = 0;
y_error_sum = 0;
yaw_error_sum = 0;
linear_distance = 0;
angular_distance = 0;

% Initialize global saturation limits from the Turtlebot object
linear_speed_sat = obj.linear_speed_sat;
angular_speed_sat = obj.angular_speed_sat;

% dt = 0.01s because frequency is 100 Hz
%% Section 1: Reference trajectory generation
% Create a time column vector, starting from 0s, up to 10s, in TS steps.
Time = [0:obj.TS:10]';
% Create x and y reference position column vectors, of the same size as the time vector.
% This is an example of a step function, a good start.You can get creative!
xd = 1 * ones(size(Time));            % x position
yd = 0 * ones(size(Time));         % y position
yawd = (pi/4) * ones(size(Time));

% Or you can use a fucntion, just like this!
% [xr, yr] = trajectory(Time);    % Uncomment me to use!
writeTrajectory(Time, xd, yd, yawd); % This function needs to be used to create a
% trajectory file for the real robot. Not need to use it now, but does not hurt either.
traj = importdata('myTrajectory.txt').data;
Time = traj(:,1);
xd   = traj(:,4);
yd   = traj(:,5);
yawd = traj(:,6);
% This line modifies the trajectory after import.

% We can also plot the trajectory to see what it looks like! Is it a step?
ax_traj = plotTrajectory(Time, xd, yd, yawd, xd, yd, yawd);
title(ax_traj(1), 'Reference Trajectory Yaw'); % Title for the first subplot (yaw)
title(ax_traj(2), 'Reference Trajectory X');   % Title for the second subplot (x)
title(ax_traj(3), 'Reference Trajectory Y');   % Title for the third subplot (y)

%% Section 2: Simulation Loop
% We are going to store importan variables to ve able to plot them later in
% a data matrix: [ u1, u2, x, y, psi, vx, vy, v, wz]
data = zeros(length(Time), 9);
% This is the main simulation loop.
stopSimulation = false; % simuation flag
i = 0;                  % loop index

x_accept = 0.01;
y_accept = 0.01;
yaw_accept = 0.11;
max_sim_steps = 1000;

while not(stopSimulation)
    i = i + 1;
    % Reference at each time step:
    x_d= xd(i);
    y_d= yd(i);
    yaw_d = yawd(i);
    % Get the information from the sensors!
    [x_sensor, y_sensor, psi_sensor, vx_sensor, vy_sensor, omega_sensor] = obj.get_perfectSensor();
    % Yes, this is a "perfect" sensor that gives us true values. In real
    % life, these values are noisy. We can add some white noise to the
    % measuremnts!
    %     psi_sensor     =   psi_sensor    +  (-0.5 + rand(1,1))*10^-2;
    % Control: This is the importat part where we actually have to work.
    % Let's implement the control algorithm in a separate funtion.
    [u] = controller(x_sensor, y_sensor, psi_sensor, x_d, y_d, yaw_d);
    data(i,:) = obj.get_log(); %[ u1, u2, x, y, psi, vx, vy, v, wz]
    % Bot simulation
    obj.move(u);            % Do not modify

    % Stop Simulation Condition: Check if errors are within tolerance
    current_x_error = abs(x_d - x_sensor);
    current_y_error = abs(y_d - y_sensor);
    % Normalize yaw error to [-pi, pi] for correct comparison
    current_yaw_error = abs(atan2(sin(yaw_d - psi_sensor), cos(yaw_d - psi_sensor)));

    % solving for the integrals of errors
    x_error_sum = x_error_sum + abs(current_x_error * 0.01);
    y_error_sum = y_error_sum + abs(current_y_error * 0.01);
    yaw_error_sum = yaw_error_sum + abs(current_yaw_error * 0.01);

    %if current_x_error <= x_accept && current_y_error <= y_accept && current_yaw_error <= yaw_accept
    %    stopSimulation = true;
    %    fprintf('Simulation point stopped: Target reached at time step %d.\n', i);
    %end

    % Safety net: Stop if maximum simulation steps reached
    if i >= max_sim_steps
        stopSimulation = true;
        fprintf('Simulation stopped: Maximum time steps (%d) reached without convergence.\n', max_sim_steps);
    end
end
%% Section 3: Plot some results!
ax_robot_traj = plotTrajectory(Time(1:i), data(1:i,3), data(1:i,4), data(1:i,5), xd(1:i), yd(1:i), yawd(1:i));
title(ax_robot_traj(1), 'Robot Trajectory Yaw', 'FontSize', 14); 
title(ax_robot_traj(2), 'Robot Trajectory X','FontSize', 14); 
title(ax_robot_traj(3), 'Robot Trajectory Y','FontSize', 14); 

plotOutputs(obj, Time(1:i), data(1:i,:), xd(1:i), yd(1:i), yawd(1:i))
writeTrajectory(Time(1:i), xd(1:i), yd(1:i), yawd(1:i))

fprintf('\nTotal accumulated x error integral: %d.\n', x_error_sum);
fprintf('Total accumulated y error integral: %d.\n', y_error_sum);
fprintf('Total accumulated yaw error integral: %d.\n\n', yaw_error_sum);

fprintf('Total distance traveled: %d.\n', linear_distance);
fprintf('Total angular distance traveled: %d.\n\n', angular_distance);

%% Function to be coded:
function [u] = controller(x, y, psi, x_d, y_d, yaw_d)
global x_error;
global y_error;
global yaw_error;
global linear_speed_sat; % Declare global saturation limits
global angular_speed_sat; % Declare global saturation limits
global net_count; % number of samples tracked
global sample_queue_x;
global sample_queue_y;
global sample_queue_yaw;
global linear_distance;
global angular_distance;
global x_error_sum;
global y_error_sum;
global yaw_error_sum;

% Position control
% Step 1: Start implementing a simple control in x direction only and simulate.
KPx = 1.5;
KPy = 1;
KPyaw = 0.99; %pi/4.75;

KIx = 0.5;
KIy = 0.5;
KIyaw = 0.12;

KDx = 0.1; %10;
KDy = 0.1; %10;
KDyaw = 0.02; %pi;

dt = 0.01; % Sampling time, equivalent to obj.TS

x_e=x_d-x;
y_e=y_d-y;

max_samples = 25; % TUNABLE
% Updating queue
sample_queue_x{end+1} = x_e;
sample_queue_y{end+1} = y_e;
net_count = net_count + 1;
if net_count > max_samples
    sample_queue_x(1) = [];
    sample_queue_y(1) = [];
end

% Update integral terms
numeric_x_samples = cell2mat(sample_queue_x);
x_e_integral = sum(numeric_x_samples) * dt;
numeric_y_samples = cell2mat(sample_queue_y);
y_e_integral = sum(numeric_y_samples) * dt;

% Calculate derivative terms
x_e_derivative = (x_e - x_error) / dt;
y_e_derivative = (y_e - y_error) / dt;

% Store current errors for next derivative calculation
x_error = x_e;
y_error = y_e;

% Calculate linear velocities in x and y
v_x=x_e*KPx + x_e_integral*KIx + x_e_derivative*KDx;
v_y=y_e*KPy + y_e_integral*KIy + y_e_derivative*KDy;

% Attitude control
yaw = atan2(v_y,v_x); % desired yaw
% TDL: FINISH COMPLEX CONTROLS FOR THIS ACCOUNTING FOR YAW FROM SPEED AND
% DESIRED YAW
v = sqrt(v_y^2 + v_x^2); % Complete this yourself
v = max(-linear_speed_sat, min(v, linear_speed_sat));
% Don't update integral if the speed is at saturation limit
if v==-linear_speed_sat | v==linear_speed_sat
    sample_queue_x{end} = 0;
    sample_queue_y{end} = 0;
    v_x=x_e*KPx + x_e_derivative*KDx;
    v_y=y_e*KPy + y_e_derivative*KDy;
    yaw = atan2(v_y,v_x);
    v = sqrt(v_y^2 + v_x^2); % Complete this yourself
    v = max(-linear_speed_sat, min(v, linear_speed_sat));
end
if v < 0.02 % TUNABLE VALUE/THRESHOLD FOR FINAL VELOCITY
    yaw = yaw_d; % default to final angle at end of path
end

% Step 4: Avoid "u" turns by keeping the |error| less that pi rad
yaw_e = yaw-psi; % Yaw error: Complete this yourself
yaw_e = atan2(sin(yaw_e), cos(yaw_e)); % Normalize angle to [-pi, pi]

% Update integral term for yaw
% Updating queue
sample_queue_yaw{end+1} = yaw_e;
if net_count > max_samples
    sample_queue_yaw(1) = [];
end
numeric_yaw_samples = cell2mat(sample_queue_yaw);
yaw_e_integral = sum(numeric_yaw_samples) * dt;

% Calculate derivative term for yaw
yaw_e_derivative = (yaw_e - yaw_error) / dt;

% Store current yaw error for next derivative calculation
yaw_error = yaw_e;

omg = KPyaw*yaw_e + KIyaw*yaw_e_integral + yaw_e_derivative*KDyaw;
omg = max(-angular_speed_sat, min(omg, angular_speed_sat));
if omg==-angular_speed_sat | omg==angular_speed_sat
    sample_queue_yaw{end} = 0;
    omg=x_e*KPx + x_e_derivative*KDx;
    omg = max(-angular_speed_sat, min(omg, angular_speed_sat));
end

% Step 5: Avoid actuation saturation keeping control signals bounded
% -u1sat < u1 < +u1sat
% -u2sat < u1 < +u2sat

% Output Control Signal
[u] = [v, omg];

% solving for the integrals of errors
x_error_sum = x_error_sum + abs((x_d - x) * 0.01);
y_error_sum = y_error_sum + abs((y_d - y) * 0.01);
yaw_error_sum = yaw_error_sum + abs((yaw_d - psi) * 0.01);

linear_distance = linear_distance + abs(v) * 0.01;
angular_distance = angular_distance + abs(omg) * 0.01;
end

function ax = plotTrajectory(Time, xr, yr, yawr, xd, yd, yawd)
%PLOTTRAJECTORY Plot some outputs!
    fig = figure;

    % yaw plot
    ax(1) = subplot(3,1,1);
    hold(ax(1), "on"), grid(ax(1), "on")
    xlabel(ax(1), 'Time (s)', 'FontSize', 14)
    ylabel(ax(1), 'Yaw (rad)', 'FontSize', 14)
    plot(ax(1), Time, yawr, 'k', 'DisplayName', 'Robot Yaw')
    plot(ax(1), Time, yawd, 'b--', 'DisplayName', 'Target Yaw');
    legend(ax(1), 'Location', 'best')
    ylim(ax(1), [-3.5, 3.5]); % Apply the fixed Y-axis limits

    % x plot
    ax(2) = subplot(3,1,2);
    hold(ax(2), "on"), grid(ax(2), "on")
    xlabel(ax(2), 'Time (s)', 'FontSize', 14)
    ylabel(ax(2), 'X Position (m)', 'FontSize', 14)
    plot(ax(2), Time, xr, 'k', 'DisplayName', 'Robot X Position')
    plot(ax(2), Time, xd, 'b--', 'DisplayName', 'Target X Position');
    legend(ax(2), 'Location', 'best')
    ylim(ax(2), [-3, 3]);

    % y plot
    ax(3) = subplot(3,1,3);
    hold(ax(3), "on"), grid(ax(3), "on")
    xlabel(ax(3), 'Time (s)', 'FontSize', 14)
    ylabel(ax(3), 'Y Position (m)', 'FontSize', 14)
    plot(ax(3), Time, yr, 'k', 'DisplayName', 'Robot Y Position')
    plot(ax(3), Time, yd, 'b--', 'DisplayName', 'Target Y Position');
    legend(ax(3), 'Location', 'best')
    ylim(ax(3), [-3, 3]);

    % Link the x-axes of all subplots
    linkaxes(ax, 'x');

    sgtitle('Enhanced PID', 'FontSize', 20)
end

function plotOutputs(obj, Time, data, xr, yr, yawr)
%PLOTUTPUTS Plot some outputs!
    fig = figure;

    % Subplot for X error
    ax1 = subplot(4,1,1);
    hold(ax1, "on"), grid(ax1, "on")
    xlabel(ax1, 'Time (s)')
    ylabel(ax1, 'ex (m)')
    plot(ax1, Time, xr-data(:,3), 'k')

    % Subplot for Y error
    ax2 = subplot(4,1,2);
    hold(ax2, "on"), grid(ax2, "on")
    xlabel(ax2, 'Time (s)')
    ylabel(ax2, 'ey (m)')
    plot(ax2, Time, yr-data(:,4), 'k')

    % Subplot for Linear Velocity
    ax3 = subplot(4,1,3);
    hold(ax3, "on"), grid(ax3, "on")
    xlabel(ax3, 'Time (s)')
    ylabel(ax3, 'v (m/s)')
    plot(ax3, Time, data(:,1), 'k') % Command linear speed
    plot(ax3, Time, data(:,8), '--r') % Real linear speed
    yline(ax3, obj.linear_speed_sat,'--g', 'Saturation Limit')
    yline(ax3, -obj.linear_speed_sat,'--g')
    legend(ax3, 'cmd','real','sat', 'Location', 'best')

    % Subplot for Angular Velocity
    ax4 = subplot(4,1,4);
    hold(ax4, "on"), grid(ax4, "on")
    xlabel(ax4, 'Time (s)')
    ylabel(ax4, 'wz (rad/s)')
    plot(ax4, Time, data(:,2), 'k') % Command angular speed
    plot(ax4, Time, data(:,9), '--r') % Real angular speed
    yline(ax4, obj.angular_speed_sat,'--g', 'Saturation Limit')
    yline(ax4, -obj.angular_speed_sat,'--g')
    legend(ax4,'cmd','real','sat', 'Location', 'best')
end

%% Write files
function writeTrajectory(Time, xr, yr, yawr)
%WRITETRAJECTORY This file formats your trajectroy corrctly for the robot
% Sample data to write
saveData = zeros(length(Time),10);
saveData(:,1) = Time;
saveData(:,4) = xr;
saveData(:,5) = yr;
saveData(:,6) = yawr;
% Open a text file for writing
fid = fopen('myTrajectory.txt', 'w');
% Check if the file was successfully opened
if fid == -1
    error('Could not open the file for writing.');
end
% Write header
header = 'T (s)\tLin\tAng\tX\tY\tyaw\tdx\tdy\tv\twz\n';
fprintf(fid, header);
% Write data to the text file
for i = 1:length(Time)
    fprintf(fid, '%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n', saveData(i,:));
end
% Close the file
fclose(fid);
end