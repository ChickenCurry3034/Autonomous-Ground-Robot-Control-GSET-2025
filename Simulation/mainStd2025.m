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
psi0 = 0.5;   %pi/2;    %<-- use later!
obj.set_states(x0, y0, psi0)

% Set the sampling time TS we'll use in this simulation, 0.01 will do!
obj.set_TS(0.01);
%% Section 1: Reference trajectory generation
% Create a time column vector, starting from 0s, up to 10s, in TS steps.
Time = [0:obj.TS:10]';

% Create x and y reference position column vectors, of the same size as the time vector.
% This is an example of a step function, a good start.You can get creative!
xd = 1;            % x position
yd = 0;         % y position
yawd = pi/4;

% Or you can use a fucntion, just like this!
% [xr, yr] = trajectory(Time);    % Uncomment me to use!

writeTrajectory(Time, xd, yd, yawd); % This function needs to be used to create a
% trajectory file for the real robot. Not need to use it now, but does not hurt either.
traj = importdata('myTrajectory.txt').data;
Time = traj(:,1);
xd   = traj(:,4);
yd   = traj(:,5);
yawd = traj(:,6)
xd(1001:end)=2;

% We can also plot the trajectory to see what it looks like! Is it a step?
ax = plotTrajectory(Time, xd, yd, yawd, xd, yd, yawd);
title(ax, 'Reference Trajectory')
%% Section 2: Simulation Loop
% We are going to store importan variables to ve able to plot them later in
% a data matrix: [ u1, u2, x, y, psi, vx, vy, v, wz]
data = zeros(length(Time), 9);

% This is the main simulation loop.
stopSimulation = false; % simuation flag
i = 0;                  % loop index
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
    % Stop Simulation
    if i == length(Time)
        stopSimulation = true;
    end
end
%% Section 3: Plot some results!
ax = plotTrajectory(Time, data(:,3), data(:,4), data(:,5), xd, yd, yawd);
title(ax, 'Robot Trajectory')
plotOutputs(obj, Time, data, xd, yd, yawd)

writeTrajectory(Time, xd, yd, yawd)

%% Defining global integral and derivative helper variables
global x_e_integral;
global y_e_integral;
global yaw_e_integral;

global x_error;
global y_error;
global yaw_error;

% dt = 0.01s because frequency is 100 Hz
%% Function to be coded:
function [u] = controller(x, y, psi, x_d, y_d, yaw_d)

global x_e_integral;
global y_e_integral;
global yaw_e_integral;
global x_error;
global y_error;
global yaw_error;

% Position control
% Step 1: Start implementing a simple control in x direction only and simulate.

KPx = 8;
KPy = 0;
KPyaw = 0.35; %pi/4.75;

KIx = 0;
KIy = 0;
KIyaw = 0;

KDx = 0; %10;
KDy = 0; %10;
KDyaw = 0; %pi;

x_e=x_d-x;
y_e=y_d-y;

x_e_integral = x_e_integral + 0.01 * x_e;
y_e_integral = y_e_integral + 0.01 * y_e;

x_e_derivative = (x_e - x_error) / 0.01;
y_e_derivative = (y_e - y_error) / 0.01;

x_error = x_e;
y_error = y_e;

v_x=x_e*KPx + x_e_integral*KIx + x_e_derivative*KDx;
v_y=y_e*KPy + y_e_integral*KIy + y_e_derivative*KDy;

% Attitude control
yaw = atan2(v_y,v_x); % desired yaw
v = sqrt(v_y^2 + v_x^2); % Complete this yourself

% Step 4: Avoid "u" turns by keeping the |error| less that pi rad
yaw_e = yaw_d-psi; % Yaw error: Complete this yourself
yaw_e = atan2(sin(yaw_e), cos(yaw_e));
yaw_e_integral = yaw_e_integral + 0.01 * yaw_e;
yaw_e_derivative = (yaw_e - yaw_error) / 0.01;
yaw_error = yaw_e;
omg = KPyaw*yaw_e + KIyaw*yaw_e_integral + yaw_e_derivative*KDyaw; 

% Step 5: Avoid actuation saturation keeping control signals bounded
% -u1sat < u1 < +u1sat
% -u2sat < u1 < +u2sat
linear_speed_sat = v; % Constant value
angular_speed_sat = omg; % Constant value
...
% Complete the rest yourself
...
% Output Control Signal   
[u] = [v, omg];
end
%% Make some plots:
function [xr, yr, yawr] = trajectory(Time)
%TRAJECTORY Build your trajectory
%   This is an example
    xr = Time>1;
    yr = zeros(size(Time));
    yawr = Time>1;
end

function ax = plotTrajectory(Time, xr, yr, yawr, xd, yd, yawd)
%PLOTTRAJECTORY Plot some outputs!
    fig = figure;
    % yaw plot
    ax(1) = subplot(3,1,1);
    hold(ax(1), "on"), grid(ax(1), "on")
    xlabel(ax(1), 'Time (s)')
    ylabel(ax(1), 'Yaw (rad)')
    plot(ax(1), Time, yawr, 'k', 'DisplayName', 'Robot Yaw')
    plot(ax(1), Time, yawd, 'b--', 'DisplayName', 'Target Yaw');
    legend(ax(1), 'Location', 'best')

    % x plot
    ax(2) = subplot(3,1,2);
    hold(ax(2), "on"), grid(ax(2), "on")
    xlabel(ax(2), 'Time (s)')
    ylabel(ax(2), 'X Position (m)')
    plot(ax(2), Time, xr, 'k', 'DisplayName', 'Robot X Position')
    plot(ax(2), Time, xd, 'b--', 'DisplayName', 'Target X Position');
    legend(ax(2), 'Location', 'best')

    % y plot
    ax(3) = subplot(3,1,3);
    hold(ax(3), "on"), grid(ax(3), "on")
    xlabel(ax(3), 'Time (s)')
    ylabel(ax(3), 'Y Position (m)')
    plot(ax(3), Time, yr, 'k', 'DisplayName', 'Robot Y Position')
    plot(ax(3), Time, yd, 'b--', 'DisplayName', 'Target Y Position');
    legend(ax(3), 'Location', 'best')
end

function plotOutputs(obj, Time, data, xr, yr, yawr)
%PLOTUTPUTS Plot some outputs!
    fig = figure;
    ax = axes(fig);
    hold(ax, "on"), grid(ax, "on")
    xlabel(ax, 'Time (s)')
    ylabel(ax, 'ex (m)')
    plot(ax, Time, xr-data(:,3), 'k')
    subplot(4,1,1,ax)
    
    ax = axes(fig);
    hold(ax, "on"), grid(ax, "on")
    xlabel(ax, 'Time (s)')
    ylabel(ax, 'ey (m)')
    plot(ax, Time, yr-data(:,4), 'k')
    subplot(4,1,2,ax)
    
    ax = axes(fig);
    hold(ax, "on"), grid(ax, "on")
    xlabel(ax, 'Time (s)')
    ylabel(ax, 'v (m/s)')
    plot(ax, Time, data(:,1), 'k')
    plot(ax, Time, data(:,8), '--r')
    yline(obj.linear_speed_sat,'--g')
    yline(-obj.linear_speed_sat,'--g')
    legend(ax, 'cmd','real','sat')
    subplot(4,1,3,ax)
    
    ax = axes(fig);
    hold(ax, "on"), grid(ax, "on")
    xlabel(ax, 'Time (s)')
    ylabel(ax, 'wz (rad/s)')
    plot(ax, Time, data(:,2), 'k')
    plot(ax, Time, data(:,9), '--r')
    yline(obj.angular_speed_sat,'--g')
    yline(-obj.angular_speed_sat,'--g')
    legend(ax,'cmd','real','sat')
    subplot(4,1,4,ax)
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