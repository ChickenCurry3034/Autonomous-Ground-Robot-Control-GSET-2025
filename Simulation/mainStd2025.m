function mainStd()
% ────────────────────────────────────────────────────────────────────────%
%                     ┬─┐┌─┐┌┐ ┌─┐┌┬┐  ┬  ┌─┐┌┐                           %
%                     ├┬┘│ │├┴┐│ │ │   │  ├─┤├┴┐                          %
%                     ┴└─└─┘└─┘└─┘ ┴   ┴─┘┴ ┴└─┘                          %
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
%                 mainStd.m
%                 Turtlebot.p
%                 way_points_ext.txt
%                 way_points_int.txt
%                 way_points_mid.txt
%% Clear workspace and command window
close all;
clear;
clc;
% Before going further, scan the whole script to get an idea of what we'll do!
%% Section 0: Data Preparation
obj = Turtlebot(); % Create a Turtlebot object! I named it "obj" because
% I am boring, you can call it something else :). Do not worry about it, it will just simulate the robot dynamics.
% Set the initial position and orientation. This is the location where you want to place the robor. For now, set them to zero!
x0 = 0; %-2;    %<-- use later!
y0 = 0; %-0.8750;   %<-- use later!
psi0 = 0; %pi/2;    %<-- use later!
obj.set_states(x0, y0, psi0)
% Set the sampling time TS we'll use in this simulation, 0.01 will do!
obj.set_TS(0.01);
%% Defining global integral and derivative helper variables and saturation limits
% IMPORTANT: Global variables declared within a function (like mainStd)
% are still global. However, it's generally good practice to minimize
% the use of globals. For local functions, if you want them to share data
% without passing them as arguments, nested functions are often a better
% choice, or passing the data as arguments. For now, globals will work.
global x_error;
global y_error;
global yaw_error;
global linear_speed_sat;
global angular_speed_sat;
global net_count;
global sample_queue_x;
global sample_queue_y;
global sample_queue_yaw;
% Initialize global variables
x_error = 0;
y_error = 0;
yaw_error = 0;
net_count = 0;
sample_queue_x = {};
sample_queue_y = {};
sample_queue_yaw = {};
% Initialize global saturation limits from the Turtlebot object
linear_speed_sat = obj.linear_speed_sat;
angular_speed_sat = obj.angular_speed_sat;
% dt = 0.01s because frequency is 100 Hz
%% Section 1: Reference trajectory generation
% Create initial time and reference vectors. These define the *desired path*.
% They will be extended dynamically in the simulation loop if the simulation
% runs longer than this initial trajectory.
initialTime = [0:obj.TS:10]';
initialXd = 1 * ones(size(initialTime));
initialYd = 0 * ones(size(initialTime));
initialYawd = (pi/4) * ones(size(initialTime));

% Write the initial trajectory to file.
writeTrajectory(initialTime, initialXd, initialYd, initialYawd);

% Import data from the generated trajectory file. These will be the
% reference points the robot tries to follow.
traj = importdata('myTrajectory.txt').data;
Time_ref = traj(:,1); % Renamed to avoid conflict with dynamic Time
Xd_ref = traj(:,4);   % Renamed to avoid conflict with dynamic Xd
Yd_ref = traj(:,5);   % Renamed to avoid conflict with dynamic Yd
Yawd_ref = traj(:,6); % Renamed to avoid conflict with dynamic Yawd

% Plot the initial reference trajectory.
ax_traj = plotTrajectory(Time_ref, Xd_ref, Yd_ref, Yawd_ref, Xd_ref, Yd_ref, Yawd_ref);
title(ax_traj(1), 'Initial Reference Trajectory Yaw');
title(ax_traj(2), 'Initial Reference Trajectory X');
title(ax_traj(3), 'Initial Reference Trajectory Y');

x_accept = 0.01;
y_accept = 0.01;
yaw_accept = 0.11;

%% Section 2: Simulation Loop
% We are going to store important variables to be able to plot them later in
% a data matrix: [ u1, u2, x, y, psi, vx, vy, v, wz]
data = []; % Initialize as empty, as length is no longer fixed by Time vector

% These vectors will store the actual time and reference values for the
% duration of the simulation, extending dynamically as needed.
simTime = [];
simXd = [];
simYd = [];
simYawd = [];

stopSimulation = false; % simulation flag
i = 0; % loop index
max_sim_steps = 3000; % Safety net to prevent infinite loops (e.g., 200 seconds at 0.01s TS)

nodes_map = containers.Map('KeyType', 'char', 'ValueType', 'any');
nodes_map('A') = [0, 0, 0];
nodes_map('B') = [1, 2, pi/4];
nodes_map('C') = [2, 3, pi/3];
nodes_map('D') = [3, 2, pi/2];
nodes_map('E') = [4, 1, -pi/6];
nodes_map('F') = [4, 2, -pi/3];
nodes_map('G') = [4, 3, pi/4];

connections_map = containers.Map('KeyType', 'char', 'ValueType', 'any');
connections_map('A') = {'B', 'C'};
connections_map('B') = {'A', 'C', 'D'};
connections_map('C') = {'A', 'B', 'D', 'G'};
connections_map('D') = {'B', 'C', 'E', 'F', 'G'};
connections_map('E') = {'D', 'F'};
connections_map('F') = {'D', 'G'};
connections_map('G') = {'C', 'F'};

start_node = 'A';
end_node = 'G';
best_path = dijkstra(nodes_map, connections_map, start_node, end_node);
fprintf('Calculated path from %s to %s: %s\n', start_node, end_node, strjoin(best_path, ' -> '));
path_length = length(best_path);
current_node = 1;
keys = nodes_map.keys();

while not(stopSimulation)
    i = i + 1;

    % Determine current desired reference point
    if i <= length(Time_ref) % Use the pre-defined trajectory if within its bounds
        key = keys{current_node};
        coordinates = nodes_map(key);
        current_x_d = coordinates(1);
        current_y_d = coordinates(2);
        current_yaw_d = coordinates(3);
    else % If simulation extends beyond the initial trajectory, hold the last target
        key = keys{current_node};
        coordinates = nodes_map(key);
        current_x_d = coordinates(1);
        current_y_d = coordinates(2);
        current_yaw_d = coordinates(3);
    end

    % Get the information from the sensors!
    [x_sensor, y_sensor, psi_sensor, vx_sensor, vy_sensor, omega_sensor] = obj.get_perfectSensor();
    % Yes, this is a "perfect" sensor that gives us true values. In real
    % life, these values are noisy. We can add some white noise to the
    % measuremnts!
    %     psi_sensor     =   psi_sensor    +  (-0.5 + rand(1,1))*10^-2;

    % Control: This is the important part where we actually have to work.
    % Let's implement the control algorithm in a separate function.
    [u] = controller(x_sensor, y_sensor, psi_sensor, current_x_d, current_y_d, current_yaw_d);

    % Store simulation data. MATLAB will automatically grow 'data'.
    data(i,:) = obj.get_log(); %[ u1, u2, x, y, psi, vx, vy, v, wz]

    % Append current time and reference to dynamically growing vectors for plotting
    simTime(i) = (i-1) * obj.TS; % Calculate current time based on step and TS
    simXd(i) = current_x_d;
    simYd(i) = current_y_d;
    simYawd(i) = current_yaw_d;

    % Bot simulation
    obj.move(u); % Do not modify

    % Stop Simulation Condition: Check if errors are within tolerance
    current_x_error = abs(current_x_d - x_sensor);
    current_y_error = abs(current_y_d - y_sensor);
    % Normalize yaw error to [-pi, pi] for correct comparison
    current_yaw_error = abs(atan2(sin(current_yaw_d - psi_sensor), cos(current_yaw_d - psi_sensor)));

    if current_x_error <= x_accept && current_y_error <= y_accept && current_yaw_error <= yaw_accept
        if(current_node >= path_length)
            stopSimulation = true;
        else
            current_node = current_node + 1;
        end
        fprintf('Simulation point stopped: Target reached at time step %d.\n', i);
    end

    % Safety net: Stop if maximum simulation steps reached
    if i >= max_sim_steps
        stopSimulation = true;
        fprintf('Simulation stopped: Maximum time steps (%d) reached without convergence.\n', max_sim_steps);
    end
end

% Assign the dynamically grown vectors to the original variable names
% for use in plotting and file writing functions.
Time = simTime'; % Transpose to make it a column vector
xd = simXd';
yd = simYd';
yawd = simYawd';
% data is already correctly sized by now

%% Section 3: Plot some results!
ax_robot_traj = plotTrajectory(Time, data(:,3), data(:,4), data(:,5), xd, yd, yawd);
title(ax_robot_traj(1), 'Robot Trajectory Yaw'); % Title for the first subplot (yaw)
title(ax_robot_traj(2), 'Robot Trajectory X'); % Title for the second subplot (x)
title(ax_robot_traj(3), 'Robot Trajectory Y'); % Title for the third subplot (y)

plotOutputs(obj, Time, data, xd, yd, yawd)
writeTrajectory(Time, xd, yd, yawd)

end % <--- This ends the mainStd function

%% LOCAL FUNCTIONS (These functions are defined within the same file as mainStd)

function path = dijkstra(nodes, connections, start_node, end_node)

    distances = containers.Map('KeyType', 'char', 'ValueType', 'double');
    node_names = nodes.keys();
    for k = 1:length(node_names)
        distances(node_names{k}) = inf;
    end
    distances(start_node) = 0;

    unvisited = node_names;

    prev = containers.Map('KeyType', 'char', 'ValueType', 'char');

    while ~isempty(unvisited)
        min_distance_node = '';
        min_dist = inf;

        for k = 1:length(unvisited)
            node = unvisited{k};
            if distances.isKey(node) && distances(node) < min_dist
                min_dist = distances(node);
                min_distance_node = node;
            end
        end

        if isempty(min_distance_node) || distances(min_distance_node) == inf
            break;
        end

        current_node = min_distance_node;

        unvisited(strcmp(unvisited, current_node)) = [];

        if strcmp(current_node, end_node)
            break;
        end

        if connections.isKey(current_node)
            neighbors = connections(current_node);
            for k = 1:length(neighbors)
                neighbor = neighbors{k};

                if any(strcmp(unvisited, neighbor))
                    pos_current = nodes(current_node);
                    pos_neighbor = nodes(neighbor);
                    dist_to_neighbor = hypot(pos_neighbor(1) - pos_current(1), pos_neighbor(2) - pos_current(2));

                    alt = distances(current_node) + dist_to_neighbor;

                    if alt < distances(neighbor)
                        distances(neighbor) = alt;
                        prev(neighbor) = current_node;
                    end
                end
            end
        end
    end

    path = {};
    current = end_node;

    if strcmp(start_node, end_node)
        path = {start_node};
        return;
    end

    if ~prev.isKey(end_node)
        path = {}; % No path found
        return;
    end

    while prev.isKey(current)
        path = [current, path];
        current = prev(current);
    end

    if strcmp(current, start_node)
        path = [start_node, path];
    else
        path = {};
    end

end

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

% Position control
% Step 1: Start implementing a simple control in x direction only and simulate.
KPx = 1.5;
KPy = 1;
KPyaw = 1.2; %pi/4.75;

KIx = 0.5;
KIy = 0.5;
KIyaw = 0.3;

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
if v==-linear_speed_sat || v==linear_speed_sat % Corrected logical OR
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
if omg==-angular_speed_sat || omg==angular_speed_sat % Corrected logical OR
    sample_queue_yaw{end} = 0;
    % This line previously had a copy-paste error (KPx and KDx).
    % Corrected to use KPyaw and KDyaw for angular velocity control,
    % and removed the integral part when saturating to prevent windup.
    omg=KPyaw*yaw_e + yaw_e_derivative*KDyaw;
    omg = max(-angular_speed_sat, min(omg, angular_speed_sat));
end

% Step 5: Avoid actuation saturation keeping control signals bounded
% -u1sat < u1 < +u1sat
% -u2sat < u1 < +u2sat

% Output Control Signal
[u] = [v, omg];
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
    ylim(ax(1), [-3.5, 3.5]); % Apply the fixed Y-axis limits

    % x plot
    ax(2) = subplot(3,1,2);
    hold(ax(2), "on"), grid(ax(2), "on")
    xlabel(ax(2), 'Time (s)')
    ylabel(ax(2), 'X Position (m)')
    plot(ax(2), Time, xr, 'k', 'DisplayName', 'Robot X Position')
    plot(ax(2), Time, xd, 'b--', 'DisplayName', 'Target X Position');
    legend(ax(2), 'Location', 'best')
    ylim(ax(2), [-1, 5]);

    % y plot
    ax(3) = subplot(3,1,3);
    hold(ax(3), "on"), grid(ax(3), "on")
    xlabel(ax(3), 'Time (s)')
    ylabel(ax(3), 'Y Position (m)')
    plot(ax(3), Time, yr, 'k', 'DisplayName', 'Robot Y Position')
    plot(ax(3), Time, yd, 'b--', 'DisplayName', 'Target Y Position');
    legend(ax(3), 'Location', 'best')
    ylim(ax(3), [-1, 5]);

    sgtitle('Robot Trajectory')

    % Link the x-axes of all subplots
    linkaxes(ax, 'x');
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
