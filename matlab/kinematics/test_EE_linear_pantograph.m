clc;
clear;
close all;

%% === LOAD ROBOT ===
robot = Robot_params();

%% === LINEAR TRAJECTORY SETTINGS ===
total_time = 5;

q_start = [-50; 180];   % mm
q_final = [ 50; 180];   % mm

%% === GENERATE EE TRAJECTORY ===
[time_series, EE_path] = EE_linear_traj(total_time, q_start, q_final);

%% === COMPUTE ACTUATOR TRAJECTORY USING IPK ===
theta_path = zeros(2, length(time_series));

for k = 1:length(time_series)

    q = EE_path(:, k);

    theta_path(:, k) = IPK_pantograph(q, robot);

end

%% === 3D SIMULATION USING YOUR DRAW FUNCTION ===
figure;

for k = 1:length(time_series)

    q = EE_path(:, k);
    theta = theta_path(:, k);

    clf;

    draw_pantograph_3D(q, theta, robot);

    hold on;

    %% === TRAJECTORY HEIGHT ===
    z_traj = 30;

    %% === DRAW FULL DESIRED STRAIGHT LINE ===
    plot3(EE_path(1,:), ...
          EE_path(2,:), ...
          z_traj*ones(1,length(time_series)), ...
          'Color', [1 0 0], ...
          'LineStyle', '--', ...
          'LineWidth', 5);

    %% === DRAW TRAVELLED PATH ===
    plot3(EE_path(1,1:k), ...
          EE_path(2,1:k), ...
          z_traj*ones(1,k), ...
          'Color', [0 0 1], ...
          'LineWidth', 6);

    title('Pantograph Linear Trajectory Simulation');

    drawnow;
    pause(0.02);

end

%% === PLOT ACTUATOR ANGLES ===
figure;

plot(time_series, rad2deg(theta_path(1,:)), 'LineWidth', 2);
hold on;
plot(time_series, rad2deg(theta_path(2,:)), 'LineWidth', 2);

grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Pantograph Actuator Angles Along Linear Trajectory');
legend('\theta_1', '\theta_2');