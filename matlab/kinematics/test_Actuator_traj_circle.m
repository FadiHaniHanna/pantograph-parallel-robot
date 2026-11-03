clc;
clear;
close all;

%LOAD ROBOT
robot = Robot_params();

%CIRCLE SETTINGS 
total_time = 6;

center = [0; 180];

radius = 40;

%GENERATE ACTUATOR TRAJECTORY
[time_series, actuator_path, EE_path] = ...
    Actuator_traj(total_time, center, radius, robot);

%FIGURE 
figure;

for k = 1:length(time_series)

    q = EE_path(:,k);

    theta = actuator_path(:,k);

    clf;

    draw_pantograph_3D(q, theta, robot);

    hold on;

    %TRAJECTORY HEIGHT 
    z_traj = 30;

    %DRAW FULL CIRCLE 
    plot3(EE_path(1,:), ...
          EE_path(2,:), ...
          z_traj*ones(1,length(time_series)), ...
          'r--', ...
          'LineWidth', 5);

    %DRAW TRAVELLED PATH
    plot3(EE_path(1,1:k), ...
          EE_path(2,1:k), ...
          z_traj*ones(1,k), ...
          'b', ...
          'LineWidth', 6);

    title('Pantograph Circular Trajectory');

    drawnow;

    pause(0.02);

end