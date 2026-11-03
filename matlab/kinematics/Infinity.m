clc;
clear;
close all;

robot = Robot_params();

l1 = robot.l1;
l2 = robot.l2;
l3 = robot.l3;
l4 = robot.l4;
d  = robot.d;

theta1_min = deg2rad(20);
theta1_max = deg2rad(160);

theta2_min = deg2rad(20);
theta2_max = deg2rad(160);


x_values = -250:10:250;
y_values = -50:10:300;

workspace_x = [];
workspace_y = [];
theta_store = [];

for x = x_values
    for y = y_values

        q = [x; y];

        try
            theta = IPK_pantograph(q, robot);

            theta1 = theta(1);
            theta2 = theta(2);

            if theta1 >= theta1_min && theta1 <= theta1_max && ...
               theta2 >= theta2_min && theta2 <= theta2_max

                workspace_x(end+1) = x;
                workspace_y(end+1) = y;
                theta_store(:, end+1) = theta;

            end

        catch
            % unreachable point
        end
    end
end

disp("Reachable workspace points found:");
disp(length(workspace_x));

% Center the infinity sign in the middle of your workspace
cx = mean([min(workspace_x), max(workspace_x)]);   % center x
cy = mean([min(workspace_y), max(workspace_y)])+20 ;   % center y

scale = 60;        % half-width of the infinity sign in mm
                   % reduce if points fall outside workspace

num_points = 100;  % number of points along the path

t_inf = linspace(0, 2*pi, num_points + 1);
t_inf = t_inf(1:end-1);   % remove duplicate endpoint (path is closed)

inf_x = cx + scale .* cos(t_inf) ./ (1 + sin(t_inf).^2);
inf_y = cy + scale .* sin(t_inf) .* cos(t_inf) ./ (1 + sin(t_inf).^2);

% ==========================================================
% FILTER: KEEP ONLY POINTS INSIDE THE REACHABLE WORKSPACE
% ==========================================================
selected_x     = [];
selected_y     = [];
selected_theta = [];

for i = 1:length(inf_x)
    q = [inf_x(i); inf_y(i)];

    try
        theta = IPK_pantograph(q, robot);

        theta1 = theta(1);
        theta2 = theta(2);

        if theta1 >= theta1_min && theta1 <= theta1_max && ...
           theta2 >= theta2_min && theta2 <= theta2_max

            selected_x(end+1)       = inf_x(i);
            selected_y(end+1)       = inf_y(i);
            selected_theta(:,end+1) = theta;

        end

    catch
        % skip unreachable point
    end
end

num_points = length(selected_x);   % update after filtering

disp("Infinity sign points accepted by IK:");
disp(num_points);

disp("Selected infinity path points and MATLAB angles:");
disp(table(selected_x', selected_y', ...
    rad2deg(selected_theta(1,:))', ...
    rad2deg(selected_theta(2,:))', ...
    'VariableNames', {'x_mm','y_mm','theta1_matlab_deg','theta2_matlab_deg'}));

% ==========================================================
% CREATE SMOOTH TRAJECTORY THROUGH SELECTED POINTS
% ==========================================================
total_time = 30;        % seconds
samples    = 800;       % smoothness of signal

t_points = linspace(0, total_time, num_points);
t        = linspace(0, total_time, samples)';

theta1_matlab_signal = interp1(t_points, selected_theta(1,:), t, 'pchip');
theta2_matlab_signal = interp1(t_points, selected_theta(2,:), t, 'pchip');

% Optional smoothing
theta1_matlab_signal = smoothdata(theta1_matlab_signal, 'movmean', 15);
theta2_matlab_signal = smoothdata(theta2_matlab_signal, 'movmean', 15);

% ==========================================================
% MATLAB TO SIMSCAPE ANGLE CONVERSION
% ==========================================================
sign1 = 1;
sign2 = 1;

offset1 = pi/2;
offset2 = pi/2;

theta1_simscape_signal = sign1 * theta1_matlab_signal + offset1;
theta2_simscape_signal = sign2 * theta2_matlab_signal + offset2;

theta1_ts = timeseries(theta1_simscape_signal, t);
theta2_ts = timeseries(theta2_simscape_signal, t);

disp("theta1_ts and theta2_ts are ready for Simscape.");

% ==========================================================
% PLOT EXPECTED MATLAB LINK CONFIGURATIONS
% ==========================================================
A1 = [-d/2, 0];
A2 = [ d/2, 0];

figure;
hold on;
grid on;
axis equal;

% All reachable workspace points
scatter(workspace_x, workspace_y, 8, [0.85 0.85 0.85], 'filled');

% Full infinity sign outline (before IK filter) for reference
plot(inf_x, inf_y, 'g--', 'LineWidth', 1.2);

% Accepted path after IK filter
plot(selected_x, selected_y, 'k--', 'LineWidth', 1.5);
scatter(selected_x, selected_y, 40, 'b', 'filled');

% Base joints
plot(A1(1), A1(2), 'ro', 'MarkerSize', 9, 'LineWidth', 2);
plot(A2(1), A2(2), 'ro', 'MarkerSize', 9, 'LineWidth', 2);

text(A1(1), A1(2)-15, 'A1', 'HorizontalAlignment', 'center');
text(A2(1), A2(2)-15, 'A2', 'HorizontalAlignment', 'center');

% Draw robot links at every 10th point to avoid clutter
step = max(1, floor(num_points/10));
for i = 1:step:num_points

    theta1 = selected_theta(1, i);
    theta2 = selected_theta(2, i);

    P  = [selected_x(i), selected_y(i)];
    B1 = A1 + [l1*cos(theta1), l1*sin(theta1)];
    B2 = A2 + [l2*cos(theta2), l2*sin(theta2)];

    plot([A1(1) B1(1)], [A1(2) B1(2)], 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8);
    plot([B1(1)  P(1)], [B1(2)  P(2)], 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8);
    plot([A2(1) B2(1)], [A2(2) B2(2)], 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8);
    plot([B2(1)  P(1)], [B2(2)  P(2)], 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8);

    plot(B1(1), B1(2), 'ko', 'MarkerSize', 5, 'LineWidth', 1.5);
    plot(B2(1), B2(2), 'ko', 'MarkerSize', 5, 'LineWidth', 1.5);
    plot( P(1),  P(2), 'mo', 'MarkerSize', 7, 'LineWidth', 1.5);
end

xlabel('x (mm)');
ylabel('y (mm)');
title('Pantograph Drawing Robot — Infinity Sign Trajectory');

legend('Reachable workspace', ...
       'Infinity sign (target)', ...
       'Accepted path (IK valid)', ...
       'End-effector points', ...
       'Base joints', ...
       'Location', 'best');

% ==========================================================
% PLOT ANGLES SENT TO SIMSCAPE
% ==========================================================
figure;
plot(t, rad2deg(theta1_simscape_signal), 'LineWidth', 2);
hold on;
plot(t, rad2deg(theta2_simscape_signal), 'LineWidth', 2);
grid on;

xlabel('Time (s)');
ylabel('Simscape Joint Angle (deg)');
title('Motor Angles Sent to Simscape — Infinity Sign');
legend('theta1 to Revolute', 'theta2 to Revolute1');

disp("Run the Simscape model now.");