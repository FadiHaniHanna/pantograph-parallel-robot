clc;
clear;
close all;

robot = Robot_params();

l1 = robot.l1;
l2 = robot.l2;
l3 = robot.l3;
l4 = robot.l4;
d  = robot.d;

%ANGLE LIMITS
theta1_min = deg2rad(20);
theta1_max = deg2rad(160);

theta2_min = deg2rad(20);
theta2_max = deg2rad(160);

%GRID
x_values = -250:10:250;
y_values = -50:10:300;

workspace_x = [];
workspace_y = [];
theta_store = [];

%FIND WORKSPACE
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
                theta_store(:,end+1) = theta;

            end

        catch
            % unreachable point
        end
    end
end

%PLOT
figure;
hold on;
grid on;
axis equal;

A1 = [-d/2, 0];
A2 = [ d/2, 0];

%DRAW MANY ROBOT CONFIGURATIONS IN GRAY 
step = 20;   % increase = fewer gray robots, decrease = more

for i = 1:step:length(workspace_x)

    theta = theta_store(:,i);

    x = workspace_x(i);
    y = workspace_y(i);
    P = [x, y];

    B1 = A1 + [l1*cos(theta(1)), l1*sin(theta(1))];
    B2 = A2 + [l2*cos(theta(2)), l2*sin(theta(2))];

    plot([A1(1) B1(1)], [A1(2) B1(2)], 'Color', [0.6 0.6 0.6], 'LineWidth', 1);
    plot([B1(1) P(1)],  [B1(2) P(2)],  'Color', [0.6 0.6 0.6], 'LineWidth', 1);

    plot([A2(1) B2(1)], [A2(2) B2(2)], 'Color', [0.6 0.6 0.6], 'LineWidth', 1);
    plot([B2(1) P(1)],  [B2(2) P(2)],  'Color', [0.6 0.6 0.6], 'LineWidth', 1);

end

%PLOT WORKSPACE POINTS
scatter(workspace_x, workspace_y, 15, 'b', 'filled');

%BASE JOINTS
plot(A1(1), A1(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
plot(A2(1), A2(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);

xlabel('x (mm)');
ylabel('y (mm)');
title('Pantograph Robot Workspace with Configurations');

legend('Robot configurations', 'Workspace');