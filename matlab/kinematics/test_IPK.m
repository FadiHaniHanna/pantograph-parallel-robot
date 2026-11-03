clc;
clear;
close all;

robot = Robot_params();

q = [0; 150];   % desired end-effector point in mm

theta = IPK_pantograph(q, robot);

theta1_cmd = theta(1);
theta2_cmd = theta(2);

disp('Angles in degrees:');
disp(rad2deg(theta));

% Visualize robot configuration
plot_robot(theta, robot, q);