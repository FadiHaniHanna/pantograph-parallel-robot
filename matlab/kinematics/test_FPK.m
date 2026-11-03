clc;
clear;
close all;

robot = Robot_params();

q_original = [0; 200];

theta = IPK_pantograph(q_original, robot);

q_check = FPK_pantograph(theta, robot);

disp('Original q:')
disp(q_original)

disp('Theta from IPK in degrees:')
disp(rad2deg(theta))

disp('Recovered q from FPK:')
disp(q_check)