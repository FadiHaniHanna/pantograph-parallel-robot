clc;
clear;
close all;

robot = Robot_params();

q_original = [0; 200];

theta = IPK_pantograph(q_original, robot);

q_solutions = FPK_pantograph(theta, robot);

disp('Original q:')
disp(q_original)

disp('FPK solution 1:')
disp(q_solutions(:,1))

disp('FPK solution 2:')
disp(q_solutions(:,2))

disp('Error solution 1:')
disp(norm(q_original - q_solutions(:,1)))

disp('Error solution 2:')
disp(norm(q_original - q_solutions(:,2)))