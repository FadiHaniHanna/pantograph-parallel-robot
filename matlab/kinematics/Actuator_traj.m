function [time_series, Actuator_traj, EE_path] = ...
    Actuator_traj(total_time, center, radius, robot_params)

%GENERATE CIRCULAR EE TRAJECTORY
[time_series, EE_path] = ...
    EE_circular_traj(total_time, center, radius);

%NUMBER OF TIME STEPS 
N = length(time_series);

%INITIAL ACTUATOR VALUES
L0 = IPK_pantograph(EE_path(:,1), robot_params);

%NUMBER OF ACTUATORS
m = length(L0);

%PREALLOCATE ACTUATOR TRAJECTORY
Actuator_traj = zeros(m, N);

%FIRST POINT 
Actuator_traj(:,1) = L0;

%LOOP OVER ALL EE POINTS
for k = 2:N

    Actuator_traj(:,k) = ...
        IPK_pantograph(EE_path(:,k), robot_params);

end

end