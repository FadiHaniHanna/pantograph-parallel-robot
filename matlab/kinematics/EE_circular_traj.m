function [time_series, EE_path] = ...
    EE_circular_traj(total_time, center, radius)

    dt = 0.05;

    N = floor(total_time / dt);

    time_series = zeros(1, N + 1);

    EE_path = zeros(2, N + 1);

    for k = 1:(N + 1)

        theta = 2*pi*(k-1)/N;

        x = center(1) + radius*cos(theta);

        y = center(2) + radius*sin(theta);

        EE_path(:,k) = [x; y];

        time_series(k) = dt*(k-1);

    end

end