function [time_series, EE_path] = EE_linear_traj(total_time, q_start, q_final)

    dt = 0.05;

    N = floor(total_time / dt);

    n = length(q_start);

    EE_path = zeros(n, N + 1);
    time_series = zeros(1, N + 1);

    for k = 1:(N + 1)

        alpha = (k - 1) / N;

        EE_path(:, k) = q_start + alpha * (q_final - q_start);

        time_series(k) = dt * (k - 1);

    end

end