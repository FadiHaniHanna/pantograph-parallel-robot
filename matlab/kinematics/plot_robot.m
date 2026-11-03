% ==========================================================
% GET SIMSCAPE TRACE FROM out.frame_trace
% ==========================================================
trace_xy_m = out.frame_trace.Data;
trace_t    = out.frame_trace.Time;

trace_x_mm = trace_xy_m(:,1) * 1000;
trace_y_mm = trace_xy_m(:,2) * 1000;


t_start_compare = 0;
t_end_compare   = 30;

idx = trace_t >= t_start_compare & trace_t <= t_end_compare;

trace_x_mm = trace_x_mm(idx);
trace_y_mm = trace_y_mm(idx);

% ==========================================================
% FRAME MAPPING
% ==========================================================
% Your red trace is vertical, so try rotating it 90 degrees.
% Try this first:
x_raw = trace_y_mm;
y_raw = -trace_x_mm;

% Shift Simscape trace center to desired MATLAB path center
x_shift = mean(selected_x) - mean(x_raw);
y_shift = mean(selected_y) - mean(y_raw);

x_aligned = x_raw + x_shift;
y_aligned = y_raw + y_shift;

% ==========================================================
% PLOT
% ==========================================================
figure;
hold on;
grid on;
axis equal;

plot(selected_x, selected_y, 'b--', 'LineWidth', 2);
plot(x_aligned, y_aligned, 'r', 'LineWidth', 2);

xlabel('x (mm)');
ylabel('y (mm)');
title('Desired MATLAB Path vs Rotated Simscape Trace');

legend('Desired MATLAB path', 'Aligned Simscape trace');