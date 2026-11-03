function pantograph_integrated_line_calibration()
clc;
clear;
close all;

%% ============================================================
% FINAL LINE CALIBRATION CODE
%
% This version generates the line in TASK SPACE:
%   x-y line points -> IPK at every point
%
% This is better for calibration than joint-space interpolation.
%
% Simulink uses normal visual path.
% Hardware uses mirrored + calibrated path.
%
% Arduino protocol:
%   HOME,theta1,theta2
%   BEGIN,N
%   P,theta1,theta2
%   ...
%   END
%   RUN
%% ============================================================

%% ================= USER SETTINGS =================

PORT = "COM8";
BAUD = 115200;

SEND_TO_ARDUINO = true;
RUN_SIMULINK_MODEL = false;
SIMULINK_MODEL_NAME = "simplifiedfinal";

% Robot dimensions in cm
robot.l1 = 12.5;
robot.l2 = 12.5;
robot.l3 = 16.6;
robot.l4 = 16.6;
robot.d  = 5.8;

% Joint limits in degrees
THETA1_MIN = 0;
THETA1_MAX = 180;
THETA2_MIN = -5;
THETA2_MAX = 180;

% Line settings in mm
% To reverse the visual direction, swap START_MM and FINAL_MM.
START_MM = [-50; 180];
FINAL_MM = [ 50; 180];

% Time settings
MOTION_TIME = 5;
SIM_STOP_TIME = 8;

% Number of command points
N_CMD = 120;

% Simscape angle convention
SIM_SIGN1 = 1;
SIM_SIGN2 = 1;
SIM_OFFSET1 = pi/2;
SIM_OFFSET2 = pi/2;

%% ================= HARDWARE CALIBRATION =================
% Tune these values only.
% Start with these defaults.

MIRROR_HARDWARE_X = true;

HW_X_SCALE = 1.00;
HW_Y_SCALE = 1.00;

HW_X_OFFSET_CM = 0.00;
HW_Y_OFFSET_CM = 1.00;

% This fixes line tilt.
% If the right side of the drawn line is lower, try +0.02.
% If the right side is higher, try -0.02.
HW_Y_SLOPE_PER_X = 0.04;

%% ================= CREATE TASK-SPACE LINE =================

t_motion = linspace(0, MOTION_TIME, N_CMD)';

tau = t_motion / MOTION_TIME;

% 5th-order smooth scaling
s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

START_SIM_CM = START_MM / 10;
FINAL_SIM_CM = FINAL_MM / 10;

x_sim_cm = START_SIM_CM(1) + s * (FINAL_SIM_CM(1) - START_SIM_CM(1));
y_sim_cm = START_SIM_CM(2) + s * (FINAL_SIM_CM(2) - START_SIM_CM(2));

%% ================= HARDWARE CALIBRATED PATH =================

if MIRROR_HARDWARE_X
    x_hw_cm = -HW_X_SCALE * x_sim_cm + HW_X_OFFSET_CM;
else
    x_hw_cm =  HW_X_SCALE * x_sim_cm + HW_X_OFFSET_CM;
end

y_hw_cm = HW_Y_SCALE * y_sim_cm + HW_Y_OFFSET_CM + HW_Y_SLOPE_PER_X * x_sim_cm;

START_HW_CM = [x_hw_cm(1); y_hw_cm(1)];
FINAL_HW_CM = [x_hw_cm(end); y_hw_cm(end)];

fprintf("\n========== SIMULINK LINE ==========\n");
fprintf("Simulink start: X = %.3f cm, Y = %.3f cm\n", x_sim_cm(1), y_sim_cm(1));
fprintf("Simulink final: X = %.3f cm, Y = %.3f cm\n", x_sim_cm(end), y_sim_cm(end));
fprintf("===================================\n\n");

fprintf("========== REAL HARDWARE HOME ==========\n");
fprintf("Put the REAL pen physically at:\n");
fprintf("X = %.3f cm\n", START_HW_CM(1));
fprintf("Y = %.3f cm\n", START_HW_CM(2));
fprintf("========================================\n\n");

%% ================= IPK FOR SIMULINK AND HARDWARE =================

theta_sim_rad = zeros(2, N_CMD);
theta_hw_rad  = zeros(2, N_CMD);

for i = 1:N_CMD

    theta_sim = IPK_pantograph([x_sim_cm(i); y_sim_cm(i)], robot);
    deg_sim = rad2deg(theta_sim);
    check_limits(deg_sim, THETA1_MIN, THETA1_MAX, THETA2_MIN, THETA2_MAX);
    theta_sim_rad(:, i) = theta_sim;

    theta_hw = IPK_pantograph([x_hw_cm(i); y_hw_cm(i)], robot);
    deg_hw = rad2deg(theta_hw);
    check_limits(deg_hw, THETA1_MIN, THETA1_MAX, THETA2_MIN, THETA2_MAX);
    theta_hw_rad(:, i) = theta_hw;

end

%% ================= HARDWARE ANGLES =================

angles_deg = rad2deg(theta_hw_rad);

hw_home_deg = angles_deg(:,1);

% Force first and last exactly
angles_deg(:,1) = rad2deg(IPK_pantograph(START_HW_CM, robot));
angles_deg(:,end) = rad2deg(IPK_pantograph(FINAL_HW_CM, robot));

for i = 1:N_CMD
    check_limits(angles_deg(:, i), THETA1_MIN, THETA1_MAX, THETA2_MIN, THETA2_MAX);
end

fprintf("\nHardware line trajectory ready.\n");
fprintf("Command points = %d\n", N_CMD);
fprintf("Hardware home angles: theta1 = %.3f deg, theta2 = %.3f deg\n", hw_home_deg(1), hw_home_deg(2));
fprintf("Theta1 range = %.2f to %.2f deg\n", min(angles_deg(1,:)), max(angles_deg(1,:)));
fprintf("Theta2 range = %.2f to %.2f deg\n", min(angles_deg(2,:)), max(angles_deg(2,:)));

%% ================= SIMSCAPE SIGNALS =================

theta1_matlab_rad = theta_sim_rad(1,:)';
theta2_matlab_rad = theta_sim_rad(2,:)';

theta1_simscape = SIM_SIGN1 * theta1_matlab_rad + SIM_OFFSET1;
theta2_simscape = SIM_SIGN2 * theta2_matlab_rad + SIM_OFFSET2;

EE_motion = [x_sim_cm(:), y_sim_cm(:)];

if SIM_STOP_TIME > MOTION_TIME

    t_hold = linspace(MOTION_TIME, SIM_STOP_TIME, 100)';

    theta1_hold = theta1_simscape(end) * ones(size(t_hold));
    theta2_hold = theta2_simscape(end) * ones(size(t_hold));

    EE_hold = repmat(EE_motion(end,:), length(t_hold), 1);

    t_sim = [t_motion; t_hold(2:end)];

    theta1_simscape = [theta1_simscape; theta1_hold(2:end)];
    theta2_simscape = [theta2_simscape; theta2_hold(2:end)];

    EE_full = [EE_motion; EE_hold(2:end,:)];

else
    t_sim = t_motion;
    EE_full = EE_motion;
end

theta1_ts = timeseries(theta1_simscape, t_sim);
theta2_ts = timeseries(theta2_simscape, t_sim);
EE_path_ts = timeseries(EE_full, t_sim);

assignin('base', 'theta1_ts', theta1_ts);
assignin('base', 'theta2_ts', theta2_ts);
assignin('base', 'EE_path_ts', EE_path_ts);
assignin('base', 'angles_deg_hardware', angles_deg);
assignin('base', 'hw_home_deg', hw_home_deg);
assignin('base', 't_sim', t_sim);

fprintf("\nSimscape line variables created:\n");
fprintf("theta1_ts\n");
fprintf("theta2_ts\n");
fprintf("EE_path_ts\n");
fprintf("Set Simulink Stop Time to %.1f seconds.\n", SIM_STOP_TIME);

%% ================= PLOTS =================

figure;
hold on;
grid on;
axis equal;

plot(x_sim_cm, y_sim_cm, 'b-', 'LineWidth', 2);
plot(x_hw_cm, y_hw_cm, 'r--', 'LineWidth', 2);

plot(x_sim_cm(1), y_sim_cm(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(x_hw_cm(1), y_hw_cm(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

xlabel("X (cm)");
ylabel("Y (cm)");
title("Line: Simulink Visual Path vs Hardware Calibrated Path");
legend("Simulink visual line", "Hardware calibrated line", ...
       "Simulink start", "Hardware home", "Location", "best");

figure;
plot(1:N_CMD, angles_deg(1,:), 'LineWidth', 2);
hold on;
plot(1:N_CMD, angles_deg(2,:), 'LineWidth', 2);
grid on;
xlabel("Command Point");
ylabel("Hardware Angle (deg)");
legend("theta1 hardware", "theta2 hardware");
title("Line Angles Sent to Arduino");

figure;
plot(t_sim, rad2deg(theta1_simscape), 'LineWidth', 2);
hold on;
plot(t_sim, rad2deg(theta2_simscape), 'LineWidth', 2);
grid on;
xlabel("Time (s)");
ylabel("Simscape Joint Angle (deg)");
legend("theta1\_ts", "theta2\_ts");
title("Line Angles Sent to Simscape");

%% ================= RUN SIMULINK =================

if RUN_SIMULINK_MODEL
    fprintf("\nRunning Simulink model: %s\n", SIMULINK_MODEL_NAME);
    sim(SIMULINK_MODEL_NAME);
    fprintf("Simulink simulation finished.\n");
else
    fprintf("\nSimscape inputs are ready. Run Simulink manually if needed.\n");
end

%% ================= SEND TO ARDUINO =================

if SEND_TO_ARDUINO

    fprintf("\nConnecting to Arduino on %s...\n", PORT);

    s = serialport(PORT, BAUD);
    configureTerminator(s, "LF");
    pause(2.5);
    flush(s);

    home_cmd = sprintf("HOME,%.3f,%.3f", hw_home_deg(1), hw_home_deg(2));
    writeline(s, home_cmd);
    wait_for(s, "READY", 10);

    fprintf("Arduino synced at hardware home.\n");
    fprintf("Sent: %s\n", home_cmd);

    begin_cmd = sprintf("BEGIN,%d", N_CMD);
    writeline(s, begin_cmd);
    wait_for(s, "READY", 10);

    fprintf("Sending line trajectory points...\n");

    for i = 1:N_CMD
        cmd = sprintf("P,%.3f,%.3f", angles_deg(1,i), angles_deg(2,i));
        writeline(s, cmd);
        wait_for(s, "OKP", 10);
    end

    writeline(s, "END");
    wait_for(s, "READY", 10);

    fprintf("Trajectory sent.\n");

    fprintf("Running real hardware line...\n");
    writeline(s, "RUN");
    wait_for(s, "DONE", 120);

    fprintf("Done. Robot should be at the line final point.\n");

    clear s;
end

end

%% ========================================================================
% INVERSE POSITION KINEMATICS
%% ========================================================================

function theta = IPK_pantograph(q, robot)

    x = q(1);
    y = q(2);

    l1 = robot.l1;
    l2 = robot.l2;
    l3 = robot.l3;
    l4 = robot.l4;
    d  = robot.d;

    if y <= 0
        error("Target must be in front half workspace: y > 0.");
    end

    A1 = [-d/2, 0];
    A2 = [ d/2, 0];

    P = [x, y];

    v1 = P - A1;
    r1 = norm(v1);

    if r1 > (l1 + l3) || r1 < abs(l1 - l3)
        error("Point unreachable by left chain.");
    end

    alpha1 = atan2(v1(2), v1(1));
    c1 = (l1^2 + r1^2 - l3^2) / (2*l1*r1);
    c1 = max(min(c1, 1), -1);
    beta1 = acos(c1);

    theta1 = alpha1 + beta1;

    v2 = P - A2;
    r2 = norm(v2);

    if r2 > (l2 + l4) || r2 < abs(l2 - l4)
        error("Point unreachable by right chain.");
    end

    alpha2 = atan2(v2(2), v2(1));
    c2 = (l2^2 + r2^2 - l4^2) / (2*l2*r2);
    c2 = max(min(c2, 1), -1);
    beta2 = acos(c2);

    theta2 = alpha2 - beta2;

    theta = [theta1; theta2];
end

function check_limits(deg, t1min, t1max, t2min, t2max)

    if deg(1) < t1min || deg(1) > t1max || ...
       deg(2) < t2min || deg(2) > t2max

        error("Joint limit error: theta1=%.2f deg, theta2=%.2f deg.", ...
            deg(1), deg(2));
    end
end

function wait_for(s, expected, timeout_s)

    t0 = tic;

    while toc(t0) < timeout_s

        if s.NumBytesAvailable > 0

            resp = strtrim(readline(s));

            if strcmp(resp, expected)
                return;
            else
                fprintf("Arduino: %s\n", resp);
            end
        end

        pause(0.01);
    end

    error("Timeout waiting for Arduino response: %s", expected);
end