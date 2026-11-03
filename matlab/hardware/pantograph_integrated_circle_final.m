function pantograph_integrated_circle_final()
clc;
clear;
close all;

%% ============================================================
% FINAL INTEGRATED CIRCLE CODE
%
% Simulink:
%   Starts from the RIGHT point of the circle and moves UP.
%
% Hardware:
%   X is mirrored to compensate the real robot mirror.
%   Hardware order is reversed because your hardware was moving DOWN
%   while Simulink was moving UP.
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
RUN_SIMULINK_MODEL = true;
SIMULINK_MODEL_NAME = "simplifiedfinal";

% Keep true because this fixed the left/right mismatch in infinity
MIRROR_HARDWARE_X = true;

% IMPORTANT FOR CIRCLE:
% Hardware was moving down while Simulink was moving up.
% Keep this true.
REVERSE_HARDWARE_CIRCLE_ORDER = true;

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

% Circle settings in mm
CENTER_MM = [0; 180];
RADIUS_MM = 40;

% Time settings
TOTAL_TIME = 6;
SIM_STOP_TIME = 8;

% Number of points
N_RAW = 800;
N_CMD = 120;        % Arduino MAX_POINTS must be >= 120

% Simscape angle convention
SIM_SIGN1 = 1;
SIM_SIGN2 = 1;
SIM_OFFSET1 = pi/2;
SIM_OFFSET2 = pi/2;

%% ================= GENERATE SIMULINK VISUAL CIRCLE =================
% Simulink circle starts from right point:
% x = center_x + radius, y = center_y
% Then moves upward because sin(t) increases.

t_raw = linspace(0, 2*pi, N_RAW);

x_sim_raw_mm = CENTER_MM(1) + RADIUS_MM * cos(t_raw);
y_sim_raw_mm = CENTER_MM(2) + RADIUS_MM * sin(t_raw);

% Force closed path
x_sim_raw_mm(end) = x_sim_raw_mm(1);
y_sim_raw_mm(end) = y_sim_raw_mm(1);

% Convert to cm
x_sim_raw_cm = x_sim_raw_mm / 10;
y_sim_raw_cm = y_sim_raw_mm / 10;

%% ================= CREATE HARDWARE-CORRECTED CIRCLE =================
% Mirror X for hardware only.

if MIRROR_HARDWARE_X
    x_hw_raw_cm = -x_sim_raw_cm;
else
    x_hw_raw_cm = x_sim_raw_cm;
end

y_hw_raw_cm = y_sim_raw_cm;

% Reverse hardware order only.
% Because the hardware was moving downward while Simulink moved upward.
% Since the circle is closed, the first point remains the same.
if REVERSE_HARDWARE_CIRCLE_ORDER
    x_hw_raw_cm = fliplr(x_hw_raw_cm);
    y_hw_raw_cm = fliplr(y_hw_raw_cm);
end

SIM_HOME_Q = [x_sim_raw_cm(1); y_sim_raw_cm(1)];
HW_HOME_Q  = [x_hw_raw_cm(1);  y_hw_raw_cm(1)];

fprintf("\n========== SIMULINK CIRCLE START ==========\n");
fprintf("Simulink visual start:\n");
fprintf("X = %.3f cm\n", SIM_HOME_Q(1));
fprintf("Y = %.3f cm\n", SIM_HOME_Q(2));
fprintf("Simulink should move UP from this point.\n");
fprintf("==========================================\n\n");

fprintf("========== REAL HARDWARE HOME ==========\n");
fprintf("Put the REAL pen physically at:\n");
fprintf("X = %.3f cm\n", HW_HOME_Q(1));
fprintf("Y = %.3f cm\n", HW_HOME_Q(2));
fprintf("Hardware path is X-mirrored and order-reversed to match Simulink.\n");
fprintf("========================================\n\n");

%% ================= IPK FOR RAW HARDWARE PATH =================

theta_hw_raw_rad = zeros(2, N_RAW);

for i = 1:N_RAW

    theta_hw = IPK_pantograph([x_hw_raw_cm(i); y_hw_raw_cm(i)], robot);
    deg_hw = rad2deg(theta_hw);

    check_limits(deg_hw, THETA1_MIN, THETA1_MAX, THETA2_MIN, THETA2_MAX);

    theta_hw_raw_rad(:, i) = theta_hw;
end

fprintf("All hardware circle points are reachable.\n");

%% ================= RESAMPLE HARDWARE FOR SMOOTHNESS =================

hw_home_theta = IPK_pantograph(HW_HOME_Q, robot);
hw_home_deg = rad2deg(hw_home_theta);

theta_hw_raw_deg = rad2deg(theta_hw_raw_rad);

% Smooth hardware raw angles
theta_hw_raw_deg(1,:) = smoothdata(theta_hw_raw_deg(1,:), 'movmean', 9);
theta_hw_raw_deg(2,:) = smoothdata(theta_hw_raw_deg(2,:), 'movmean', 9);

% Force raw start/end to hardware home
theta_hw_raw_deg(:,1)   = hw_home_deg;
theta_hw_raw_deg(:,end) = hw_home_deg;

% Joint-space distance for smoother hardware motion
dtheta1 = diff(theta_hw_raw_deg(1,:));
dtheta2 = diff(theta_hw_raw_deg(2,:));

joint_distance = sqrt(dtheta1.^2 + dtheta2.^2);
s_raw = [0, cumsum(joint_distance)];

[s_raw_unique, unique_idx] = unique(s_raw, 'stable');

theta1_hw_unique = theta_hw_raw_deg(1, unique_idx);
theta2_hw_unique = theta_hw_raw_deg(2, unique_idx);

s_cmd = linspace(0, s_raw_unique(end), N_CMD);

theta1_hw_cmd = interp1(s_raw_unique, theta1_hw_unique, s_cmd, 'pchip');
theta2_hw_cmd = interp1(s_raw_unique, theta2_hw_unique, s_cmd, 'pchip');

angles_deg = [theta1_hw_cmd; theta2_hw_cmd];

% Force hardware start/end exactly
angles_deg(:,1)   = hw_home_deg;
angles_deg(:,end) = hw_home_deg;

for i = 1:N_CMD
    check_limits(angles_deg(:, i), THETA1_MIN, THETA1_MAX, THETA2_MIN, THETA2_MAX);
end

fprintf("\nHardware circle trajectory ready.\n");
fprintf("Command points = %d\n", N_CMD);
fprintf("Hardware home angles: theta1 = %.3f deg, theta2 = %.3f deg\n", hw_home_deg(1), hw_home_deg(2));
fprintf("Theta1 range = %.2f to %.2f deg\n", min(angles_deg(1,:)), max(angles_deg(1,:)));
fprintf("Theta2 range = %.2f to %.2f deg\n", min(angles_deg(2,:)), max(angles_deg(2,:)));

%% ================= CREATE SIMSCAPE SIGNALS =================
% Simscape uses the NORMAL circle order:
% starts from right and moves up.

t_sim_motion = linspace(0, TOTAL_TIME, N_CMD)';

t_circle_cmd = linspace(0, 2*pi, N_CMD);

x_sim_cmd_cm = (CENTER_MM(1) + RADIUS_MM * cos(t_circle_cmd)) / 10;
y_sim_cmd_cm = (CENTER_MM(2) + RADIUS_MM * sin(t_circle_cmd)) / 10;

x_sim_cmd_cm(end) = x_sim_cmd_cm(1);
y_sim_cmd_cm(end) = y_sim_cmd_cm(1);

theta_sim_cmd_rad = zeros(2, N_CMD);

for i = 1:N_CMD
    theta_sim_cmd_rad(:, i) = IPK_pantograph([x_sim_cmd_cm(i); y_sim_cmd_cm(i)], robot);
end

sim_home_theta = IPK_pantograph(SIM_HOME_Q, robot);

theta_sim_cmd_rad(:,1)   = sim_home_theta;
theta_sim_cmd_rad(:,end) = sim_home_theta;

theta1_matlab_rad = theta_sim_cmd_rad(1,:)';
theta2_matlab_rad = theta_sim_cmd_rad(2,:)';

theta1_simscape = SIM_SIGN1 * theta1_matlab_rad + SIM_OFFSET1;
theta2_simscape = SIM_SIGN2 * theta2_matlab_rad + SIM_OFFSET2;

EE_motion = [x_sim_cmd_cm(:), y_sim_cmd_cm(:)];

% Hold final position until Simulink stop time
if SIM_STOP_TIME > TOTAL_TIME

    t_hold = linspace(TOTAL_TIME, SIM_STOP_TIME, 100)';

    theta1_hold = theta1_simscape(end) * ones(size(t_hold));
    theta2_hold = theta2_simscape(end) * ones(size(t_hold));

    EE_hold = repmat(EE_motion(end,:), length(t_hold), 1);

    t_sim = [t_sim_motion; t_hold(2:end)];

    theta1_simscape = [theta1_simscape; theta1_hold(2:end)];
    theta2_simscape = [theta2_simscape; theta2_hold(2:end)];

    EE_full = [EE_motion; EE_hold(2:end,:)];

else
    t_sim = t_sim_motion;
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

fprintf("\nSimscape circle variables created:\n");
fprintf("theta1_ts\n");
fprintf("theta2_ts\n");
fprintf("EE_path_ts\n");
fprintf("Set Simulink Stop Time to %.1f seconds.\n", SIM_STOP_TIME);

% %% ================= PLOTS =================
% 
% figure;
% plot(x_sim_raw_cm, y_sim_raw_cm, 'LineWidth', 2);
% hold on;
% plot(x_hw_raw_cm, y_hw_raw_cm, '--', 'LineWidth', 2);
% plot(SIM_HOME_Q(1), SIM_HOME_Q(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
% plot(HW_HOME_Q(1), HW_HOME_Q(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
% grid on;
% axis equal;
% xlabel("X (cm)");
% ylabel("Y (cm)");
% title("Circle: Simulink Path vs Hardware Corrected Path");
% legend("Simulink visual circle", "Hardware corrected circle", ...
%        "Simulink start", "Hardware home", "Location", "best");
% 
% figure;
% plot(1:N_CMD, angles_deg(1,:), 'LineWidth', 2);
% hold on;
% plot(1:N_CMD, angles_deg(2,:), 'LineWidth', 2);
% grid on;
% xlabel("Command Point");
% ylabel("Hardware Angle (deg)");
% legend("theta1 hardware", "theta2 hardware");
% title("Circle Angles Sent to Arduino");
% 
% figure;
% plot(t_sim, rad2deg(theta1_simscape), 'LineWidth', 2);
% hold on;
% plot(t_sim, rad2deg(theta2_simscape), 'LineWidth', 2);
% grid on;
% xlabel("Time (s)");
% ylabel("Simscape Joint Angle (deg)");
% legend("theta1\_ts", "theta2\_ts");
% title("Circle Angles Sent to Simscape");

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

    fprintf("Sending circle trajectory points...\n");

    for i = 1:N_CMD
        cmd = sprintf("P,%.3f,%.3f", angles_deg(1,i), angles_deg(2,i));
        writeline(s, cmd);
        wait_for(s, "OKP", 10);
    end

    writeline(s, "END");
    wait_for(s, "READY", 10);

    fprintf("Trajectory sent.\n");

    fprintf("Running real hardware circle...\n");
    writeline(s, "RUN");
    wait_for(s, "DONE", 120);

    fprintf("Done. Robot should be back at hardware home.\n");

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

%% ========================================================================
% HELPERS
%% ========================================================================

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