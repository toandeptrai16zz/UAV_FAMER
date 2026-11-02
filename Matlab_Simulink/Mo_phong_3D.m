function [summary, logs] = quadrotor_pid_testbench()
% QUADROTOR_PID_TESTBENCH
% Three simulation tests for a quadrotor:
%   1) TAKEOFF_LAND : vertical take-off, altitude hold, landing
%   2) WAYPOINT     : fly through a sequence of 3-D waypoints
%   3) ROW          : follow a crop row while holding clearance above canopy
%
% Run in MATLAB with:
%   [summary, logs] = quadrotor_pid_testbench;
%
% IMPORTANT:
% This is a 6-DOF rigid-body simulation with motor lag, motor saturation,
% position/attitude/rate loops, wind, sensor noise, and a simulated downward
% rangefinder. It is still a model, not proof that a real aircraft is safe.
% Replace every assumed physical parameter with measured data before using
% the simulation to guide real-flight tuning.

clc;
close all;
rng(12);                         % Repeatable disturbances/noise

cfg = default_config();

% Choose one or all tests. Examples:
% cfg.scenarios = {'TAKEOFF_LAND'};
% cfg.scenarios = {'WAYPOINT'};
% cfg.scenarios = {'ROW'};
cfg.scenarios = {'ROW'};  % Start with take-off/landing so it is easy to observe.
% To run all tests, use: {'TAKEOFF_LAND','WAYPOINT','ROW'}

% 3-D playback settings.
cfg.animate = true;
cfg.realtime_playback = true;
cfg.startup_display_s = 1.5;  % Show the drone on the ground before take-off.

summary = struct([]);
logs = cell(numel(cfg.scenarios), 1);

fprintf('\n============================================================\n');
fprintf(' QUADROTOR PID TESTBENCH - MODEL-BASED TESTS\n');
fprintf('============================================================\n');
fprintf('Mass assumption: %.2f kg | Max thrust/motor: %.2f N\n', ...
        cfg.mass, cfg.max_motor_thrust);
fprintf('Theoretical T/W from these assumptions: %.2f\n', ...
        4*cfg.max_motor_thrust/(cfg.mass*cfg.g));
fprintf('These values MUST be replaced by measured aircraft data.\n');

for n = 1:numel(cfg.scenarios)
    scenario = upper(cfg.scenarios{n});
    logs{n} = simulate_one_scenario(cfg, scenario);

    summary(n).scenario = scenario; %#ok<AGROW>
    summary(n).rmse_xy_m = logs{n}.rmse_xy;
    summary(n).rmse_z_or_clearance_m = logs{n}.rmse_z;
    summary(n).max_tilt_deg = logs{n}.max_tilt_deg;
    summary(n).motor_saturation_percent = logs{n}.motor_saturation_percent;
    summary(n).max_motor_thrust_N = logs{n}.max_motor_thrust;
    summary(n).range_fallback_percent = logs{n}.range_fallback_percent;
    summary(n).model_flag = logs{n}.model_flag;
end

fprintf('\n======================= SUMMARY ============================\n');
for n = 1:numel(summary)
    fprintf('%-14s | RMSE XY %.3f m | RMSE Z/clearance %.3f m | ', ...
        summary(n).scenario, summary(n).rmse_xy_m, ...
        summary(n).rmse_z_or_clearance_m);
    fprintf('max tilt %.1f deg | motor saturation %.2f%% | %s\n', ...
        summary(n).max_tilt_deg, summary(n).motor_saturation_percent, ...
        summary(n).model_flag);
end
fprintf('============================================================\n');
fprintf(['Reminder: a good simulation result is NOT a flight-safety ', ...
         'certificate. Validate sensors, thrust, ESCs, frame and failsafes.\n']);
end

function cfg = default_config()
% PHYSICAL PARAMETERS -- starter assumptions, not measured specifications.
cfg.mass = 1.25;                  % kg; replace with measured AUW
cfg.g = 9.81;                     % m/s^2
cfg.I = diag([0.018, 0.018, 0.030]); % kg.m^2; measure/estimate for this frame
cfg.arm_length = 0.225;           % m; motor centre to frame centre
cfg.max_motor_thrust = 8.5;       % N per motor; MUST be measured on a thrust stand
cfg.motor_tau = 0.045;            % s; approximate motor+ESC thrust response
cfg.yaw_moment_ratio = 0.015;     % m; yaw torque / thrust, placeholder
cfg.linear_drag = [0.22; 0.22; 0.30]; % kg/s, simplified translational drag
cfg.rate_damping = [0.003; 0.003; 0.002]; % N.m.s/rad

% Numerical integration and simulated sensor rates.
cfg.dt = 0.002;                   % 500 Hz integration/controller
cfg.position_sensor_hz = 50;      % assumed GPS/optical-flow position update
cfg.range_sensor_hz = 50;         % MT-06-like range update
cfg.range_min_m = 0.02;
cfg.range_max_m = 6.0;
cfg.range_noise_sigma_m = 0.015;
cfg.range_dropout_probability = 0.01; % random invalid samples per measurement
cfg.range_timeout_s = 0.20;
cfg.range_clearance_target_m = 1.00;
cfg.range_fallback_altitude_hold = true;

% OUTER POSITION PID: desired acceleration in world coordinates.
% XYZ gains are intentionally exposed here for tuning.
cfg.Kp_pos = [1.45; 1.45; 2.60];
cfg.Ki_pos = [0.05; 0.05; 0.25];
cfg.Kd_pos = [1.85; 1.85; 2.20];
cfg.pos_integral_limit = [1.5; 1.5; 0.8];

% ROW mode uses a direct range/clearance loop for vertical clearance.
cfg.Kp_range = 3.0;
cfg.Ki_range = 0.35;
cfg.Kd_range = 2.0;
cfg.range_integral_limit = 0.8;

% ATTITUDE loop: angle error -> desired body rates.
cfg.Kp_angle = [5.0; 5.0; 3.0];
cfg.max_body_rate = [3.5; 3.5; 2.0]; % rad/s

% BODY-RATE PID: desired body rates -> torque.
cfg.Kp_rate = [0.22; 0.22; 0.12];
cfg.Ki_rate = [0.06; 0.06; 0.025];
cfg.Kd_rate = [0.0025; 0.0025; 0.0015];
cfg.rate_integral_limit = [2.0; 2.0; 1.5];
cfg.max_torque = [0.90; 0.90; 0.25]; % N.m, then motor allocator applies limits

% Limits and disturbances.
cfg.max_tilt = deg2rad(25);
cfg.max_xy_acc = 3.5;             % m/s^2
cfg.max_z_acc = 3.0;              % m/s^2
cfg.wind_gust_start_s = 8.0;
cfg.wind_gust_end_s = 10.0;
cfg.wind_gust_accel = [0.45; -0.30; 0.0]; % m/s^2
cfg.animate_fps = 25;
cfg.ground_contact_z = 0.08;       % Simplified body-centre height at touchdown (m).
end

function out = simulate_one_scenario(cfg, scenario)
dt = cfg.dt;
if strcmp(scenario, 'TAKEOFF_LAND')
    Tsim = 18.0;
    initial_pos = [0; 0; cfg.ground_contact_z];
elseif strcmp(scenario, 'WAYPOINT')
    Tsim = 23.0;
    initial_pos = [0; 0; 0.20];
elseif strcmp(scenario, 'ROW')
    Tsim = 23.0;
    initial_pos = [0; 0; canopy_height(0,0) + cfg.range_clearance_target_m];
else
    error('Unknown scenario "%s". Use TAKEOFF_LAND, WAYPOINT or ROW.', scenario);
end

t = 0:dt:Tsim;
N = numel(t);
sensor_period = 1/cfg.position_sensor_hz;
range_period = 1/cfg.range_sensor_hz;
sensor_div = max(1, round(sensor_period/dt));
range_div = max(1, round(range_period/dt));
anim_div = max(1, round((1/cfg.animate_fps)/dt));

% State: world position/velocity, Z-up; Euler angles [roll pitch yaw];
% body angular velocity [p q r].
pos = zeros(3,N); vel = zeros(3,N);
att = zeros(3,N); omega = zeros(3,N);
pos(:,1) = initial_pos;

% Four motor thrust states (N). Start at approximate hover thrust.
motor_pos = cfg.arm_length/sqrt(2) * ...
    [ 1,  1, 0; ...
     -1,  1, 0; ...
     -1, -1, 0; ...
      1, -1, 0];
spin_dir = [1; -1; 1; -1];
alloc = [ones(1,4); motor_pos(:,2)'; -motor_pos(:,1)'; ...
         (cfg.yaw_moment_ratio*spin_dir)'];
motor_thrust = (cfg.mass*cfg.g/4)*ones(4,1);
motor_thrust_log = zeros(4,N);
motor_thrust_log(:,1) = motor_thrust;
motor_saturated = false(1,N);

% Build reference arrays and simulated position/velocity sensor outputs.
[ref_pos, ref_vel, ref_acc, yaw_ref] = build_reference(cfg, scenario, t);
pos_meas = zeros(3,N); vel_meas = zeros(3,N);
pos_meas(:,1) = pos(:,1); vel_meas(:,1) = vel(:,1);

% Rangefinder signals (used only for ROW vertical clearance control).
range_raw = nan(1,N);
range_filtered = nan(1,N);
range_age = zeros(1,N);
range_fallback = false(1,N);
actual_clearance = nan(1,N);
range_filter_state = cfg.range_clearance_target_m;
range_rate_filter = 0;
range_int = 0;
range_buf = zeros(3,1);
range_buf_count = 0;
range_buf_index = 1;
last_range_valid_time = -inf;
last_range_filter = range_filter_state;
baro_fallback_active = false;
baro_fallback_target = initial_pos(3);

% Controller integrators and filtered derivative of body rates.
pos_int = zeros(3,1);
rate_int = zeros(3,1);
omega_dot_filtered = zeros(3,1);
omega_prev = omega(:,1);

% Logs.
attitude_sp = zeros(3,N);
torque_cmd_log = zeros(3,N);
thrust_cmd_log = zeros(1,N);
wind_acc_log = zeros(3,N);
collision_log = false(1,N);

% Create the 3-D scene and a simple animated quadrotor model.
if cfg.animate
    [fig3d, ax3d, drone_group, prop_groups, motor_graphics_pos, ...
        actual_path_handle] = create_scene(cfg, scenario, ref_pos, t);
    % Place the model at its true initial state BEFORE the control loop.
    update_scene(ax3d, drone_group, prop_groups, motor_graphics_pos, ...
        actual_path_handle, pos(:,1), att(:,1), 0, 1);
    figure(fig3d);
    drawnow;
    pause(cfg.startup_display_s);
else
    fig3d = []; ax3d = []; drone_group = []; prop_groups = [];
    motor_graphics_pos = []; actual_path_handle = [];
end
frame_clock = tic;
landed_disarm = false;

for k = 1:N-1
    tk = t(k);

    % 1) Simulated position/velocity sensor: 50 Hz noisy sample-and-hold.
    if mod(k-1, sensor_div) == 0
        pos_meas(:,k) = pos(:,k) + [0.025; 0.025; 0.035].*randn(3,1);
        vel_meas(:,k) = vel(:,k) + [0.035; 0.035; 0.045].*randn(3,1);
    else
        pos_meas(:,k) = pos_meas(:,k-1);
        vel_meas(:,k) = vel_meas(:,k-1);
    end

    % 2) Simulated downward rangefinder for ROW mode.
    canopy_now = canopy_height(pos(1,k),pos(2,k));
    true_clearance = pos(3,k) - canopy_now;
    actual_clearance(k) = true_clearance;
    if strcmp(scenario,'ROW') && mod(k-1,range_div) == 0
        slant_range = true_clearance / max(cos(att(1,k))*cos(att(2,k)),0.5);
        sample_valid = (rand() >= cfg.range_dropout_probability) && ...
                       (slant_range >= cfg.range_min_m) && ...
                       (slant_range <= cfg.range_max_m);
        if sample_valid
            range_sample = slant_range + cfg.range_noise_sigma_m*randn();
            range_sample = min(max(range_sample,cfg.range_min_m),cfg.range_max_m);
            range_buf(range_buf_index) = range_sample;
            range_buf_index = mod(range_buf_index,3)+1;
            range_buf_count = min(range_buf_count+1,3);
            med_range = median(range_buf(1:range_buf_count));
            vertical_range = med_range*cos(att(1,k))*cos(att(2,k));
            prev_filtered = range_filter_state;
            range_filter_state = 0.35*vertical_range + 0.65*range_filter_state;
            rate_est = (range_filter_state-prev_filtered)/range_period;
            rate_est = min(max(rate_est,-2.0),2.0);
            range_rate_filter = 0.30*rate_est + 0.70*range_rate_filter;
            last_range_valid_time = tk;
            range_raw(k) = range_sample;
        end
    end
    range_age(k) = tk-last_range_valid_time;
    range_filtered(k) = range_filter_state;

    % 3) OUTER POSITION LOOP: desired acceleration.
    e_pos = ref_pos(:,k)-pos_meas(:,k);
    e_vel = ref_vel(:,k)-vel_meas(:,k);
    int_candidate = pos_int + e_pos*dt;
    int_candidate = min(max(int_candidate,-cfg.pos_integral_limit), ...
                              cfg.pos_integral_limit);

    a_cmd = ref_acc(:,k) + cfg.Kp_pos.*e_pos + ...
            cfg.Ki_pos.*int_candidate + cfg.Kd_pos.*e_vel;

    % ROW mode does not use an absolute Z-position reference. Its Z loop
    % closes around range-to-canopy instead.
    if strcmp(scenario,'ROW')
        a_cmd(3) = 0;
        int_candidate(3) = pos_int(3);
        if range_age(k) <= cfg.range_timeout_s
            baro_fallback_active = false;
            e_clearance = cfg.range_clearance_target_m-range_filter_state;
            range_int_candidate = range_int + e_clearance*dt;
            range_int_candidate = min(max(range_int_candidate, ...
                -cfg.range_integral_limit),cfg.range_integral_limit);
            az_candidate = cfg.Kp_range*e_clearance + ...
                          cfg.Ki_range*range_int_candidate - ...
                          cfg.Kd_range*range_rate_filter;
            if abs(az_candidate) <= cfg.max_z_acc
                range_int = range_int_candidate;
            end
            a_cmd(3) = cfg.Kp_range*e_clearance + ...
                       cfg.Ki_range*range_int - ...
                       cfg.Kd_range*range_rate_filter;
        else
            % Do not keep controlling from a stale range value. For this
            % simulation, fall back to a fixed absolute-altitude hold.
            if cfg.range_fallback_altitude_hold && ~baro_fallback_active
                baro_fallback_target = pos_meas(3,k);
                baro_fallback_active = true;
            end
            if cfg.range_fallback_altitude_hold
                a_cmd(3) = cfg.Kp_pos(3)*(baro_fallback_target-pos_meas(3,k)) ...
                         - cfg.Kd_pos(3)*vel_meas(3,k);
            else
                a_cmd(3) = 0;
            end
        end
    end

    % Anti-windup for horizontal position integrators when XY acceleration
    % saturates. Keep the integral update only when not pushing farther out.
    xy_norm_candidate = norm(a_cmd(1:2));
    if xy_norm_candidate <= cfg.max_xy_acc
        pos_int(1:2) = int_candidate(1:2);
    end
    if ~strcmp(scenario,'ROW')
        if abs(a_cmd(3)) <= cfg.max_z_acc
            pos_int(3) = int_candidate(3);
        end
    end

    xy_norm = norm(a_cmd(1:2));
    if xy_norm > cfg.max_xy_acc
        a_cmd(1:2) = a_cmd(1:2)*(cfg.max_xy_acc/xy_norm);
    end
    a_cmd(3) = min(max(a_cmd(3),-cfg.max_z_acc),cfg.max_z_acc);

    % 4) Convert requested world acceleration to desired roll/pitch.
    yaw_sp = yaw_ref(k);
    desired_force_world = cfg.mass*[a_cmd(1); a_cmd(2); cfg.g+a_cmd(3)];
    desired_force_norm = max(norm(desired_force_world),1e-6);
    b3_world = desired_force_world/desired_force_norm;
    b3_yaw_frame = [ cos(yaw_sp), sin(yaw_sp), 0; ...
                    -sin(yaw_sp), cos(yaw_sp), 0; ...
                     0,           0,          1]*b3_world;
    roll_sp = asin(min(max(-b3_yaw_frame(2),-1),1));
    pitch_sp = atan2(b3_yaw_frame(1),max(b3_yaw_frame(3),1e-6));
    roll_sp = min(max(roll_sp,-cfg.max_tilt),cfg.max_tilt);
    pitch_sp = min(max(pitch_sp,-cfg.max_tilt),cfg.max_tilt);
    attitude_sp(:,k) = [roll_sp; pitch_sp; yaw_sp];

    % 5) ATTITUDE loop -> desired body rates.
    e_att = attitude_sp(:,k)-att(:,k);
    e_att(3) = atan2(sin(e_att(3)),cos(e_att(3)));
    euler_rate_sp = cfg.Kp_angle.*e_att;
    euler_rate_sp = min(max(euler_rate_sp,-cfg.max_body_rate), ...
                              cfg.max_body_rate);
    phi = att(1,k); theta = att(2,k);
    cos_theta = max(cos(theta),0.2);
    E = [1, sin(phi)*tan(theta), cos(phi)*tan(theta); ...
         0, cos(phi), -sin(phi); ...
         0, sin(phi)/cos_theta, cos(phi)/cos_theta];
    body_rate_sp = E\euler_rate_sp;
    body_rate_sp = min(max(body_rate_sp,-cfg.max_body_rate), ...
                                  cfg.max_body_rate);

    % 6) BODY-RATE PID -> requested torque.
    omega_meas = omega(:,k) + [0.002;0.002;0.003].*randn(3,1);
    omega_dot_raw = (omega_meas-omega_prev)/dt;
    omega_dot_filtered = 0.20*omega_dot_raw + 0.80*omega_dot_filtered;
    omega_prev = omega_meas;
    e_rate = body_rate_sp-omega_meas;
    rate_int_candidate = rate_int+e_rate*dt;
    rate_int_candidate = min(max(rate_int_candidate, ...
        -cfg.rate_integral_limit),cfg.rate_integral_limit);
    torque_candidate = cfg.Kp_rate.*e_rate + ...
                       cfg.Ki_rate.*rate_int_candidate - ...
                       cfg.Kd_rate.*omega_dot_filtered;
    torque = min(max(torque_candidate,-cfg.max_torque),cfg.max_torque);
    for axis_i = 1:3
        if abs(torque_candidate(axis_i)) <= cfg.max_torque(axis_i)
            rate_int(axis_i) = rate_int_candidate(axis_i);
        end
    end
    torque = cfg.Kp_rate.*e_rate + cfg.Ki_rate.*rate_int - ...
             cfg.Kd_rate.*omega_dot_filtered;
    torque = min(max(torque,-cfg.max_torque),cfg.max_torque);
    torque_cmd_log(:,k) = torque;

    % 7) Collective thrust request and four-motor control allocation.
    collective_cmd = desired_force_norm;
    collective_cmd = min(max(collective_cmd,0),4*cfg.max_motor_thrust);
    wrench_cmd = [collective_cmd; torque];
    motor_cmd_raw = alloc\wrench_cmd;
    motor_cmd = min(max(motor_cmd_raw,0),cfg.max_motor_thrust);
    motor_saturated(k) = any(motor_cmd_raw < 0) || ...
                        any(motor_cmd_raw > cfg.max_motor_thrust);

    % Simplified landing/disarm for the TAKEOFF_LAND test.
    if strcmp(scenario,'TAKEOFF_LAND') && tk >= 14.0 && ...
            pos(3,k) <= cfg.ground_contact_z + 0.08 && abs(vel(3,k)) <= 0.25
        landed_disarm = true;
    end
    if landed_disarm
        motor_cmd(:) = 0;
    end

    % First-order motor/ESC response. Motor thrust cannot change instantly.
    motor_alpha = 1-exp(-dt/cfg.motor_tau);
    motor_thrust = motor_thrust + motor_alpha*(motor_cmd-motor_thrust);
    motor_thrust_log(:,k) = motor_thrust;
    thrust_actual = sum(motor_thrust);
    torque_actual = alloc(2:4,:)*motor_thrust;
    thrust_cmd_log(k) = collective_cmd;

    % 8) 6-DOF rigid-body dynamics (world translation + body rotation).
    phi = att(1,k); theta = att(2,k); psi = att(3,k);
    Rx = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi),cos(phi)];
    Ry = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
    Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
    R_body_to_world = Rz*Ry*Rx;
    wind_velocity = [0.35*sin(0.35*tk); 0.25*cos(0.22*tk); 0];
    wind_accel = [0.035*sin(1.3*tk); 0.030*cos(0.9*tk); 0];
    if tk >= cfg.wind_gust_start_s && tk <= cfg.wind_gust_end_s
        wind_accel = wind_accel+cfg.wind_gust_accel;
    end
    wind_acc_log(:,k) = wind_accel;
    drag_accel = -(cfg.linear_drag/cfg.mass).*(vel(:,k)-wind_velocity);
    acc_world = R_body_to_world*[0;0;thrust_actual]/cfg.mass + ...
                [0;0;-cfg.g] + drag_accel + wind_accel;
    vel(:,k+1) = vel(:,k)+acc_world*dt;
    pos(:,k+1) = pos(:,k)+vel(:,k+1)*dt;

    omega_dot = cfg.I\(torque_actual-cross(omega(:,k),cfg.I*omega(:,k)) ...
                         -cfg.rate_damping.*omega(:,k));
    omega(:,k+1) = omega(:,k)+omega_dot*dt;
    euler_rate = E*omega(:,k+1);
    att(:,k+1) = att(:,k)+euler_rate*dt;
    att(3,k+1) = atan2(sin(att(3,k+1)),cos(att(3,k+1)));

    % Ground/canopy collision boundary. This is a simple contact clamp,
    % not a detailed impact or landing-gear model.
    if strcmp(scenario,'TAKEOFF_LAND')
        z_floor = cfg.ground_contact_z;
    else
        z_floor = 0;
    end
    if pos(3,k+1) < z_floor
        pos(3,k+1) = z_floor;
        % A gentle touchdown in the dedicated landing window is expected,
        % not a collision. Early/fast contact is still flagged.
        expected_touchdown = strcmp(scenario,'TAKEOFF_LAND') && ...
            tk >= 13.5 && abs(vel(3,k)) <= 0.6;
        if ~expected_touchdown
            collision_log(k+1) = true;
        end
        vel(3,k+1) = max(vel(3,k+1),0);
    end
    if strcmp(scenario,'ROW')
        canopy_next = canopy_height(pos(1,k+1),pos(2,k+1));
        if pos(3,k+1) < canopy_next+0.05
            pos(3,k+1) = canopy_next+0.05;
            vel(3,k+1) = max(vel(3,k+1),0);
            collision_log(k+1) = true;
        end
    end

    % Carry sample-and-hold sensor data to next integration step.
    if mod(k, sensor_div) ~= 0
        pos_meas(:,k+1) = pos_meas(:,k);
        vel_meas(:,k+1) = vel_meas(:,k);
    end
    range_age(k+1) = tk+dt-last_range_valid_time;
    range_filtered(k+1) = range_filter_state;
    actual_clearance(k+1) = pos(3,k+1)-canopy_height(pos(1,k+1),pos(2,k+1));
    range_fallback(k+1) = strcmp(scenario,'ROW') && ...
                           (range_age(k+1)>cfg.range_timeout_s);

    % 9) Animation, decoupled from integration frequency.
    if cfg.animate && (mod(k,anim_div)==0 || k==N-1)
        update_scene(ax3d, drone_group, prop_groups, motor_graphics_pos, ...
            actual_path_handle, pos(:,1:k+1), att(:,k+1), tk, k);
        drawnow;  % Keep the take-off frames visible.
        if cfg.realtime_playback
            frame_period = 1/cfg.animate_fps;
            frame_elapsed = toc(frame_clock);
            if frame_elapsed < frame_period
                pause(frame_period-frame_elapsed);
            end
        end
        frame_clock = tic;
    end
end

% Complete final samples for reports/plots.
pos_meas(:,N) = pos_meas(:,N-1);
vel_meas(:,N) = vel_meas(:,N-1);
attitude_sp(:,N) = attitude_sp(:,N-1);
motor_thrust_log(:,N) = motor_thrust;
actual_clearance(N) = pos(3,N)-canopy_height(pos(1,N),pos(2,N));
range_fallback(N) = strcmp(scenario,'ROW') && range_age(N)>cfg.range_timeout_s;

% Compute performance metrics. In ROW mode, vertical performance is
% measured relative to the simulated canopy, not sea-level altitude.
if strcmp(scenario,'ROW')
    z_error = cfg.range_clearance_target_m-actual_clearance;
else
    z_error = ref_pos(3,:)-pos(3,:);
end
xy_error = ref_pos(1:2,:)-pos(1:2,:);
rmse_xy = sqrt(mean(sum(xy_error.^2,1)));
rmse_z = sqrt(mean(z_error.^2));
max_tilt_deg = max(sqrt(att(1,:).^2+att(2,:).^2))*180/pi;
motor_saturation_percent = 100*mean(motor_saturated);
max_motor_thrust = max(motor_thrust_log(:));
range_fallback_percent = 100*mean(range_fallback);

% Very coarse model-only screening flag. It does not imply real-flight safety.
finite_state = all(isfinite(pos(:))) && all(isfinite(att(:))) && ...
               all(isfinite(motor_thrust_log(:)));
if finite_state && max_tilt_deg < 40 && motor_saturation_percent < 10 && ...
        rmse_xy < 0.45 && rmse_z < 0.35 && ~any(collision_log)
    model_flag = 'MODEL SCREEN: OK';
else
    model_flag = 'MODEL SCREEN: REVIEW/TUNE';
end

fprintf('\n--- %s ---\n',scenario);
fprintf('RMSE XY                 : %.3f m\n',rmse_xy);
if strcmp(scenario,'ROW')
    fprintf('RMSE canopy clearance   : %.3f m (target %.2f m)\n', ...
        rmse_z,cfg.range_clearance_target_m);
else
    fprintf('RMSE altitude           : %.3f m\n',rmse_z);
end
fprintf('Maximum combined tilt   : %.1f deg\n',max_tilt_deg);
fprintf('Motor saturation        : %.2f %% of samples\n',motor_saturation_percent);
fprintf('Maximum motor thrust    : %.2f N\n',max_motor_thrust);
if strcmp(scenario,'ROW')
    fprintf('Range fallback          : %.2f %% of samples\n',range_fallback_percent);
end
fprintf('Model-only screening    : %s\n',model_flag);
if any(collision_log)
    fprintf('WARNING: simulated ground/canopy contact occurred.\n');
end

plot_results(cfg,scenario,t,ref_pos,pos,att,attitude_sp, ...
    motor_thrust_log,range_filtered,range_age,actual_clearance, ...
    range_fallback,z_error);

out.scenario = scenario;
out.t = t;
out.pos = pos;
out.vel = vel;
out.att = att;
out.omega = omega;
out.ref_pos = ref_pos;
out.ref_vel = ref_vel;
out.ref_acc = ref_acc;
out.attitude_sp = attitude_sp;
out.motor_thrust = motor_thrust_log;
out.motor_saturated = motor_saturated;
out.range_raw = range_raw;
out.range_filtered = range_filtered;
out.range_age = range_age;
out.range_fallback = range_fallback;
out.actual_clearance = actual_clearance;
out.collision = collision_log;
out.rmse_xy = rmse_xy;
out.rmse_z = rmse_z;
out.max_tilt_deg = max_tilt_deg;
out.motor_saturation_percent = motor_saturation_percent;
out.max_motor_thrust = max_motor_thrust;
out.range_fallback_percent = range_fallback_percent;
out.model_flag = model_flag;

if cfg.animate && isgraphics(fig3d)
    figure(fig3d);
end
end

function [ref_pos,ref_vel,ref_acc,yaw_ref] = build_reference(cfg,scenario,t)
N = numel(t);
ref_pos = zeros(3,N); ref_vel = zeros(3,N); ref_acc = zeros(3,N);
yaw_ref = zeros(1,N);

if strcmp(scenario,'TAKEOFF_LAND')
    z0 = cfg.ground_contact_z; z_hover = 1.60;
    for k = 1:N
        tk = t(k);
        if tk < 2
            [z,zd,zdd] = smooth_move(tk,0,2,z0,z0);  % Ground idle.
        elseif tk < 5
            [z,zd,zdd] = smooth_move(tk,2,3,z0,z_hover);  % Take-off.
        elseif tk < 11
            [z,zd,zdd] = smooth_move(tk,5,6,z_hover,z_hover);  % Hover.
        elseif tk < 14
            [z,zd,zdd] = smooth_move(tk,11,3,z_hover,z0);  % Landing.
        else
            [z,zd,zdd] = smooth_move(tk,14,4,z0,z0);  % Landed.
        end
        ref_pos(:,k) = [0;0;z];
        ref_vel(:,k) = [0;0;zd];
        ref_acc(:,k) = [0;0;zdd];
    end

elseif strcmp(scenario,'WAYPOINT')
    waypoints = [0.0,0.0,0.20; ...
                 2.5,0.0,1.40; ...
                 2.5,2.5,1.60; ...
                 0.0,2.5,1.30; ...
                 0.0,0.0,0.80];
    segment_start = 1.0;
    segment_duration = 5.0;
    for k = 1:N
        tk = t(k);
        if tk < segment_start
            ref_pos(:,k) = waypoints(1,:)';
            continue;
        end
        segment = floor((tk-segment_start)/segment_duration)+1;
        segment = min(max(segment,1),size(waypoints,1)-1);
        t0 = segment_start+(segment-1)*segment_duration;
        [s,sd,sdd] = smoothstep5((tk-t0)/segment_duration);
        delta = (waypoints(segment+1,:)-waypoints(segment,:))';
        ref_pos(:,k) = waypoints(segment,:)'+delta*s;
        ref_vel(:,k) = delta*(sd/segment_duration);
        ref_acc(:,k) = delta*(sdd/segment_duration^2);
        if norm(ref_vel(1:2,k)) > 0.05
            yaw_ref(k) = atan2(ref_vel(2,k),ref_vel(1,k));
        elseif k > 1
            yaw_ref(k) = yaw_ref(k-1);
        end
    end

elseif strcmp(scenario,'ROW')
    row_length = 10.0;
    row_duration = 18.0;
    row_start = 1.0;
    for k = 1:N
        tk = t(k);
        [s,sd,sdd] = smoothstep5((tk-row_start)/row_duration);
        x = row_length*s;
        xd = row_length*sd/row_duration;
        xdd = row_length*sdd/row_duration^2;
        y = 0.35*sin(0.25*x);
        dydx = 0.35*0.25*cos(0.25*x);
        d2ydx2 = -0.35*0.25^2*sin(0.25*x);
        yd = dydx*xd;
        ydd = d2ydx2*xd^2+dydx*xdd;
        z = canopy_height(x,y)+cfg.range_clearance_target_m;
        ref_pos(:,k) = [x;y;z];
        ref_vel(:,k) = [xd;yd;0];
        ref_acc(:,k) = [xdd;ydd;0];
        if hypot(xd,yd) > 0.05
            yaw_ref(k) = atan2(yd,xd);
        elseif k > 1
            yaw_ref(k) = yaw_ref(k-1);
        end
    end
else
    error('Unknown scenario in build_reference.');
end
end

function [value,velocity,acceleration] = smooth_move(t,t0,duration,start_value,end_value)
if duration <= 0 || t <= t0
    s = 0; sd = 0; sdd = 0;
elseif t >= t0+duration
    s = 1; sd = 0; sdd = 0;
else
    [s,sd,sdd] = smoothstep5((t-t0)/duration);
    sd = sd/duration;
    sdd = sdd/duration^2;
end
delta = end_value-start_value;
value = start_value+delta*s;
velocity = delta*sd;
acceleration = delta*sdd;
end

function [s,sd,sdd] = smoothstep5(u)
u = min(max(u,0),1);
s = 10*u^3-15*u^4+6*u^5;
sd = 30*u^2-60*u^3+30*u^4;
sdd = 60*u-180*u^2+120*u^3;
end

function h = canopy_height(x,y)
% Synthetic smooth canopy-top profile for simulation only.
h = 1.25 + 0.12*sin(0.45*x) + 0.05*sin(1.4*x) + 0.08*cos(0.35*y);
end

function [fig,ax,drone_group,prop_groups,motor_pos,h_path] = ...
        create_scene(cfg,scenario,ref_pos,t)
fig = figure('Name',['Quadrotor PID test - ' scenario], ...
    'Color','w','Position',[80 70 1050 760]);
ax = axes('Parent',fig);
hold(ax,'on'); grid(ax,'on'); box(ax,'on');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
title(ax,strrep(scenario,'_',' '));

if strcmp(scenario,'ROW')
    [Xc,Yc] = meshgrid(0:0.25:10,-1.2:0.15:1.2);
    Zc = canopy_height(Xc,Yc);
    surf(ax,Xc,Yc,Zc,'FaceColor',[0.30 0.62 0.25], ...
        'EdgeColor','none','FaceAlpha',0.75);
    xlim(ax,[-1 11]); ylim(ax,[-2.5 2.5]); zlim(ax,[0 4]);
else
    [Xg,Yg] = meshgrid(-2:1:5,-2:1:5);
    surf(ax,Xg,Yg,zeros(size(Xg)),'FaceColor',[0.78 0.84 0.78], ...
        'EdgeColor','none','FaceAlpha',0.45);
    xlim(ax,[-2 5]); ylim(ax,[-2 5]); zlim(ax,[0 4]);
end
plot3(ax,ref_pos(1,:),ref_pos(2,:),ref_pos(3,:), ...
    'r--','LineWidth',1.8,'DisplayName','Reference');
h_path = plot3(ax,nan,nan,nan,'b-','LineWidth',2.0, ...
    'DisplayName','Actual');
legend(ax,'Location','northeast');
axis(ax,'vis3d'); daspect(ax,[1 1 1]); view(ax,3);

arm = cfg.arm_length;
angles = deg2rad([45 135 225 315]);
motor_pos = [arm*cos(angles(:)), arm*sin(angles(:)), zeros(4,1)];
drone_group = hgtransform('Parent',ax);
[V,F] = create_box(0.13,0.16,0.035);
patch('Parent',drone_group,'Vertices',V,'Faces',F, ...
    'FaceColor',[0.18 0.18 0.20],'EdgeColor','none');
for i = 1:4
    a = angles(i);
    Rz = [cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];
    local_arm = [0 -0.014 0; arm -0.014 0; arm 0.014 0; 0 0.014 0; ...
                 0 -0.014 -0.015; arm -0.014 -0.015; ...
                 arm 0.014 -0.015; 0 0.014 -0.015];
    arm_vertices = (Rz*local_arm')';
    faces = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    patch('Parent',drone_group,'Vertices',arm_vertices,'Faces',faces, ...
        'FaceColor',[0.30 0.30 0.32],'EdgeColor','none');
    [Xm,Ym,Zm] = cylinder(0.022,16);
    Zm = 0.04*Zm-0.02;
    surf('Parent',drone_group,'XData',Xm+motor_pos(i,1), ...
        'YData',Ym+motor_pos(i,2),'ZData',Zm, ...
        'FaceColor',[0.65 0.65 0.68],'EdgeColor','none');
end
prop_groups = gobjects(1,4);
for i = 1:4
    prop_groups(i) = hgtransform('Parent',drone_group);
    [Vp,Fp] = create_box(0.22,0.025,0.004);
    patch('Parent',prop_groups(i),'Vertices',Vp,'Faces',Fp, ...
        'FaceColor',[0.88 0.18 0.16],'EdgeColor','none');
end
end

function update_scene(ax,drone_group,prop_groups,motor_pos,h_path,path,att,tk,k)
set(h_path,'XData',path(1,:),'YData',path(2,:),'ZData',path(3,:));
T = makehgtform('translate',path(:,end)') * ...
    makehgtform('zrotate',att(3)) * ...
    makehgtform('yrotate',att(2)) * ...
    makehgtform('xrotate',att(1));
set(drone_group,'Matrix',T);
for i = 1:4
    spin = (-1)^i;
    angle = mod(900*tk*spin,360)*pi/180;
    Tp = makehgtform('translate',motor_pos(i,:)+[0 0 0.025])* ...
         makehgtform('zrotate',angle);
    set(prop_groups(i),'Matrix',Tp);
end
%camtarget(ax,path(:,end)');
%campos(ax,path(:,end)'+[-3.5 -4.5 3.0]);
end

function plot_results(cfg,scenario,t,ref_pos,pos,att,attitude_sp, ...
    motor_thrust,range_filtered,range_age,actual_clearance, ...
    range_fallback,z_error)
figure('Name',['Tracking - ' scenario],'Color','w', ...
    'Position',[120 90 1000 760]);
subplot(3,1,1);
plot(t,ref_pos(1,:),'r--',t,pos(1,:),'b','LineWidth',1.1);
grid on; ylabel('X (m)'); title([scenario ' - position tracking']);
legend('Reference','Actual','Location','best');
subplot(3,1,2);
plot(t,ref_pos(2,:),'r--',t,pos(2,:),'b','LineWidth',1.1);
grid on; ylabel('Y (m)');
subplot(3,1,3);
if strcmp(scenario,'ROW')
    plot(t,cfg.range_clearance_target_m*ones(size(t)),'r--', ...
         t,actual_clearance,'b','LineWidth',1.1);
    ylabel('Canopy clearance (m)');
    legend('Target clearance','Actual clearance','Location','best');
else
    plot(t,ref_pos(3,:),'r--',t,pos(3,:),'b','LineWidth',1.1);
    ylabel('Z (m)');
    legend('Reference','Actual','Location','best');
end
xlabel('Time (s)'); grid on;

figure('Name',['Attitude - ' scenario],'Color','w', ...
    'Position',[150 110 950 650]);
labels = {'Roll (deg)','Pitch (deg)','Yaw (deg)'};
for i = 1:3
    subplot(3,1,i);
    plot(t,attitude_sp(i,:)*180/pi,'r--', ...
         t,att(i,:)*180/pi,'b','LineWidth',1.1);
    grid on; ylabel(labels{i});
    if i==1, title('Attitude setpoint vs actual'); legend('Setpoint','Actual'); end
    if i==3, xlabel('Time (s)'); end
end

figure('Name',['Motor thrust - ' scenario],'Color','w', ...
    'Position',[180 130 950 500]);
plot(t,motor_thrust(1,:),'LineWidth',1.0); hold on;
plot(t,motor_thrust(2,:),'LineWidth',1.0);
plot(t,motor_thrust(3,:),'LineWidth',1.0);
plot(t,motor_thrust(4,:),'LineWidth',1.0);
yline(cfg.max_motor_thrust,'k--','Motor max (assumed)');
grid on; xlabel('Time (s)'); ylabel('Motor thrust (N)');
title('Individual motor thrust demand (model)');
legend('M1','M2','M3','M4','Max thrust','Location','best');

if strcmp(scenario,'ROW')
    figure('Name','Rangefinder diagnostics - ROW','Color','w', ...
        'Position',[210 150 950 520]);
    subplot(2,1,1);
    plot(t,range_filtered,'b','LineWidth',1.0); hold on;
    yline(cfg.range_clearance_target_m,'r--','Target');
    grid on; ylabel('Filtered vertical range (m)');
    title('Simulated downward rangefinder');
    subplot(2,1,2);
    plot(t,range_age,'k','LineWidth',1.0); hold on;
    yline(cfg.range_timeout_s,'r--','Timeout');
    stairs(t,double(range_fallback),'m','LineWidth',1.0);
    grid on; xlabel('Time (s)'); ylabel('Range age (s) / fallback');
    legend('Range age','Timeout','Fallback active','Location','best');
end
end

function [V,F] = create_box(w,l,h)
V = [-w/2 -l/2 -h/2; w/2 -l/2 -h/2; w/2 l/2 -h/2; -w/2 l/2 -h/2; ...
     -w/2 -l/2 h/2; w/2 -l/2 h/2; w/2 l/2 h/2; -w/2 l/2 h/2];
F = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
end
