% Simulate the proportional guidance algorithm implemented in src/dart_py/guidance.py.
%
% Scenario:
%   - Launch pitch angle: 35 deg
%   - Target bearing: 7.8 deg to the right-front
%   - Launch/closing speed: solved so an uncontrolled projectile can hit the
%     stationary target on the same horizontal plane.
%   - Target forward perpendicular distance: 24.5 m
%   - Projectile and target are on the same horizontal plane before launch.
%   - The projectile yaw points at the target before launch.
%   - Target lateral motion: 0 mm at launch, then moves during the first
%     600 ms to a random position in [-280 mm, +280 mm] and remains still.
%   - A 140 mm x 140 mm square centered on the target is judged as a hit.
%   - The camera can detect the target only during the descending phase.
%
% The trajectory is propagated with gravity. The default speed is computed
% from the same-plane ballistic range equation for the nominal static target.
%
% Run from MATLAB:
%   run('matlab/simulate_guidance_algorithm.m')

clear;
clc;
close all;

rng('shuffle');

cfg = default_guidance_config();
scene = default_launch_scene();
cfg.closing_velocity = scene.launch_speed_mps;

dt = 0.01;
t_end = scene.sim_time_s;
t = 0:dt:t_end;
if t(end) < t_end
    t = [t, t_end];
end
n = numel(t);

[true_x, true_y, scene_log] = build_launch_scene_pixels(t, cfg, scene);
hit_result = evaluate_hit_result(t, scene_log, scene);

noise_sigma_px = scene.measurement_noise_sigma_px;
meas_x = true_x + noise_sigma_px * randn(size(true_x));
meas_y = true_y + noise_sigma_px * randn(size(true_y));

valid_pixel = true_x >= 0.0 & true_x <= cfg.image_width & true_y >= 0.0 & true_y <= cfg.image_height;
valid_meas = scene_log.camera_enabled & valid_pixel & ...
    meas_x >= 0.0 & meas_x <= cfg.image_width & ...
    meas_y >= 0.0 & meas_y <= cfg.image_height;
meas_x(~valid_meas) = -1.0;
meas_y(~valid_meas) = -1.0;

roll_deg = zeros(size(t));
roll_rad = deg2rad_local(roll_deg);

gyro_b = zeros(3, n);

state = init_guidance_state();
sim = init_sim_log(n);

for k = 1:n
    [out, state] = guidance_step( ...
        meas_x(k), ...
        meas_y(k), ...
        dt, ...
        roll_rad(k), ...
        gyro_b(:, k), ...
        state, ...
        cfg);

    sim.detected(k) = out.detected;
    sim.pixel_error_x(k) = out.pixel_error_x;
    sim.pixel_error_y(k) = out.pixel_error_y;
    sim.yaw_angle(k) = out.yaw_los_angle_rad;
    sim.pitch_angle(k) = out.pitch_los_angle_rad;
    sim.yaw_rate(k) = out.yaw_los_rate_rad_s;
    sim.pitch_rate(k) = out.pitch_los_rate_rad_s;
    sim.raw_yaw_angle(k) = out.raw_yaw_los_angle_rad;
    sim.raw_pitch_angle(k) = out.raw_pitch_los_angle_rad;
    sim.raw_yaw_rate(k) = out.raw_yaw_los_rate_rad_s;
    sim.raw_pitch_rate(k) = out.raw_pitch_los_rate_rad_s;
    sim.yaw_overload_g(k) = out.yaw_overload_g;
    sim.pitch_overload_g(k) = out.pitch_overload_g;
end

valid = sim.detected > 0.5;
meas_x_plot = meas_x;
meas_y_plot = meas_y;
meas_x_plot(~valid) = NaN;
meas_y_plot(~valid) = NaN;

fprintf('Guidance simulation complete.\n');
fprintf('Samples: %d, dt: %.3f s, valid detections: %d\n', n, dt, nnz(valid));
fprintf('Launch pitch/yaw: %.1f deg / %.1f deg right, speed: %.2f m/s\n', ...
    scene.launch_pitch_deg, scene.launch_yaw_deg, scene.launch_speed_mps);
fprintf('Target forward distance: %.2f m, horizontal range: %.2f m, move time: %.3f s\n', ...
    scene.target_forward_distance_m, scene.target_horizontal_range_m, scene.target_move_time_s);
fprintf('Nominal target position: forward %.2f m, right %.2f m, up %.2f m\n', ...
    scene_log.target_nominal_world_m(1), scene_log.target_nominal_world_m(2), ...
    -scene_log.target_nominal_world_m(3));
fprintf('Apex time: %.3f s, static-target hit time: %.3f s\n', ...
    scene.apex_time_s, scene.nominal_static_hit_time_s);
fprintf('Same-plane ballistic range: %.2f m, range error: %.3f mm\n', ...
    scene.same_plane_ballistic_range_m, ...
    (scene.same_plane_ballistic_range_m - scene.target_horizontal_range_m) * 1000.0);
fprintf('Target lateral start/final: %.0f mm -> %.0f mm\n', ...
    scene.target_initial_lateral_m * 1000.0, scene.target_final_lateral_m * 1000.0);
fprintf('Hit box: %.0f mm x %.0f mm\n', ...
    scene.hit_box_side_m * 1000.0, scene.hit_box_side_m * 1000.0);
fprintf('Impact time: %.3f s, judgement: %s\n', ...
    hit_result.impact_time_s, hit_result.judgement);
fprintf('Miss components: range %.1f mm, lateral %.1f mm, height %.1f mm\n', ...
    hit_result.range_error_m * 1000.0, ...
    hit_result.lateral_error_m * 1000.0, ...
    hit_result.height_error_m * 1000.0);
fprintf('Center miss distance: %.1f mm, outside-box miss distance: %.1f mm\n', ...
    hit_result.center_miss_m * 1000.0, hit_result.outside_box_miss_m * 1000.0);
fprintf('Measurement noise sigma: %.2f px\n', noise_sigma_px);
fprintf('Yaw overload range:   [%.3f, %.3f] g\n', min(sim.yaw_overload_g), max(sim.yaw_overload_g));
fprintf('Pitch overload range: [%.3f, %.3f] g\n', min(sim.pitch_overload_g), max(sim.pitch_overload_g));
fprintf('Output image: %s\n', make_output_path('guidance_simulation_result.png'));
fprintf('Flight/hit image: %s\n', make_output_path('flight_hit_result.png'));

plot_simulation_result(t, true_x, true_y, meas_x_plot, meas_y_plot, scene_log, sim, cfg, scene);
plot_flight_hit_result(t, scene_log, scene, hit_result);
save_simulation_data(t, true_x, true_y, meas_x, meas_y, roll_deg, gyro_b, scene_log, sim, cfg, scene, hit_result);

function cfg = default_guidance_config()
    cfg.G = 9.80665;
    cfg.EPS = 1e-9;

    cfg.image_width = 640.0;
    cfg.image_height = 480.0;
    cfg.fov_x_deg = 60.0;
    cfg.fov_y_deg = 45.0;
    cfg.cx = cfg.image_width * 0.5;
    cfg.cy = cfg.image_height * 0.5;
    cfg.fx = focal_length(cfg.image_width, cfg.fov_x_deg);
    cfg.fy = focal_length(cfg.image_height, cfg.fov_y_deg);

    % Camera x-right/y-down/z-forward -> body x-forward/y-right/z-down.
    cfg.R_bc = [
        0.0, 0.0, 1.0;
        1.0, 0.0, 0.0;
        0.0, 1.0, 0.0
    ];

    cfg.navigation_ratio = 3.0;
    cfg.closing_velocity = 15.0;
    cfg.position_to_rate_gain = 0.0;
    cfg.rate_filter_alpha = NaN;
    cfg.use_kalman_filter = true;

    cfg.kalman_angle_variance = 0.05;
    cfg.kalman_rate_variance = 1.0;
    cfg.kalman_process_angle_variance = 0.0001;
    cfg.kalman_process_rate_variance = 0.02;
    cfg.kalman_measurement_angle_variance = 0.0025;
    cfg.kalman_measurement_rate_variance = 10.0;

    cfg.max_overload_g = 6.0;
    cfg.roll_compensation = true;
    cfg.roll_sign = 1.0;
end

function scene = default_launch_scene()
    scene.launch_pitch_deg = 35.0;
    scene.target_bearing_right_deg = 7.8;
    scene.launch_yaw_deg = scene.target_bearing_right_deg;
    scene.target_forward_distance_m = 24.5;
    scene.target_lateral_limit_m = 0.280;
    scene.target_move_time_s = 0.600;
    scene.target_initial_lateral_m = 0.0;
    scene.target_final_lateral_m = (2.0 * rand() - 1.0) * scene.target_lateral_limit_m;
    scene.hit_box_side_m = 0.140;
    scene.measurement_noise_sigma_px = 2.0;

    % Gravity is enabled so that camera detection can be gated by descent.
    scene.use_gravity = true;
    scene.gravity_mps2 = 9.80665;
    scene.camera_only_descending = true;
    scene.camera_aligns_with_velocity = true;

    pitch_rad = deg2rad_local(scene.launch_pitch_deg);
    yaw_rad = deg2rad_local(scene.launch_yaw_deg);
    scene.target_right_distance_m = scene.target_forward_distance_m * tan(yaw_rad);
    scene.target_horizontal_range_m = ...
        scene.target_forward_distance_m / cos(yaw_rad);
    scene.launch_speed_mps = required_same_plane_speed( ...
        scene.target_horizontal_range_m, ...
        scene.launch_pitch_deg, ...
        scene.gravity_mps2);
    scene.apex_time_s = scene.launch_speed_mps * sin(pitch_rad) / scene.gravity_mps2;
    scene.same_plane_flight_time_s = 2.0 * scene.apex_time_s;
    scene.nominal_static_hit_time_s = scene.same_plane_flight_time_s;
    scene.same_plane_ballistic_range_m = ...
        scene.launch_speed_mps * cos(pitch_rad) * scene.same_plane_flight_time_s;
    scene.sim_time_s = scene.same_plane_flight_time_s;
end

function speed_mps = required_same_plane_speed(horizontal_range_m, launch_pitch_deg, gravity_mps2)
    pitch_rad = deg2rad_local(launch_pitch_deg);
    range_factor = sin(2.0 * pitch_rad);
    if range_factor <= 0.0
        error('Launch pitch must satisfy 0 < pitch < 90 deg for same-plane range solving.');
    end
    speed_mps = sqrt(horizontal_range_m * gravity_mps2 / range_factor);
end

function hit_result = evaluate_hit_result(t, scene_log, scene)
    impact_time_s = scene.nominal_static_hit_time_s;
    projectile_w = interp_vector(t, scene_log.projectile_world_m, impact_time_s);
    target_w = interp_vector(t, scene_log.target_world_m, impact_time_s);

    yaw = deg2rad_local(scene.launch_yaw_deg);
    range_axis_w = [cos(yaw); sin(yaw); 0.0];
    lateral_axis_w = [-sin(yaw); cos(yaw); 0.0];

    error_w = projectile_w - target_w;
    range_error_m = dot(error_w, range_axis_w);
    lateral_error_m = dot(error_w, lateral_axis_w);
    height_error_m = -error_w(3);

    half_side_m = scene.hit_box_side_m * 0.5;
    range_outside_m = max(abs(range_error_m) - half_side_m, 0.0);
    lateral_outside_m = max(abs(lateral_error_m) - half_side_m, 0.0);
    is_hit = abs(range_error_m) <= half_side_m && abs(lateral_error_m) <= half_side_m;

    if is_hit
        judgement = 'HIT';
    else
        judgement = 'MISS';
    end

    hit_result.impact_time_s = impact_time_s;
    hit_result.projectile_world_m = projectile_w;
    hit_result.target_world_m = target_w;
    hit_result.error_world_m = error_w;
    hit_result.range_error_m = range_error_m;
    hit_result.lateral_error_m = lateral_error_m;
    hit_result.height_error_m = height_error_m;
    hit_result.center_miss_m = sqrt(range_error_m * range_error_m + lateral_error_m * lateral_error_m);
    hit_result.outside_box_miss_m = sqrt(range_outside_m * range_outside_m + lateral_outside_m * lateral_outside_m);
    hit_result.half_side_m = half_side_m;
    hit_result.is_hit = is_hit;
    hit_result.judgement = judgement;
end

function value = interp_vector(t, values, query_t)
    value = interp1(t(:), values.', query_t, 'linear', 'extrap').';
end

function [pixel_x, pixel_y, log_data] = build_launch_scene_pixels(t, cfg, scene)
    n = numel(t);
    pixel_x = -ones(1, n);
    pixel_y = -ones(1, n);

    launch_R_wb = body_to_world_rotation(scene.launch_yaw_deg, scene.launch_pitch_deg);

    target_lateral = target_lateral_motion(t, scene);
    target_nominal_w = [
        scene.target_forward_distance_m;
        scene.target_right_distance_m;
        0.0
    ];
    target_w = repmat(target_nominal_w, 1, n) + launch_R_wb(:, 2) * target_lateral;

    initial_velocity_w = launch_R_wb(:, 1) * scene.launch_speed_mps;
    projectile_w = initial_velocity_w * t;
    velocity_w = initial_velocity_w * ones(1, n);

    if scene.use_gravity
        t2 = t .* t;
        projectile_w(3, :) = projectile_w(3, :) + 0.5 * scene.gravity_mps2 * t2;
        velocity_w(3, :) = velocity_w(3, :) + scene.gravity_mps2 * t;
    end

    rel_w = target_w - projectile_w;
    rel_b = zeros(3, n);
    camera_enabled = true(1, n);
    body_pitch_deg = zeros(1, n);

    for k = 1:n
        if scene.camera_aligns_with_velocity
            R_wb = body_to_world_from_velocity(velocity_w(:, k), scene.launch_yaw_deg);
        else
            R_wb = launch_R_wb;
        end
        R_bw = R_wb.';
        rel_b(:, k) = R_bw * rel_w(:, k);
        body_pitch_deg(k) = velocity_pitch_deg(velocity_w(:, k));
        camera_enabled(k) = ~scene.camera_only_descending || velocity_w(3, k) > 0.0;

        if rel_b(1, k) <= 0.0
            continue;
        end
        if ~camera_enabled(k)
            continue;
        end
        [pixel_x(k), pixel_y(k)] = body_vector_to_pixel(rel_b(:, k), cfg);
    end

    log_data.target_lateral_m = target_lateral;
    log_data.target_world_m = target_w;
    log_data.projectile_world_m = projectile_w;
    log_data.velocity_world_mps = velocity_w;
    log_data.vertical_velocity_down_mps = velocity_w(3, :);
    log_data.body_pitch_deg = body_pitch_deg;
    log_data.camera_enabled = camera_enabled;
    log_data.target_nominal_world_m = target_nominal_w;
    log_data.launch_R_wb = launch_R_wb;
    log_data.rel_b_m = rel_b;
    log_data.range_m = sqrt(sum(rel_b .* rel_b, 1));
end

function target_y = target_lateral_motion(t, scene)
    tau = clamp(t / scene.target_move_time_s, 0.0, 1.0);
    blend = tau .* tau .* (3.0 - 2.0 * tau);
    target_y = scene.target_initial_lateral_m + ...
        (scene.target_final_lateral_m - scene.target_initial_lateral_m) .* blend;
end

function R_wb = body_to_world_rotation(yaw_deg, pitch_deg)
    yaw = deg2rad_local(yaw_deg);
    pitch = deg2rad_local(pitch_deg);
    cp = cos(pitch);
    sp = sin(pitch);
    cy = cos(yaw);
    sy = sin(yaw);

    x_axis_w = [cp * cy; cp * sy; -sp];
    y_axis_w = [-sy; cy; 0.0];
    z_axis_w = [sp * cy; sp * sy; cp];
    R_wb = [x_axis_w, y_axis_w, z_axis_w];
end

function R_wb = body_to_world_from_velocity(velocity_w, fallback_yaw_deg)
    speed = sqrt(sum(velocity_w .* velocity_w));
    if speed < 1e-9
        R_wb = body_to_world_rotation(fallback_yaw_deg, 0.0);
        return;
    end

    x_axis_w = velocity_w / speed;
    world_down = [0.0; 0.0; 1.0];
    y_axis_w = cross(world_down, x_axis_w);
    y_norm = sqrt(sum(y_axis_w .* y_axis_w));
    if y_norm < 1e-9
        yaw = deg2rad_local(fallback_yaw_deg);
        y_axis_w = [-sin(yaw); cos(yaw); 0.0];
    else
        y_axis_w = y_axis_w / y_norm;
    end
    z_axis_w = cross(x_axis_w, y_axis_w);
    z_axis_w = z_axis_w / sqrt(sum(z_axis_w .* z_axis_w));
    R_wb = [x_axis_w, y_axis_w, z_axis_w];
end

function pitch_deg = velocity_pitch_deg(velocity_w)
    horizontal_speed = sqrt(velocity_w(1) * velocity_w(1) + velocity_w(2) * velocity_w(2));
    pitch_deg = rad2deg_local(atan2(-velocity_w(3), horizontal_speed));
end

function [pixel_x, pixel_y] = body_vector_to_pixel(vector_b, cfg)
    % guidance.py maps camera [x-right, y-down, z-forward] to body
    % [x-forward, y-right, z-down], so camera x/z = body y/x.
    pixel_x = cfg.cx + cfg.fx * vector_b(2) / vector_b(1);
    pixel_y = cfg.cy + cfg.fy * vector_b(3) / vector_b(1);
end

function state = init_guidance_state()
    state.last_los_b = [];
    state.filtered_yaw_dot = 0.0;
    state.filtered_pitch_dot = 0.0;
    state.has_filtered_rate = false;
    state.yaw_filter = init_axis_filter();
    state.pitch_filter = init_axis_filter();
end

function filter = init_axis_filter()
    filter.initialized = false;
    filter.x = zeros(2, 1);
    filter.P = eye(2);
end

function sim = init_sim_log(n)
    sim.detected = zeros(1, n);
    sim.pixel_error_x = zeros(1, n);
    sim.pixel_error_y = zeros(1, n);
    sim.yaw_angle = zeros(1, n);
    sim.pitch_angle = zeros(1, n);
    sim.yaw_rate = zeros(1, n);
    sim.pitch_rate = zeros(1, n);
    sim.raw_yaw_angle = zeros(1, n);
    sim.raw_pitch_angle = zeros(1, n);
    sim.raw_yaw_rate = zeros(1, n);
    sim.raw_pitch_rate = zeros(1, n);
    sim.yaw_overload_g = zeros(1, n);
    sim.pitch_overload_g = zeros(1, n);
end

function [out, state] = guidance_step(target_x, target_y, dt, roll_rad, gyro_b, state, cfg)
    if target_x < 0.0 || target_y < 0.0
        state = init_guidance_state();
        out = lost_result();
        return;
    end

    if ~cfg.roll_compensation
        roll_rad = 0.0;
    end
    roll_rad = roll_rad * cfg.roll_sign;

    los_c = pixel_to_camera_los(target_x, target_y, cfg);
    los_b = normalize_vec(cfg.R_bc * los_c, cfg.EPS);

    R_roll_comp = roll_compensation_matrix(roll_rad);
    los_s = normalize_vec(R_roll_comp * los_b, cfg.EPS);

    [yaw_angle, pitch_angle] = los_angles(los_s);
    [los_dot_b, state] = los_dot_body(los_b, dt, state);
    los_dot_true_b = los_dot_b + cross(gyro_b(:), los_b);
    los_dot_s = R_roll_comp * los_dot_true_b;
    [yaw_dot, pitch_dot] = los_angle_rates(los_s, los_dot_s, cfg.EPS);

    raw_yaw_angle = yaw_angle;
    raw_pitch_angle = pitch_angle;
    raw_yaw_dot = yaw_dot;
    raw_pitch_dot = pitch_dot;

    [yaw_angle, yaw_dot, pitch_angle, pitch_dot, state] = ...
        filter_los_states(yaw_angle, yaw_dot, pitch_angle, pitch_dot, dt, state, cfg);

    yaw_command_rate = yaw_dot + cfg.position_to_rate_gain * yaw_angle;
    pitch_command_rate = pitch_dot + cfg.position_to_rate_gain * pitch_angle;
    yaw_overload_g = cfg.navigation_ratio * cfg.closing_velocity * yaw_command_rate / cfg.G;
    pitch_overload_g = cfg.navigation_ratio * cfg.closing_velocity * pitch_command_rate / cfg.G;

    out.detected = true;
    out.pixel_error_x = target_x - cfg.cx;
    out.pixel_error_y = target_y - cfg.cy;
    out.yaw_los_angle_rad = yaw_angle;
    out.pitch_los_angle_rad = pitch_angle;
    out.yaw_los_rate_rad_s = yaw_dot;
    out.pitch_los_rate_rad_s = pitch_dot;
    out.raw_yaw_los_angle_rad = raw_yaw_angle;
    out.raw_pitch_los_angle_rad = raw_pitch_angle;
    out.raw_yaw_los_rate_rad_s = raw_yaw_dot;
    out.raw_pitch_los_rate_rad_s = raw_pitch_dot;
    out.yaw_overload_g = limit_overload(yaw_overload_g, cfg.max_overload_g);
    out.pitch_overload_g = limit_overload(pitch_overload_g, cfg.max_overload_g);
end

function out = lost_result()
    out.detected = false;
    out.pixel_error_x = 0.0;
    out.pixel_error_y = 0.0;
    out.yaw_los_angle_rad = 0.0;
    out.pitch_los_angle_rad = 0.0;
    out.yaw_los_rate_rad_s = 0.0;
    out.pitch_los_rate_rad_s = 0.0;
    out.raw_yaw_los_angle_rad = 0.0;
    out.raw_pitch_los_angle_rad = 0.0;
    out.raw_yaw_los_rate_rad_s = 0.0;
    out.raw_pitch_los_rate_rad_s = 0.0;
    out.yaw_overload_g = 0.0;
    out.pitch_overload_g = 0.0;
end

function los_c = pixel_to_camera_los(target_x, target_y, cfg)
    x_n = (target_x - cfg.cx) / cfg.fx;
    y_n = (target_y - cfg.cy) / cfg.fy;
    los_c = normalize_vec([x_n; y_n; 1.0], cfg.EPS);
end

function [los_dot_b, state] = los_dot_body(los_b, dt, state)
    if isempty(state.last_los_b) || isempty(dt) || dt <= 0.0
        state.last_los_b = los_b;
        los_dot_b = zeros(3, 1);
        return;
    end

    los_dot_b = (los_b - state.last_los_b) / dt;
    state.last_los_b = los_b;
end

function R = roll_compensation_matrix(roll_rad)
    cos_roll = cos(roll_rad);
    sin_roll = sin(roll_rad);
    R = [
        1.0, 0.0, 0.0;
        0.0, cos_roll, sin_roll;
        0.0, -sin_roll, cos_roll
    ];
end

function [yaw_angle, pitch_angle] = los_angles(los_s)
    x = los_s(1);
    y = los_s(2);
    z = los_s(3);
    rho = sqrt(x * x + y * y);
    yaw_angle = atan2(y, x);
    pitch_angle = atan2(-z, rho);
end

function [yaw_dot, pitch_dot] = los_angle_rates(los_s, los_dot_s, eps_value)
    x = los_s(1);
    y = los_s(2);
    z = los_s(3);
    xd = los_dot_s(1);
    yd = los_dot_s(2);
    zd = los_dot_s(3);

    rho2 = x * x + y * y;
    if rho2 < eps_value
        yaw_dot = 0.0;
        pitch_dot = 0.0;
        return;
    end

    rho = sqrt(rho2);
    yaw_dot = (x * yd - y * xd) / rho2;
    pitch_dot = -rho * zd + z * (x * xd + y * yd) / rho;
end

function [yaw_angle, yaw_dot, pitch_angle, pitch_dot, state] = ...
    filter_los_states(yaw_angle, yaw_dot, pitch_angle, pitch_dot, dt, state, cfg)

    if cfg.use_kalman_filter
        [yaw_angle, yaw_dot, state.yaw_filter] = ...
            filter_axis_state(state.yaw_filter, yaw_angle, yaw_dot, dt, cfg);
        [pitch_angle, pitch_dot, state.pitch_filter] = ...
            filter_axis_state(state.pitch_filter, pitch_angle, pitch_dot, dt, cfg);
        return;
    end

    [yaw_dot, pitch_dot, state] = filter_los_rates(yaw_dot, pitch_dot, state, cfg);
end

function [angle, rate, axis_filter] = filter_axis_state(axis_filter, angle, rate, dt, cfg)
    if ~axis_filter.initialized
        axis_filter = create_axis_filter(angle, rate, cfg);
        return;
    end

    if isempty(dt) || dt <= 0.0
        dt = 0.0;
    end

    A = [
        1.0, dt;
        0.0, 1.0
    ];
    z = [angle; rate];
    [axis_filter.x, axis_filter.P] = kalman_step(axis_filter.x, axis_filter.P, A, z, cfg);
    angle = axis_filter.x(1);
    rate = axis_filter.x(2);
end

function axis_filter = create_axis_filter(angle, rate, cfg)
    axis_filter.initialized = true;
    axis_filter.x = [angle; rate];
    axis_filter.P = [
        cfg.kalman_angle_variance, 0.0;
        0.0, cfg.kalman_rate_variance
    ];
end

function [x, P] = kalman_step(x, P, A, z, cfg)
    H = eye(2);
    Q = [
        cfg.kalman_process_angle_variance, 0.0;
        0.0, cfg.kalman_process_rate_variance
    ];
    R = [
        cfg.kalman_measurement_angle_variance, 0.0;
        0.0, cfg.kalman_measurement_rate_variance
    ];

    x = A * x;
    P = A * P * A.' + Q;

    residual = z - H * x;
    S = H * P * H.' + R;
    K = P * H.' / S;
    x = x + K * residual;

    I = eye(2);
    I_KH = I - K * H;
    P = I_KH * P * I_KH.' + K * R * K.';
end

function [yaw_dot, pitch_dot, state] = filter_los_rates(yaw_dot, pitch_dot, state, cfg)
    if isnan(cfg.rate_filter_alpha)
        return;
    end

    if ~state.has_filtered_rate
        state.filtered_yaw_dot = yaw_dot;
        state.filtered_pitch_dot = pitch_dot;
        state.has_filtered_rate = true;
        return;
    end

    alpha = clamp(cfg.rate_filter_alpha, 0.0, 1.0);
    state.filtered_yaw_dot = alpha * yaw_dot + (1.0 - alpha) * state.filtered_yaw_dot;
    state.filtered_pitch_dot = alpha * pitch_dot + (1.0 - alpha) * state.filtered_pitch_dot;
    yaw_dot = state.filtered_yaw_dot;
    pitch_dot = state.filtered_pitch_dot;
end

function y = limit_overload(value, max_overload_g)
    if isempty(max_overload_g) || max_overload_g <= 0.0
        y = value;
        return;
    end
    y = clamp(value, -max_overload_g, max_overload_g);
end

function value = focal_length(pixels, fov_deg)
    fov_rad = deg2rad_local(fov_deg);
    value = (pixels * 0.5) / tan(fov_rad * 0.5);
end

function y = normalize_vec(v, eps_value)
    norm_v = sqrt(sum(v .* v));
    if norm_v < eps_value
        y = zeros(size(v));
        return;
    end
    y = v / norm_v;
end

function y = clamp(x, low, high)
    y = min(max(x, low), high);
end

function y = deg2rad_local(x)
    y = x * pi / 180.0;
end

function y = rad2deg_local(x)
    y = x * 180.0 / pi;
end

function path = make_output_path(file_name)
    script_path = mfilename('fullpath');
    script_dir = fileparts(script_path);
    output_dir = fullfile(script_dir, 'output');
    path = fullfile(output_dir, file_name);
end

function plot_simulation_result(t, true_x, true_y, meas_x_plot, meas_y_plot, scene_log, sim, cfg, scene)
    output_path = make_output_path('guidance_simulation_result.png');
    output_dir = fileparts(output_path);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    fig = figure('Name', 'Guidance Algorithm Simulation', 'Color', 'w');
    set(fig, 'Position', [80, 80, 1150, 860]);

    subplot(3, 2, 1);
    plot(true_x, true_y, 'LineWidth', 1.6);
    hold on;
    plot(meas_x_plot, meas_y_plot, '.', 'MarkerSize', 5);
    plot(cfg.cx, cfg.cy, 'kx', 'LineWidth', 1.8, 'MarkerSize', 10);
    set(gca, 'YDir', 'reverse');
    grid on;
    axis([0, cfg.image_width, 0, cfg.image_height]);
    xlabel('pixel x');
    ylabel('pixel y');
    title('Target pixel trajectory');
    legend('true', 'measured', 'center', 'Location', 'best');

    subplot(3, 2, 2);
    plot(t, scene_log.target_lateral_m * 1000.0, 'LineWidth', 1.5);
    hold on;
    plot(t, scene_log.range_m, ':', 'LineWidth', 1.3);
    xline_local(scene.target_move_time_s, 'k:');
    xline_local(scene.apex_time_s, 'r:');
    grid on;
    xlabel('time (s)');
    ylabel('mm / m');
    title(sprintf('Pitch %.1f deg, yaw %.1f deg, forward %.1f m', ...
        scene.launch_pitch_deg, scene.launch_yaw_deg, scene.target_forward_distance_m));
    legend('target lateral (mm)', 'range (m)', '0.6 s', 'apex', 'Location', 'best');

    subplot(3, 2, 3);
    plot(t, rad2deg_local(sim.raw_yaw_angle), ':', 'LineWidth', 1.0);
    hold on;
    plot(t, rad2deg_local(sim.yaw_angle), 'LineWidth', 1.4);
    plot(t, rad2deg_local(sim.raw_pitch_angle), ':', 'LineWidth', 1.0);
    plot(t, rad2deg_local(sim.pitch_angle), 'LineWidth', 1.4);
    grid on;
    xlabel('time (s)');
    ylabel('angle (deg)');
    title('LOS angles');
    legend('yaw raw', 'yaw filtered', 'pitch raw', 'pitch filtered', 'Location', 'best');

    subplot(3, 2, 4);
    plot(t, rad2deg_local(sim.raw_yaw_rate), ':', 'LineWidth', 1.0);
    hold on;
    plot(t, rad2deg_local(sim.yaw_rate), 'LineWidth', 1.4);
    plot(t, rad2deg_local(sim.raw_pitch_rate), ':', 'LineWidth', 1.0);
    plot(t, rad2deg_local(sim.pitch_rate), 'LineWidth', 1.4);
    grid on;
    xlabel('time (s)');
    ylabel('rate (deg/s)');
    title('LOS angle rates');
    legend('yaw raw', 'yaw filtered', 'pitch raw', 'pitch filtered', 'Location', 'best');

    subplot(3, 2, 5);
    plot(t, sim.yaw_overload_g, 'LineWidth', 1.5);
    hold on;
    plot(t, sim.pitch_overload_g, 'LineWidth', 1.5);
    yline_local(cfg.max_overload_g, 'k:');
    yline_local(-cfg.max_overload_g, 'k:');
    grid on;
    xlabel('time (s)');
    ylabel('overload (g)');
    title('Proportional guidance command');
    legend('yaw', 'pitch', 'upper limit', 'lower limit', 'Location', 'best');

    subplot(3, 2, 6);
    plot(t, sim.detected, 'LineWidth', 1.4);
    hold on;
    plot(t, scene_log.camera_enabled, ':', 'LineWidth', 1.4);
    ylim([-0.1, 1.1]);
    grid on;
    xlabel('time (s)');
    ylabel('state');
    title('Camera/guidance availability');
    legend('detected and commanded', 'camera gate', 'Location', 'best');

    saveas(fig, output_path);
end

function plot_flight_hit_result(t, scene_log, scene, hit_result)
    output_path = make_output_path('flight_hit_result.png');
    output_dir = fileparts(output_path);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    yaw = deg2rad_local(scene.launch_yaw_deg);
    range_axis_w = [cos(yaw); sin(yaw); 0.0];
    lateral_axis_w = [-sin(yaw); cos(yaw); 0.0];

    target_center = repmat(hit_result.target_world_m, 1, numel(t));
    projectile_rel = scene_log.projectile_world_m - target_center;
    projectile_range_m = range_axis_w.' * projectile_rel;
    projectile_lateral_m = lateral_axis_w.' * projectile_rel;
    projectile_up_m = -projectile_rel(3, :);

    impact_range_m = dot(hit_result.projectile_world_m - hit_result.target_world_m, range_axis_w);
    impact_lateral_m = dot(hit_result.projectile_world_m - hit_result.target_world_m, lateral_axis_w);
    hit_half_mm = hit_result.half_side_m * 1000.0;

    fig = figure('Name', 'Flight Trajectory and Hit Result', 'Color', 'w');
    set(fig, 'Position', [120, 80, 1180, 820]);

    subplot(2, 2, 1);
    plot(scene_log.projectile_world_m(1, :), scene_log.projectile_world_m(2, :), 'LineWidth', 1.6);
    hold on;
    plot(scene_log.target_world_m(1, :), scene_log.target_world_m(2, :), '--', 'LineWidth', 1.4);
    plot(hit_result.projectile_world_m(1), hit_result.projectile_world_m(2), 'ro', 'LineWidth', 1.8, 'MarkerSize', 8);
    plot(hit_result.target_world_m(1), hit_result.target_world_m(2), 'kx', 'LineWidth', 1.8, 'MarkerSize', 9);
    draw_hit_box_world(hit_result.target_world_m, range_axis_w, lateral_axis_w, hit_result.half_side_m);
    axis equal;
    grid on;
    xlabel('forward x (m)');
    ylabel('right y (m)');
    title('Top view trajectory');
    legend('projectile', 'target', 'impact', 'target center', '140 mm box', 'Location', 'best');

    subplot(2, 2, 2);
    plot(projectile_range_m, projectile_up_m, 'LineWidth', 1.6);
    hold on;
    plot(0.0, 0.0, 'kx', 'LineWidth', 1.8, 'MarkerSize', 9);
    plot(impact_range_m, -hit_result.height_error_m, 'ro', 'LineWidth', 1.8, 'MarkerSize', 8);
    grid on;
    xlabel('range from target center (m)');
    ylabel('height above target plane (m)');
    title('Side view trajectory');
    legend('projectile', 'target center', 'impact', 'Location', 'best');

    subplot(2, 2, 3);
    box_x = hit_half_mm * [-1, 1, 1, -1, -1];
    box_y = hit_half_mm * [-1, -1, 1, 1, -1];
    plot(box_x, box_y, 'k-', 'LineWidth', 1.7);
    hold on;
    plot(0.0, 0.0, 'kx', 'LineWidth', 1.9, 'MarkerSize', 10);
    plot(impact_range_m * 1000.0, impact_lateral_m * 1000.0, 'ro', ...
        'LineWidth', 1.9, 'MarkerSize', 8);
    grid on;
    axis equal;
    margin_mm = max(160.0, max(abs([impact_range_m, impact_lateral_m])) * 1000.0 + 80.0);
    axis([-margin_mm, margin_mm, -margin_mm, margin_mm]);
    xlabel('range error (mm)');
    ylabel('lateral error (mm)');
    title(sprintf('Hit box judgement: %s', hit_result.judgement));
    legend('140 mm box', 'target center', 'impact', 'Location', 'best');

    subplot(2, 2, 4);
    values_mm = [
        hit_result.range_error_m;
        hit_result.lateral_error_m;
        hit_result.center_miss_m;
        hit_result.outside_box_miss_m
    ] * 1000.0;
    bar(values_mm);
    set(gca, 'XTickLabel', {'range', 'lateral', 'center miss', 'outside box'});
    xtickangle_local(20);
    grid on;
    ylabel('mm');
    title(sprintf('%s, center miss %.1f mm', ...
        hit_result.judgement, hit_result.center_miss_m * 1000.0));

    saveas(fig, output_path);
end

function draw_hit_box_world(center_w, range_axis_w, lateral_axis_w, half_side_m)
    corners = [
        -half_side_m, -half_side_m;
        half_side_m, -half_side_m;
        half_side_m, half_side_m;
        -half_side_m, half_side_m;
        -half_side_m, -half_side_m
    ];
    points = repmat(center_w, 1, size(corners, 1)) + ...
        range_axis_w * corners(:, 1).' + lateral_axis_w * corners(:, 2).';
    plot(points(1, :), points(2, :), 'k-', 'LineWidth', 1.6);
end

function save_simulation_data(t, true_x, true_y, meas_x, meas_y, roll_deg, gyro_b, scene_log, sim, cfg, scene, hit_result)
    output_path = make_output_path('guidance_simulation_data.mat');
    output_dir = fileparts(output_path);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    save(output_path, 't', 'true_x', 'true_y', 'meas_x', 'meas_y', ...
        'roll_deg', 'gyro_b', 'scene_log', 'sim', 'cfg', 'scene', 'hit_result');
    fprintf('Output data: %s\n', output_path);
end

function xtickangle_local(angle_deg)
    if exist('xtickangle', 'file') == 2 || exist('xtickangle', 'builtin') == 5
        xtickangle(angle_deg);
    end
end

function xline_local(x, style)
    if exist('xline', 'file') == 2 || exist('xline', 'builtin') == 5
        xline(x, style);
        return;
    end
    limits = ylim;
    plot([x, x], limits, style);
end

function yline_local(y, style)
    if exist('yline', 'file') == 2 || exist('yline', 'builtin') == 5
        yline(y, style);
        return;
    end
    limits = xlim;
    plot(limits, [y, y], style);
end
