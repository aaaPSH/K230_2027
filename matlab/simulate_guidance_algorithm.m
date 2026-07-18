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
% The trajectory is propagated with gravity and closed-loop guidance
% acceleration. The default speed is computed from the same-plane ballistic
% range equation for the nominal static target.
% Guidance commands are updated only on camera frames and held constant until
% the next camera frame. Actual overload follows the held command through a
% first-order response and slew-rate limit, so acceleration changes smoothly.
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

dt = 1.0 / scene.physics_update_rate_hz;
t_end = scene.sim_time_s;
t = 0:dt:t_end;
if t(end) < t_end
    t = [t, t_end];
end
n = numel(t);

noise_sigma_px = scene.measurement_noise_sigma_px;
roll_deg = zeros(size(t));
gyro_b = zeros(3, n);

[true_x, true_y, meas_x, meas_y, scene_log, sim, hit_result] = ...
    run_closed_loop_simulation(t, cfg, scene, noise_sigma_px);

valid = meas_x >= 0.0 & meas_y >= 0.0;
pre_impact_mask = t <= hit_result.impact_time_s;
display_valid = valid & pre_impact_mask;
meas_x_plot = meas_x;
meas_y_plot = meas_y;
meas_x_plot(~display_valid) = NaN;
meas_y_plot(~display_valid) = NaN;

fprintf('导引仿真完成。\n');
fprintf('落地前显示采样点数：%d，积分步长：%.3f 秒，有效相机测量：%d\n', ...
    nnz(pre_impact_mask), dt, nnz(display_valid));
fprintf('动力学更新频率：%.1f Hz，相机帧率：%.1f Hz\n', ...
    scene.physics_update_rate_hz, scene.camera_frame_rate_hz);
fprintf('实际过载响应：时间常数 %.3f 秒，最大变化率 %.1f g/s\n', ...
    scene.overload_response_time_constant_s, scene.overload_slew_rate_g_per_s);
fprintf('场地方位：目标在物体右前方 %.1f 度，发射偏航正对目标 %.1f 度\n', ...
    scene.target_bearing_right_deg, scene.launch_yaw_deg);
fprintf('发射俯仰/偏航：%.1f 度 / 右偏 %.1f 度，速度：%.2f m/s\n', ...
    scene.launch_pitch_deg, scene.launch_yaw_deg, scene.launch_speed_mps);
fprintf('目标前向距离：%.2f m，水平距离：%.2f m，移动时间：%.3f 秒\n', ...
    scene.target_forward_distance_m, scene.target_horizontal_range_m, scene.target_move_time_s);
fprintf('目标名义位置：前向 %.2f m，右向 %.2f m，上向 %.2f m\n', ...
    scene_log.target_nominal_world_m(1), scene_log.target_nominal_world_m(2), ...
    -scene_log.target_nominal_world_m(3));
fprintf('弹道最高点时间：%.3f 秒，静止目标无控命中时间：%.3f 秒\n', ...
    scene.apex_time_s, scene.nominal_static_hit_time_s);
fprintf('同平面无控射程：%.2f m，射程误差：%.3f mm\n', ...
    scene.same_plane_ballistic_range_m, ...
    (scene.same_plane_ballistic_range_m - scene.target_horizontal_range_m) * 1000.0);
fprintf('目标横向移动：%.0f mm -> %.0f mm\n', ...
    scene.target_initial_lateral_m * 1000.0, scene.target_final_lateral_m * 1000.0);
fprintf('弹目角控制增益：偏航 %.3f 1/s，俯仰 %.3f 1/s\n', ...
    cfg.yaw_angle_control_gain + cfg.position_to_rate_gain, ...
    cfg.pitch_angle_control_gain + cfg.position_to_rate_gain);
fprintf('命中判定框：%.0f mm x %.0f mm\n', ...
    scene.hit_box_side_m * 1000.0, scene.hit_box_side_m * 1000.0);
fprintf('落点判定时间：%.3f 秒，判定结果：%s\n', ...
    hit_result.impact_time_s, hit_result.judgement);
fprintf('脱靶分量：前后 %.1f mm，横向 %.1f mm（右为正），高度 %.1f mm\n', ...
    hit_result.range_error_m * 1000.0, ...
    hit_result.lateral_error_m * 1000.0, ...
    hit_result.height_error_m * 1000.0);
fprintf('中心脱靶量：%.1f mm，命中框外脱靶量：%.1f mm\n', ...
    hit_result.center_miss_m * 1000.0, hit_result.outside_box_miss_m * 1000.0);
fprintf('测量噪声标准差：%.2f 像素\n', noise_sigma_px);
fprintf('偏航指令过载范围：   [%.3f, %.3f] g\n', ...
    min(sim.yaw_overload_g(pre_impact_mask)), max(sim.yaw_overload_g(pre_impact_mask)));
fprintf('俯仰指令过载范围：   [%.3f, %.3f] g\n', ...
    min(sim.pitch_overload_g(pre_impact_mask)), max(sim.pitch_overload_g(pre_impact_mask)));
fprintf('偏航实际过载范围：   [%.3f, %.3f] g\n', ...
    min(sim.actual_yaw_overload_g(pre_impact_mask)), max(sim.actual_yaw_overload_g(pre_impact_mask)));
fprintf('俯仰实际过载范围：   [%.3f, %.3f] g\n', ...
    min(sim.actual_pitch_overload_g(pre_impact_mask)), max(sim.actual_pitch_overload_g(pre_impact_mask)));
fprintf('导引结果图：%s\n', make_output_path('guidance_simulation_result.png'));
fprintf('飞行/命中结果图：%s\n', make_output_path('flight_hit_result.png'));

plot_simulation_result(t, true_x, true_y, meas_x_plot, meas_y_plot, scene_log, sim, cfg, scene, hit_result);
plot_flight_hit_result(t, scene_log, scene, hit_result);
save_simulation_data(t, true_x, true_y, meas_x, meas_y, roll_deg, gyro_b, scene_log, sim, cfg, scene, hit_result);

function cfg = default_guidance_config()
    cfg.G = 9.80665;
    cfg.EPS = 1e-9;

    % 与 src/dart_py/config/camera.py、guidance.py 的实机默认值同步。
    cfg.image_width = 320.0;
    cfg.image_height = 240.0;
    cfg.fov_x_deg = 65.0;
    cfg.fov_y_deg = 40.0;
    % 与运行时一致，内参矩阵是像素到 LOS 的唯一标定入口。
    cfg.camera_matrix = [
        251.149692, 0.0, 160.0;
        0.0, 329.697290, 120.0;
        0.0, 0.0, 1.0
    ];
    cfg.fx = cfg.camera_matrix(1, 1);
    cfg.fy = cfg.camera_matrix(2, 2);
    cfg.cx = cfg.camera_matrix(1, 3);
    cfg.cy = cfg.camera_matrix(2, 3);
    cfg.camera_skew = cfg.camera_matrix(1, 2);

    % Camera x-right/y-down/z-forward -> body x-forward/y-right/z-down.
    cfg.R_bc = [
        0.0, 0.0, 1.0;
        1.0, 0.0, 0.0;
        0.0, 1.0, 0.0
    ];

    cfg.navigation_ratio = 3.0;
    cfg.closing_velocity = 14.0;
    cfg.position_to_rate_gain = 0.0;
    cfg.yaw_angle_control_gain = 0.0;
    cfg.pitch_angle_control_gain = 0.0;
    cfg.rate_filter_alpha = NaN;
    cfg.use_kalman_filter = true;

    cfg.kalman_angle_variance = 0.0001;
    cfg.kalman_rate_variance = 10.0;
    cfg.kalman_process_angle_variance = 0.0001;
    cfg.kalman_process_rate_variance = 0.2;
    cfg.kalman_measurement_noise_px = 1.0;
    cfg.kalman_yaw_measurement_angle_variance = ...
        atan(cfg.kalman_measurement_noise_px / cfg.fx)^2;
    cfg.kalman_pitch_measurement_angle_variance = ...
        atan(cfg.kalman_measurement_noise_px / cfg.fy)^2;
    cfg.max_prediction_time_s = 0.1;

    cfg.max_overload_g = 0.5;
    cfg.roll_compensation = true;
    cfg.roll_sign = -1.0;
end

function scene = default_launch_scene()
    scene.launch_pitch_deg = 35.0;
    scene.target_bearing_right_deg = 7.8;
    scene.target_forward_distance_m = 24.5;
    scene.target_lateral_limit_m = 0.280;
    scene.target_move_time_s = 0.600;
    scene.target_initial_lateral_m = 0.0;
    scene.target_final_lateral_m = (2.0 * rand() - 1.0) * scene.target_lateral_limit_m;
    scene.hit_box_side_m = 0.140;
    scene.measurement_noise_sigma_px = 1.0;

    % Gravity is enabled so that camera detection can be gated by descent.
    scene.use_gravity = true;
    scene.gravity_mps2 = 9.80665;
    scene.camera_only_descending = true;
    scene.camera_aligns_with_velocity = true;
    scene.physics_update_rate_hz = 240.0;
    scene.camera_frame_rate_hz = 60.0;
    scene.command_hold_between_frames = true;
    scene.overload_response_time_constant_s = 0.060;
    scene.overload_slew_rate_g_per_s = 120.0;

    pitch_rad = deg2rad_local(scene.launch_pitch_deg);
    target_bearing_rad = deg2rad_local(scene.target_bearing_right_deg);
    scene.target_right_distance_m = scene.target_forward_distance_m * tan(target_bearing_rad);
    scene.target_horizontal_range_m = hypot( ...
        scene.target_forward_distance_m, ...
        scene.target_right_distance_m);
    scene.launch_yaw_deg = rad2deg_local(atan2( ...
        scene.target_right_distance_m, ...
        scene.target_forward_distance_m));
    scene.launch_speed_mps = required_same_plane_speed( ...
        scene.target_horizontal_range_m, ...
        scene.launch_pitch_deg, ...
        scene.gravity_mps2);
    scene.apex_time_s = scene.launch_speed_mps * sin(pitch_rad) / scene.gravity_mps2;
    scene.same_plane_flight_time_s = 2.0 * scene.apex_time_s;
    scene.nominal_static_hit_time_s = scene.same_plane_flight_time_s;
    scene.same_plane_ballistic_range_m = ...
        scene.launch_speed_mps * cos(pitch_rad) * scene.same_plane_flight_time_s;
    scene.sim_time_s = 1.25 * scene.same_plane_flight_time_s;
end

function speed_mps = required_same_plane_speed(horizontal_range_m, launch_pitch_deg, gravity_mps2)
    pitch_rad = deg2rad_local(launch_pitch_deg);
    range_factor = sin(2.0 * pitch_rad);
    if range_factor <= 0.0
        error('发射俯仰角必须在 0 到 90 度之间，才能求解同平面无控射程。');
    end
    speed_mps = sqrt(horizontal_range_m * gravity_mps2 / range_factor);
end

function [true_x, true_y, meas_x, meas_y, log_data, sim, hit_result] = run_closed_loop_simulation(t, cfg, scene, noise_sigma_px)

    n = numel(t);
    true_x = -ones(1, n);
    true_y = -ones(1, n);
    meas_x = -ones(1, n);
    meas_y = -ones(1, n);
    sim = init_sim_log(n);
    log_data = init_scene_log(n);

    launch_R_wb = body_to_world_rotation(scene.launch_yaw_deg, scene.launch_pitch_deg);
    target_nominal_w = [
        scene.target_forward_distance_m;
        scene.target_right_distance_m;
        0.0
    ];
    log_data.target_nominal_world_m = target_nominal_w;
    log_data.launch_R_wb = launch_R_wb;

    projectile_w = [0.0; 0.0; 0.0];
    velocity_w = launch_R_wb(:, 1) * scene.launch_speed_mps;
    state = init_guidance_state();
    held_out = lost_result();
    actual_yaw_overload_g = 0.0;
    actual_pitch_overload_g = 0.0;
    next_camera_time = 0.0;
    last_camera_time = NaN;
    last_camera_R_wb = [];
    camera_period_s = 1.0 / scene.camera_frame_rate_hz;
    time_eps = 1e-9;

    for k = 1:n
        target_lateral = target_lateral_motion(t(k), scene);
        target_w = target_nominal_w + launch_R_wb(:, 2) * target_lateral;

        if scene.camera_aligns_with_velocity
            R_wb = body_to_world_from_velocity(velocity_w, scene.launch_yaw_deg);
        else
            R_wb = launch_R_wb;
        end
        R_bw = R_wb.';

        rel_w = target_w - projectile_w;
        rel_b = R_bw * rel_w;
        body_pitch_deg = velocity_pitch_deg(velocity_w);
        camera_enabled = ~scene.camera_only_descending || velocity_w(3) > 0.0;

        if rel_b(1) > 0.0
            [true_x(k), true_y(k)] = body_vector_to_pixel(rel_b, cfg);
        end

        is_camera_frame = t(k) + time_eps >= next_camera_time;
        if is_camera_frame
            while next_camera_time <= t(k) + time_eps
                next_camera_time = next_camera_time + camera_period_s;
            end

            valid_true_pixel = true_x(k) >= 0.0 && true_x(k) <= cfg.image_width && ...
                true_y(k) >= 0.0 && true_y(k) <= cfg.image_height;
            if camera_enabled && valid_true_pixel
                noisy_x = true_x(k) + noise_sigma_px * randn();
                noisy_y = true_y(k) + noise_sigma_px * randn();
                if noisy_x >= 0.0 && noisy_x <= cfg.image_width && ...
                        noisy_y >= 0.0 && noisy_y <= cfg.image_height
                    meas_x(k) = noisy_x;
                    meas_y(k) = noisy_y;
                end
            end

            if isnan(last_camera_time)
                frame_dt = 0.0;
            else
                frame_dt = t(k) - last_camera_time;
            end
            last_camera_time = t(k);

            camera_gyro_b = zeros(3, 1);
            if ~isempty(last_camera_R_wb) && frame_dt > 0.0
                R_dot = (R_wb - last_camera_R_wb) / frame_dt;
                omega_skew_b = R_wb.' * R_dot;
                omega_skew_b = 0.5 * (omega_skew_b - omega_skew_b.');
                camera_gyro_b = [
                    omega_skew_b(3, 2);
                    omega_skew_b(1, 3);
                    omega_skew_b(2, 1)
                ];
            end
            last_camera_R_wb = R_wb;

            [held_out, state] = guidance_step( ...
                meas_x(k), ...
                meas_y(k), ...
                frame_dt, ...
                0.0, ...
                camera_gyro_b, ...
                state, ...
                cfg);
        elseif ~scene.command_hold_between_frames
            held_out = lost_result();
        end

        if k == 1
            response_dt = 0.0;
        else
            response_dt = t(k) - t(k - 1);
        end
        [actual_yaw_overload_g, actual_pitch_overload_g] = update_actual_overload( ...
            actual_yaw_overload_g, ...
            actual_pitch_overload_g, ...
            held_out.yaw_overload_g, ...
            held_out.pitch_overload_g, ...
            response_dt, ...
            scene);

        out = held_out;
        sim = store_guidance_sample(sim, k, out, actual_yaw_overload_g, actual_pitch_overload_g);

        control_accel_b = [
            0.0;
            actual_yaw_overload_g * cfg.G;
            -actual_pitch_overload_g * cfg.G
        ];
        if scene.use_gravity
            gravity_accel_w = [0.0; 0.0; scene.gravity_mps2];
        else
            gravity_accel_w = [0.0; 0.0; 0.0];
        end
        control_accel_w = R_wb * control_accel_b;
        accel_w = gravity_accel_w + control_accel_w;

        log_data.target_lateral_m(k) = target_lateral;
        log_data.target_world_m(:, k) = target_w;
        log_data.projectile_world_m(:, k) = projectile_w;
        log_data.velocity_world_mps(:, k) = velocity_w;
        log_data.accel_world_mps2(:, k) = accel_w;
        log_data.control_accel_world_mps2(:, k) = control_accel_w;
        log_data.vertical_velocity_down_mps(k) = velocity_w(3);
        log_data.body_pitch_deg(k) = body_pitch_deg;
        log_data.camera_enabled(k) = camera_enabled;
        log_data.camera_frame(k) = is_camera_frame;
        log_data.rel_b_m(:, k) = rel_b;
        log_data.range_m(k) = sqrt(sum(rel_b .* rel_b));

        if k < n
            step_dt = t(k + 1) - t(k);
            projectile_w = projectile_w + velocity_w * step_dt + 0.5 * accel_w * step_dt * step_dt;
            velocity_w = velocity_w + accel_w * step_dt;
        end
    end

    hit_result = evaluate_hit_result(t, log_data, scene);
end

function log_data = init_scene_log(n)
    log_data.target_lateral_m = zeros(1, n);
    log_data.target_world_m = zeros(3, n);
    log_data.projectile_world_m = zeros(3, n);
    log_data.velocity_world_mps = zeros(3, n);
    log_data.accel_world_mps2 = zeros(3, n);
    log_data.control_accel_world_mps2 = zeros(3, n);
    log_data.vertical_velocity_down_mps = zeros(1, n);
    log_data.body_pitch_deg = zeros(1, n);
    log_data.camera_enabled = false(1, n);
    log_data.camera_frame = false(1, n);
    log_data.rel_b_m = zeros(3, n);
    log_data.range_m = zeros(1, n);
    log_data.target_nominal_world_m = zeros(3, 1);
    log_data.launch_R_wb = eye(3);
end

function [actual_yaw_g, actual_pitch_g] = update_actual_overload( ...
    current_yaw_g, current_pitch_g, command_yaw_g, command_pitch_g, dt, scene)

    if dt <= 0.0
        actual_yaw_g = current_yaw_g;
        actual_pitch_g = current_pitch_g;
        return;
    end

    tau = max(scene.overload_response_time_constant_s, 1e-6);
    alpha = 1.0 - exp(-dt / tau);
    target_yaw_g = current_yaw_g + alpha * (command_yaw_g - current_yaw_g);
    target_pitch_g = current_pitch_g + alpha * (command_pitch_g - current_pitch_g);

    max_delta_g = scene.overload_slew_rate_g_per_s * dt;
    actual_yaw_g = current_yaw_g + clamp(target_yaw_g - current_yaw_g, -max_delta_g, max_delta_g);
    actual_pitch_g = current_pitch_g + clamp(target_pitch_g - current_pitch_g, -max_delta_g, max_delta_g);
end

function sim = store_guidance_sample(sim, k, out, actual_yaw_overload_g, actual_pitch_overload_g)
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
    sim.yaw_rate_control_rad_s(k) = out.yaw_rate_control_rad_s;
    sim.pitch_rate_control_rad_s(k) = out.pitch_rate_control_rad_s;
    sim.yaw_angle_control_rad_s(k) = out.yaw_angle_control_rad_s;
    sim.pitch_angle_control_rad_s(k) = out.pitch_angle_control_rad_s;
    sim.yaw_command_rate_rad_s(k) = out.yaw_command_rate_rad_s;
    sim.pitch_command_rate_rad_s(k) = out.pitch_command_rate_rad_s;
    sim.yaw_overload_g(k) = out.yaw_overload_g;
    sim.pitch_overload_g(k) = out.pitch_overload_g;
    sim.actual_yaw_overload_g(k) = actual_yaw_overload_g;
    sim.actual_pitch_overload_g(k) = actual_pitch_overload_g;
end

function hit_result = evaluate_hit_result(t, scene_log, scene)
    impact_time_s = find_impact_time(t, scene_log, scene);
    projectile_w = interp_vector(t, scene_log.projectile_world_m, impact_time_s);
    target_w = interp_vector(t, scene_log.target_world_m, impact_time_s);
    initial_target_w = scene_log.target_world_m(:, 1);

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
        judgement = '命中';
    else
        judgement = '未命中';
    end

    hit_result.impact_time_s = impact_time_s;
    hit_result.projectile_world_m = projectile_w;
    hit_result.target_world_m = target_w;
    hit_result.initial_target_world_m = initial_target_w;
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

function impact_time_s = find_impact_time(t, scene_log, scene)
    height_error_down_m = scene_log.projectile_world_m(3, :) - scene_log.target_world_m(3, :);
    for k = 2:numel(t)
        descending = scene_log.vertical_velocity_down_mps(k) > 0.0;
        crossed_target_plane = height_error_down_m(k - 1) < 0.0 && height_error_down_m(k) >= 0.0;
        if descending && crossed_target_plane
            dz = height_error_down_m(k) - height_error_down_m(k - 1);
            if abs(dz) < 1e-12
                impact_time_s = t(k);
            else
                ratio = -height_error_down_m(k - 1) / dz;
                impact_time_s = t(k - 1) + ratio * (t(k) - t(k - 1));
            end
            return;
        end
    end

    impact_time_s = t(end);
    if scene_log.projectile_world_m(3, end) < scene_log.target_world_m(3, end)
        warning('仿真结束前弹体未到达目标平面，使用最后一个采样点进行脱靶判定。');
    end
end

function value = interp_vector(t, values, query_t)
    value = interp1(t(:), values.', query_t, 'linear', 'extrap').';
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
    x_n = vector_b(2) / vector_b(1);
    y_n = vector_b(3) / vector_b(1);
    pixel_x = cfg.cx + cfg.fx * x_n + cfg.camera_skew * y_n;
    pixel_y = cfg.cy + cfg.fy * y_n;
end

function state = init_guidance_state()
    state.last_los_b = [];
    state.last_los_s = [];
    state.prediction_age_s = 0.0;
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
    sim.yaw_rate_control_rad_s = zeros(1, n);
    sim.pitch_rate_control_rad_s = zeros(1, n);
    sim.yaw_angle_control_rad_s = zeros(1, n);
    sim.pitch_angle_control_rad_s = zeros(1, n);
    sim.yaw_command_rate_rad_s = zeros(1, n);
    sim.pitch_command_rate_rad_s = zeros(1, n);
    sim.yaw_overload_g = zeros(1, n);
    sim.pitch_overload_g = zeros(1, n);
    sim.actual_yaw_overload_g = zeros(1, n);
    sim.actual_pitch_overload_g = zeros(1, n);
end

function [out, state] = guidance_step(target_x, target_y, dt, roll_rad, gyro_b, state, cfg)
    if target_x < 0.0 || target_y < 0.0
        [out, state] = guidance_predict(dt, roll_rad, gyro_b, state, cfg);
        return;
    end
    state.prediction_age_s = 0.0;
    filter_was_initialized = ...
        state.yaw_filter.initialized && state.pitch_filter.initialized;

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
    [los_dot_s, relative_rate_valid, state] = los_dot_stable(los_s, dt, state);
    [relative_yaw_dot, relative_pitch_dot] = ...
        los_angle_rates(los_s, los_dot_s, cfg.EPS);
    [gyro_yaw_dot, gyro_pitch_dot] = ...
        gyro_rate_correction(los_s, gyro_b, R_roll_comp, cfg.EPS);
    if ~relative_rate_valid && ~filter_was_initialized
        gyro_yaw_dot = 0.0;
        gyro_pitch_dot = 0.0;
    end

    raw_yaw_angle = yaw_angle;
    raw_pitch_angle = pitch_angle;
    raw_yaw_dot = relative_yaw_dot + gyro_yaw_dot;
    raw_pitch_dot = relative_pitch_dot + gyro_pitch_dot;

    [yaw_angle, yaw_dot, pitch_angle, pitch_dot, state] = ...
        filter_los_states( ...
            yaw_angle, relative_yaw_dot, pitch_angle, relative_pitch_dot, ...
            dt, gyro_yaw_dot, gyro_pitch_dot, state, cfg);

    yaw_angle_gain = cfg.position_to_rate_gain + cfg.yaw_angle_control_gain;
    pitch_angle_gain = cfg.position_to_rate_gain + cfg.pitch_angle_control_gain;
    yaw_rate_control = yaw_dot;
    pitch_rate_control = pitch_dot;
    yaw_angle_control = yaw_angle_gain * yaw_angle;
    pitch_angle_control = pitch_angle_gain * pitch_angle;
    yaw_command_rate = yaw_rate_control + yaw_angle_control;
    pitch_command_rate = pitch_rate_control + pitch_angle_control;
    stable_yaw_overload_g = cfg.navigation_ratio * cfg.closing_velocity * yaw_command_rate / cfg.G;
    stable_pitch_overload_g = cfg.navigation_ratio * cfg.closing_velocity * pitch_command_rate / cfg.G;
    % PN 律在滚转稳定系计算；执行机构使用弹体系 y/z 通道，故需逆变换。
    body_overload = R_roll_comp.' * [0.0; stable_yaw_overload_g; stable_pitch_overload_g];
    yaw_overload_g = body_overload(2);
    pitch_overload_g = body_overload(3);

    out.detected = true;
    out.predicted = false;
    out.guidance_valid = true;
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
    out.yaw_rate_control_rad_s = yaw_rate_control;
    out.pitch_rate_control_rad_s = pitch_rate_control;
    out.yaw_angle_control_rad_s = yaw_angle_control;
    out.pitch_angle_control_rad_s = pitch_angle_control;
    out.yaw_command_rate_rad_s = yaw_command_rate;
    out.pitch_command_rate_rad_s = pitch_command_rate;
    out.yaw_overload_g = limit_overload(yaw_overload_g, cfg.max_overload_g);
    out.pitch_overload_g = limit_overload(pitch_overload_g, cfg.max_overload_g);
end

function [out, state] = guidance_predict(dt, roll_rad, gyro_b, state, cfg)
    state.last_los_b = [];
    state.last_los_s = [];
    if ~state.yaw_filter.initialized || ~state.pitch_filter.initialized
        out = lost_result();
        return;
    end
    if isempty(dt) || dt <= 0.0
        dt = 0.0;
    end
    state.prediction_age_s = state.prediction_age_s + dt;
    if cfg.max_prediction_time_s <= 0.0 || ...
            state.prediction_age_s > cfg.max_prediction_time_s + cfg.EPS
        state = init_guidance_state();
        out = lost_result();
        return;
    end
    if ~cfg.roll_compensation
        roll_rad = 0.0;
    end
    roll_rad = roll_rad * cfg.roll_sign;
    current_yaw_angle = state.yaw_filter.x(1);
    current_pitch_angle = state.pitch_filter.x(1);
    los_s = los_from_angles(current_yaw_angle, current_pitch_angle);
    R_roll_comp = roll_compensation_matrix(roll_rad);
    [gyro_yaw_dot, gyro_pitch_dot] = ...
        gyro_rate_correction(los_s, gyro_b, R_roll_comp, cfg.EPS);
    state.yaw_filter = ...
        predict_axis_filter(state.yaw_filter, dt, gyro_yaw_dot, cfg);
    state.pitch_filter = ...
        predict_axis_filter(state.pitch_filter, dt, gyro_pitch_dot, cfg);
    yaw_angle = state.yaw_filter.x(1);
    yaw_dot = state.yaw_filter.x(2);
    pitch_angle = state.pitch_filter.x(1);
    pitch_dot = state.pitch_filter.x(2);
    yaw_angle_control = (cfg.position_to_rate_gain + cfg.yaw_angle_control_gain) * yaw_angle;
    pitch_angle_control = (cfg.position_to_rate_gain + cfg.pitch_angle_control_gain) * pitch_angle;
    yaw_command_rate = yaw_dot + yaw_angle_control;
    pitch_command_rate = pitch_dot + pitch_angle_control;
    stable_yaw_g = cfg.navigation_ratio * cfg.closing_velocity * yaw_command_rate / cfg.G;
    stable_pitch_g = cfg.navigation_ratio * cfg.closing_velocity * pitch_command_rate / cfg.G;
    body_overload = R_roll_comp.' * [0.0; stable_yaw_g; stable_pitch_g];
    out = lost_result();
    out.predicted = true;
    out.guidance_valid = true;
    out.yaw_los_angle_rad = yaw_angle;
    out.pitch_los_angle_rad = pitch_angle;
    out.yaw_los_rate_rad_s = yaw_dot;
    out.pitch_los_rate_rad_s = pitch_dot;
    out.yaw_rate_control_rad_s = yaw_dot;
    out.pitch_rate_control_rad_s = pitch_dot;
    out.yaw_angle_control_rad_s = yaw_angle_control;
    out.pitch_angle_control_rad_s = pitch_angle_control;
    out.yaw_command_rate_rad_s = yaw_command_rate;
    out.pitch_command_rate_rad_s = pitch_command_rate;
    out.yaw_overload_g = limit_overload(body_overload(2), cfg.max_overload_g);
    out.pitch_overload_g = limit_overload(body_overload(3), cfg.max_overload_g);
end

function out = lost_result()
    out.detected = false;
    out.predicted = false;
    out.guidance_valid = false;
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
    out.yaw_rate_control_rad_s = 0.0;
    out.pitch_rate_control_rad_s = 0.0;
    out.yaw_angle_control_rad_s = 0.0;
    out.pitch_angle_control_rad_s = 0.0;
    out.yaw_command_rate_rad_s = 0.0;
    out.pitch_command_rate_rad_s = 0.0;
    out.yaw_overload_g = 0.0;
    out.pitch_overload_g = 0.0;
end

function los_c = pixel_to_camera_los(target_x, target_y, cfg)
    y_n = (target_y - cfg.cy) / cfg.fy;
    x_n = (target_x - cfg.cx - cfg.camera_skew * y_n) / cfg.fx;
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

function [los_dot_s, valid, state] = los_dot_stable(los_s, dt, state)
    if isempty(state.last_los_s) || isempty(dt) || dt <= 0.0
        state.last_los_s = los_s;
        los_dot_s = zeros(3, 1);
        valid = false;
        return;
    end
    los_dot_s = (los_s - state.last_los_s) / dt;
    state.last_los_s = los_s;
    valid = true;
end

function [yaw_dot, pitch_dot] = gyro_rate_correction(los_s, gyro_b, R_roll_comp, eps_value)
    gyro_s = R_roll_comp * gyro_b(:);
    % 滚转稳定系已移除绕前向轴的转动，只补偿俯仰和偏航分量。
    gyro_s(1) = 0.0;
    los_dot_correction_s = cross(gyro_s, los_s);
    [yaw_dot, pitch_dot] = ...
        los_angle_rates(los_s, los_dot_correction_s, eps_value);
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

function los_s = los_from_angles(yaw_angle, pitch_angle)
    cos_pitch = cos(pitch_angle);
    los_s = [
        cos_pitch * cos(yaw_angle);
        cos_pitch * sin(yaw_angle);
        -sin(pitch_angle)
    ];
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
    filter_los_states( ...
        yaw_angle, yaw_dot, pitch_angle, pitch_dot, dt, ...
        gyro_yaw_dot, gyro_pitch_dot, state, cfg)

    if cfg.use_kalman_filter
        [yaw_angle, yaw_dot, state.yaw_filter] = ...
            filter_axis_state( ...
                state.yaw_filter, yaw_angle, yaw_dot, dt, ...
                gyro_yaw_dot, cfg.kalman_yaw_measurement_angle_variance, cfg);
        [pitch_angle, pitch_dot, state.pitch_filter] = ...
            filter_axis_state( ...
                state.pitch_filter, pitch_angle, pitch_dot, dt, ...
                gyro_pitch_dot, cfg.kalman_pitch_measurement_angle_variance, cfg);
        return;
    end

    [yaw_dot, pitch_dot, state] = filter_los_rates(yaw_dot, pitch_dot, state, cfg);
    yaw_dot = yaw_dot + gyro_yaw_dot;
    pitch_dot = pitch_dot + gyro_pitch_dot;
end

function [angle, rate, axis_filter] = ...
    filter_axis_state( ...
        axis_filter, angle, rate, dt, gyro_rate_correction, measurement_variance, cfg)
    if ~axis_filter.initialized
        % 与 K230 运行时一致：仅视觉 LOS 角是量测，初始角速度置零。
        axis_filter = create_axis_filter(angle, cfg);
        rate = 0.0;
        return;
    end

    if isempty(dt) || dt <= 0.0
        dt = 0.0;
    end

    A = [
        1.0, dt;
        0.0, 1.0
    ];
    [axis_filter.x, axis_filter.P, reset_filter] = ...
        kalman_angle_step( ...
            axis_filter.x, axis_filter.P, A, angle, dt, ...
            gyro_rate_correction, measurement_variance, cfg);
    if reset_filter
        axis_filter = create_axis_filter(angle, cfg);
    end
    angle = axis_filter.x(1);
    rate = axis_filter.x(2);
end

function axis_filter = create_axis_filter(angle, cfg)
    axis_filter.initialized = true;
    axis_filter.x = [angle; 0.0];
    axis_filter.P = [
        cfg.kalman_angle_variance, 0.0;
        0.0, cfg.kalman_rate_variance
    ];
end

function [x, P, reset_filter] = ...
    kalman_angle_step( ...
        x, P, A, angle, dt, gyro_rate_correction, measurement_variance, cfg)
    H = [1.0, 0.0];
    q = cfg.kalman_process_rate_variance;
    Q = [
        q * dt^3 / 3.0, q * dt^2 / 2.0;
        q * dt^2 / 2.0, q * dt
    ];
    R = measurement_variance;
    reset_filter = false;

    x = A * x;
    % 状态速度定义为惯性 LOS rate；相对角预测需扣除稳定系自身角位移。
    x(1) = x(1) - gyro_rate_correction * dt;
    P = A * P * A.' + Q;

    residual = angle - H * x;
    S = H * P * H.' + R;
    if S <= cfg.EPS || residual * residual > 9.0 * S
        reset_filter = true;
        return;
    end
    K = P * H.' / S;
    x = x + K * residual;

    I = eye(2);
    I_KH = I - K * H;
    P = I_KH * P * I_KH.' + K * R * K.';
end

function axis_filter = predict_axis_filter(axis_filter, dt, gyro_rate_correction, cfg)
    A = [
        1.0, dt;
        0.0, 1.0
    ];
    q = cfg.kalman_process_rate_variance;
    Q = [
        q * dt^3 / 3.0, q * dt^2 / 2.0;
        q * dt^2 / 2.0, q * dt
    ];
    axis_filter.x = A * axis_filter.x;
    axis_filter.x(1) = axis_filter.x(1) - gyro_rate_correction * dt;
    axis_filter.P = A * axis_filter.P * A.' + Q;
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

function plot_simulation_result(t, true_x, true_y, meas_x_plot, meas_y_plot, scene_log, sim, cfg, scene, hit_result)
    output_path = make_output_path('guidance_simulation_result.png');
    output_dir = fileparts(output_path);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    display_mask = t <= hit_result.impact_time_s;
    plot_t = t(display_mask);

    pixel_observation_mask = display_mask & scene_log.camera_enabled & ...
        true_x >= 0.0 & true_x <= cfg.image_width & ...
        true_y >= 0.0 & true_y <= cfg.image_height;
    true_x_plot = true_x;
    true_y_plot = true_y;
    true_x_plot(~pixel_observation_mask) = NaN;
    true_y_plot(~pixel_observation_mask) = NaN;
    meas_x_plot(~display_mask | ~scene_log.camera_enabled) = NaN;
    meas_y_plot(~display_mask | ~scene_log.camera_enabled) = NaN;

    fig = figure('Name', '导引算法仿真', 'Color', 'w');
    set(fig, 'Position', [80, 80, 1150, 860]);

    subplot(3, 2, 1);
    plot(true_x_plot, true_y_plot, 'LineWidth', 1.6);
    hold on;
    plot(meas_x_plot, meas_y_plot, '.', 'MarkerSize', 5);
    plot(cfg.cx, cfg.cy, 'kx', 'LineWidth', 1.8, 'MarkerSize', 10);
    set(gca, 'YDir', 'reverse');
    grid on;
    axis([0, cfg.image_width, 0, cfg.image_height]);
    xlabel('像素 x');
    ylabel('像素 y');
    title('相机观测后的目标像素轨迹');
    legend('真实值', '测量值', '图像中心', 'Location', 'best');

    subplot(3, 2, 2);
    plot(plot_t, scene_log.target_lateral_m(display_mask) * 1000.0, 'LineWidth', 1.5);
    hold on;
    plot(plot_t, scene_log.range_m(display_mask), ':', 'LineWidth', 1.3);
    if scene.target_move_time_s <= hit_result.impact_time_s
        xline_local(scene.target_move_time_s, 'k:');
    end
    if scene.apex_time_s <= hit_result.impact_time_s
        xline_local(scene.apex_time_s, 'r:');
    end
    grid on;
    xlabel('时间 (秒)');
    ylabel('目标横向 (mm) / 弹目距离 (m)');
    title(sprintf('目标右前方 %.1f 度，发射偏航正对目标，前向距离 %.1f m', ...
        scene.target_bearing_right_deg, scene.target_forward_distance_m));
    legend('目标横向位置 (mm)', '弹目距离 (m)', '0.6 秒', '弹道最高点', 'Location', 'best');

    subplot(3, 2, 3);
    plot(plot_t, rad2deg_local(sim.raw_yaw_angle(display_mask)), ':', 'LineWidth', 1.0);
    hold on;
    plot(plot_t, rad2deg_local(sim.yaw_angle(display_mask)), 'LineWidth', 1.4);
    plot(plot_t, rad2deg_local(sim.raw_pitch_angle(display_mask)), ':', 'LineWidth', 1.0);
    plot(plot_t, rad2deg_local(sim.pitch_angle(display_mask)), 'LineWidth', 1.4);
    grid on;
    xlabel('时间 (秒)');
    ylabel('角度 (度)');
    title('弹目视线角');
    legend('偏航原始值', '偏航滤波值', '俯仰原始值', '俯仰滤波值', 'Location', 'best');

    subplot(3, 2, 4);
    plot(plot_t, rad2deg_local(sim.raw_yaw_rate(display_mask)), ':', 'LineWidth', 1.0);
    hold on;
    plot(plot_t, rad2deg_local(sim.yaw_rate(display_mask)), 'LineWidth', 1.4);
    plot(plot_t, rad2deg_local(sim.raw_pitch_rate(display_mask)), ':', 'LineWidth', 1.0);
    plot(plot_t, rad2deg_local(sim.pitch_rate(display_mask)), 'LineWidth', 1.4);
    grid on;
    xlabel('时间 (秒)');
    ylabel('角速度 (度/s)');
    title('弹目视线角速度');
    legend('偏航原始值', '偏航滤波值', '俯仰原始值', '俯仰滤波值', 'Location', 'best');

    subplot(3, 2, 5);
    yaw_angle_overload_g = cfg.navigation_ratio * cfg.closing_velocity * ...
        sim.yaw_angle_control_rad_s / cfg.G;
    pitch_angle_overload_g = cfg.navigation_ratio * cfg.closing_velocity * ...
        sim.pitch_angle_control_rad_s / cfg.G;
    plot(plot_t, sim.actual_yaw_overload_g(display_mask), 'LineWidth', 1.6);
    hold on;
    plot(plot_t, sim.actual_pitch_overload_g(display_mask), 'LineWidth', 1.6);
    stairs(plot_t, sim.yaw_overload_g(display_mask), ':', 'LineWidth', 1.2);
    stairs(plot_t, sim.pitch_overload_g(display_mask), ':', 'LineWidth', 1.2);
    stairs(plot_t, yaw_angle_overload_g(display_mask), '--', 'LineWidth', 1.1);
    stairs(plot_t, pitch_angle_overload_g(display_mask), '--', 'LineWidth', 1.1);
    yline_local(cfg.max_overload_g, 'k:');
    yline_local(-cfg.max_overload_g, 'k:');
    grid on;
    xlabel('时间 (秒)');
    ylabel('过载 (g)');
    title('导引指令与弹目角控制项');
    legend( ...
        '偏航实际过载', ...
        '俯仰实际过载', ...
        '偏航指令过载', ...
        '俯仰指令过载', ...
        '偏航弹目角项', ...
        '俯仰弹目角项', ...
        '上限', ...
        '下限', ...
        'Location', 'best');

    subplot(3, 2, 6);
    plot(plot_t, sim.detected(display_mask), 'LineWidth', 1.4);
    hold on;
    plot(plot_t, scene_log.camera_enabled(display_mask), ':', 'LineWidth', 1.4);
    ylim([-0.1, 1.1]);
    grid on;
    xlabel('时间 (秒)');
    ylabel('状态');
    title('相机与导引可用性');
    legend('已识别并输出指令', '相机门控', 'Location', 'best');

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

    plot_mask = t <= hit_result.impact_time_s;
    if ~any(plot_mask)
        plot_mask = true(size(t));
    end
    target_center = repmat(hit_result.target_world_m, 1, nnz(plot_mask));
    projectile_rel = scene_log.projectile_world_m(:, plot_mask) - target_center;
    projectile_range_m = range_axis_w.' * projectile_rel;
    projectile_lateral_m = lateral_axis_w.' * projectile_rel;
    projectile_up_m = -projectile_rel(3, :);

    impact_range_m = dot(hit_result.projectile_world_m - hit_result.target_world_m, range_axis_w);
    impact_lateral_m = dot(hit_result.projectile_world_m - hit_result.target_world_m, lateral_axis_w);
    initial_target_rel_m = hit_result.initial_target_world_m - hit_result.target_world_m;
    initial_target_range_m = dot(initial_target_rel_m, range_axis_w);
    initial_target_right_m = dot(initial_target_rel_m, lateral_axis_w);
    hit_half_mm = hit_result.half_side_m * 1000.0;

    fig = figure('Name', '飞行轨迹与命中判定', 'Color', 'w');
    set(fig, 'Position', [120, 80, 1180, 820]);

    subplot(2, 2, 1);
    plot3( ...
        scene_log.projectile_world_m(1, plot_mask), ...
        scene_log.projectile_world_m(2, plot_mask), ...
        -scene_log.projectile_world_m(3, plot_mask), ...
        'LineWidth', 1.7);
    hold on;
    plot3( ...
        scene_log.target_world_m(1, plot_mask), ...
        scene_log.target_world_m(2, plot_mask), ...
        -scene_log.target_world_m(3, plot_mask), ...
        '--', 'LineWidth', 1.4);
    plot3( ...
        hit_result.projectile_world_m(1), ...
        hit_result.projectile_world_m(2), ...
        -hit_result.projectile_world_m(3), ...
        'ro', 'LineWidth', 1.8, 'MarkerSize', 8);
    plot3( ...
        hit_result.target_world_m(1), ...
        hit_result.target_world_m(2), ...
        -hit_result.target_world_m(3), ...
        'kx', 'LineWidth', 1.8, 'MarkerSize', 9);
    plot3( ...
        hit_result.initial_target_world_m(1), ...
        hit_result.initial_target_world_m(2), ...
        -hit_result.initial_target_world_m(3), ...
        'bd', 'LineWidth', 1.6, 'MarkerSize', 7);
    draw_hit_box_world_3d(hit_result.target_world_m, range_axis_w, lateral_axis_w, hit_result.half_side_m, 'k-');
    draw_hit_box_world_3d(hit_result.initial_target_world_m, range_axis_w, lateral_axis_w, hit_result.half_side_m, 'k--');
    axis equal;
    grid on;
    xlabel('世界 x / 场地前向 (m)');
    ylabel('世界 y / 场地右向 (m，右为正)');
    zlabel('上向 z (m)');
    title('三维飞行轨迹（世界坐标系）');
    view(38, 24);
    legend( ...
        '弹体', ...
        '目标', ...
        '落点', ...
        '最终靶心', ...
        '移动前靶心', ...
        '最终 140 mm 命中框', ...
        '移动前 140 mm 命中框', ...
        'Location', 'best');

    subplot(2, 2, 2);
    plot(projectile_range_m, projectile_up_m, 'LineWidth', 1.6);
    hold on;
    plot(0.0, 0.0, 'kx', 'LineWidth', 1.8, 'MarkerSize', 9);
    plot(impact_range_m, -hit_result.height_error_m, 'ro', 'LineWidth', 1.8, 'MarkerSize', 8);
    grid on;
    xlabel('相对靶心前后距离 (m)');
    ylabel('高于目标平面高度 (m)');
    title('侧视弹道');
    legend('弹体', '靶心', '落点', 'Location', 'best');

    subplot(2, 2, 3);
    box_x = hit_half_mm * [-1, 1, 1, -1, -1];
    box_y = hit_half_mm * [-1, -1, 1, 1, -1];
    initial_box_x = initial_target_right_m * 1000.0 + box_x;
    initial_box_y = initial_target_range_m * 1000.0 + box_y;
    plot(box_x, box_y, 'k-', 'LineWidth', 1.7);
    hold on;
    plot(initial_box_x, initial_box_y, 'k--', 'LineWidth', 1.4);
    plot(0.0, 0.0, 'kx', 'LineWidth', 1.9, 'MarkerSize', 10);
    plot(initial_target_right_m * 1000.0, initial_target_range_m * 1000.0, ...
        'bd', 'LineWidth', 1.6, 'MarkerSize', 7);
    plot(impact_lateral_m * 1000.0, impact_range_m * 1000.0, 'ro', ...
        'LineWidth', 1.9, 'MarkerSize', 8);
    grid on;
    axis equal;
    margin_mm = max(160.0, ...
        max(abs([ ...
            impact_range_m, ...
            impact_lateral_m, ...
            initial_target_range_m, ...
            initial_target_right_m])) * 1000.0 + 80.0);
    axis([-margin_mm, margin_mm, -margin_mm, margin_mm]);
    xlabel('左右脱靶 (mm，右为正)');
    ylabel('前后脱靶 (mm，前为正)');
    title(sprintf('命中框判定：%s', hit_result.judgement));
    legend( ...
        '最终 140 mm 命中框', ...
        '移动前 140 mm 命中框', ...
        '最终靶心', ...
        '移动前靶心', ...
        '落点', ...
        'Location', 'best');

    subplot(2, 2, 4);
    values_mm = [
        hit_result.range_error_m;
        hit_result.lateral_error_m;
        hit_result.center_miss_m;
        hit_result.outside_box_miss_m
    ] * 1000.0;
    bar(values_mm);
    set(gca, 'XTickLabel', {'前后', '左右(右+)', '中心脱靶', '框外脱靶'});
    xtickangle_local(20);
    grid on;
    ylabel('mm');
    title(sprintf('%s，中心脱靶 %.1f mm', ...
        hit_result.judgement, hit_result.center_miss_m * 1000.0));

    saveas(fig, output_path);
end

function draw_hit_box_world_3d(center_w, range_axis_w, lateral_axis_w, half_side_m, line_style)
    corners = [
        -half_side_m, -half_side_m;
        half_side_m, -half_side_m;
        half_side_m, half_side_m;
        -half_side_m, half_side_m;
        -half_side_m, -half_side_m
    ];
    points = repmat(center_w, 1, size(corners, 1)) + ...
        range_axis_w * corners(:, 1).' + lateral_axis_w * corners(:, 2).';
    plot3(points(1, :), points(2, :), -points(3, :), line_style, 'LineWidth', 1.6);
end

function save_simulation_data(t, true_x, true_y, meas_x, meas_y, roll_deg, gyro_b, scene_log, sim, cfg, scene, hit_result)
    output_path = make_output_path('guidance_simulation_data.mat');
    output_dir = fileparts(output_path);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    save(output_path, 't', 'true_x', 'true_y', 'meas_x', 'meas_y', ...
        'roll_deg', 'gyro_b', 'scene_log', 'sim', 'cfg', 'scene', 'hit_result');
    fprintf('输出数据：%s\n', output_path);
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
