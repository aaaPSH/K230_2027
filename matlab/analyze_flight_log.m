function [summary, data] = analyze_flight_log(csv_path, output_dir)
%ANALYZE_FLIGHT_LOG 分析 K230 飞镖逐帧 CSV 飞行日志。
%
% 用法：
%   analyze_flight_log('flight_123.csv');
%   [summary, data] = analyze_flight_log('flight_123.csv', 'analysis_output');
%   analyze_flight_log();  % 弹出文件选择窗口
%
% 输出：
%   summary - 关键统计量结构体。
%   data    - readtable() 读取的原始表格，附加 analysis_time_s。
%
% 脚本使用当前精简日志的核心字段，也兼容包含更多字段的旧日志。默认创新
% 门限为 4 sigma，默认过载限幅为 0.5 g，与当前 K230 配置一致。

    if nargin < 1 || isempty(csv_path)
        [file_name, path_name] = uigetfile('*.csv', '选择 K230 飞行日志');
        if isequal(file_name, 0)
            error('未选择飞行日志。');
        end
        csv_path = fullfile(path_name, file_name);
    end
    if ~isfile(csv_path)
        error('飞行日志不存在：%s', csv_path);
    end

    [csv_dir, csv_name, ~] = fileparts(csv_path);
    if nargin < 2 || isempty(output_dir)
        output_dir = fullfile(csv_dir, [csv_name, '_analysis']);
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    cfg.innovation_gate_sigma = 4.0;
    cfg.innovation_nis_threshold = cfg.innovation_gate_sigma^2;
    cfg.max_overload_g = 0.5;

    data = readtable(csv_path);
    if height(data) == 0
        error('飞行日志不包含数据行：%s', csv_path);
    end

    time_s = build_time_axis(data);
    data.analysis_time_s = time_s;

    signals = load_signals(data);
    summary = calculate_summary(time_s, signals, cfg);
    print_summary(csv_path, output_dir, summary, cfg);
    write_summary_file(fullfile(output_dir, 'summary.txt'), csv_path, summary, cfg);

    plot_overview(time_s, signals, output_dir);
    plot_los(time_s, signals, output_dir);
    plot_kalman(time_s, signals, output_dir, cfg);
    plot_control_and_imu(time_s, signals, output_dir, cfg);
end

function signals = load_signals(data)
    signals.frame_index = numeric_column(data, 'frame_index');
    signals.dt_s = numeric_column(data, 'dt_s');
    signals.fps = numeric_column(data, 'fps');
    signals.detected = numeric_column(data, 'detected');
    signals.target_x = numeric_column(data, 'target_x');
    signals.target_y = numeric_column(data, 'target_y');
    signals.target_area = numeric_column(data, 'target_area');
    signals.target_circularity = numeric_column(data, 'target_circularity');

    signals.imu_timestamp_error_us = numeric_column(data, 'imu_timestamp_error_us');
    signals.imu_source_age_us = numeric_column(data, 'imu_source_age_us');
    signals.imu_fault = numeric_column(data, 'imu_fault');
    signals.roll_rad = numeric_column(data, 'roll_rad');
    signals.gyro_x = numeric_column(data, 'gyro_x_rad_s');
    signals.gyro_y = numeric_column(data, 'gyro_y_rad_s');
    signals.gyro_z = numeric_column(data, 'gyro_z_rad_s');
    signals.gyro_held = numeric_column(data, 'gyro_held');

    signals.raw_yaw_angle = numeric_column(data, 'raw_yaw_los_angle_rad');
    signals.raw_pitch_angle = numeric_column(data, 'raw_pitch_los_angle_rad');
    signals.raw_yaw_rate = numeric_column(data, 'raw_yaw_los_rate_rad_s');
    signals.raw_pitch_rate = numeric_column(data, 'raw_pitch_los_rate_rad_s');
    signals.yaw_angle = numeric_column(data, 'yaw_los_angle_rad');
    signals.pitch_angle = numeric_column(data, 'pitch_los_angle_rad');
    signals.yaw_rate = numeric_column(data, 'yaw_los_rate_rad_s');
    signals.pitch_rate = numeric_column(data, 'pitch_los_rate_rad_s');
    signals.gyro_yaw_correction = numeric_column( ...
        data, 'gyro_yaw_los_rate_correction_rad_s');
    signals.gyro_pitch_correction = numeric_column( ...
        data, 'gyro_pitch_los_rate_correction_rad_s');

    signals.filter_reinitialized = numeric_column(data, 'filter_reinitialized');
    signals.guidance_valid = numeric_column(data, 'guidance_valid');
    signals.guidance_predicted = numeric_column(data, 'guidance_predicted');
    signals.prediction_age_s = numeric_column(data, 'prediction_age_s');
    signals.sensor_valid = numeric_column(data, 'sensor_valid');

    signals.yaw_mode = text_column(data, 'yaw_kalman_mode');
    signals.pitch_mode = text_column(data, 'pitch_kalman_mode');
    signals.yaw_rate_initialized = numeric_column( ...
        data, 'yaw_kalman_rate_initialized');
    signals.pitch_rate_initialized = numeric_column( ...
        data, 'pitch_kalman_rate_initialized');
    signals.yaw_predicted_angle = numeric_column( ...
        data, 'yaw_kalman_predicted_angle_rad');
    signals.pitch_predicted_angle = numeric_column( ...
        data, 'pitch_kalman_predicted_angle_rad');
    signals.yaw_predicted_rate = numeric_column( ...
        data, 'yaw_kalman_predicted_rate_rad_s');
    signals.pitch_predicted_rate = numeric_column( ...
        data, 'pitch_kalman_predicted_rate_rad_s');
    signals.yaw_residual = numeric_column( ...
        data, 'yaw_kalman_innovation_residual_rad');
    signals.pitch_residual = numeric_column( ...
        data, 'pitch_kalman_innovation_residual_rad');
    signals.yaw_innovation_variance = numeric_column( ...
        data, 'yaw_kalman_innovation_variance_rad2');
    signals.pitch_innovation_variance = numeric_column( ...
        data, 'pitch_kalman_innovation_variance_rad2');
    signals.yaw_nis = numeric_column(data, 'yaw_kalman_innovation_nis');
    signals.pitch_nis = numeric_column(data, 'pitch_kalman_innovation_nis');
    signals.yaw_p00 = numeric_column(data, 'yaw_kalman_covariance_angle_rad2');
    signals.pitch_p00 = numeric_column(data, 'pitch_kalman_covariance_angle_rad2');
    signals.yaw_p11 = numeric_column(data, 'yaw_kalman_covariance_rate_rad2_s2');
    signals.pitch_p11 = numeric_column(data, 'pitch_kalman_covariance_rate_rad2_s2');

    signals.command_yaw = numeric_column(data, 'command_yaw_overload_g');
    signals.command_pitch = numeric_column(data, 'command_pitch_overload_g');
end

function summary = calculate_summary(time_s, signals, cfg)
    summary.frame_count = numel(time_s);
    summary.duration_s = max(time_s) - min(time_s);
    summary.mean_fps = finite_mean(signals.fps);
    summary.p05_fps = finite_percentile(signals.fps, 5.0);
    summary.detected_percent = flag_percent(signals.detected);
    summary.guidance_valid_percent = flag_percent(signals.guidance_valid);
    summary.predicted_percent = flag_percent(signals.guidance_predicted);
    summary.sensor_valid_percent = flag_percent(signals.sensor_valid);
    summary.gyro_held_percent = flag_percent(signals.gyro_held);
    summary.imu_fault_percent = flag_percent(signals.imu_fault);
    summary.max_imu_source_age_ms = finite_max(signals.imu_source_age_us) / 1000.0;
    summary.max_imu_match_error_ms = finite_max(signals.imu_timestamp_error_us) / 1000.0;
    summary.filter_reinitialization_count = nnz(signals.filter_reinitialized > 0.5);
    summary.yaw_realignment_count = count_mode(signals.yaw_mode, 'realigned');
    summary.pitch_realignment_count = count_mode(signals.pitch_mode, 'realigned');
    summary.yaw_nis_p95 = finite_percentile(signals.yaw_nis, 95.0);
    summary.pitch_nis_p95 = finite_percentile(signals.pitch_nis, 95.0);
    summary.yaw_nis_exceed_percent = threshold_percent( ...
        signals.yaw_nis, cfg.innovation_nis_threshold);
    summary.pitch_nis_exceed_percent = threshold_percent( ...
        signals.pitch_nis, cfg.innovation_nis_threshold);
    summary.yaw_rate_raw_filtered_rmse = finite_rmse( ...
        signals.raw_yaw_rate, signals.yaw_rate);
    summary.pitch_rate_raw_filtered_rmse = finite_rmse( ...
        signals.raw_pitch_rate, signals.pitch_rate);
    summary.max_abs_command_yaw_g = finite_max(abs(signals.command_yaw));
    summary.max_abs_command_pitch_g = finite_max(abs(signals.command_pitch));
    summary.yaw_saturation_percent = saturation_percent( ...
        signals.command_yaw, cfg.max_overload_g);
    summary.pitch_saturation_percent = saturation_percent( ...
        signals.command_pitch, cfg.max_overload_g);
end

function print_summary(csv_path, output_dir, summary, cfg)
    fprintf('\nK230 飞行日志分析完成。\n');
    fprintf('日志：%s\n', csv_path);
    fprintf('输出目录：%s\n', output_dir);
    fprintf('帧数：%d，持续时间：%.3f s，平均 FPS：%.2f，P05 FPS：%.2f\n', ...
        summary.frame_count, summary.duration_s, summary.mean_fps, summary.p05_fps);
    fprintf('检测率：%.1f%%，制导有效率：%.1f%%，预测帧占比：%.1f%%\n', ...
        summary.detected_percent, summary.guidance_valid_percent, ...
        summary.predicted_percent);
    fprintf('IMU 有效率：%.1f%%，保持旧陀螺占比：%.1f%%，故障占比：%.1f%%\n', ...
        summary.sensor_valid_percent, summary.gyro_held_percent, ...
        summary.imu_fault_percent);
    fprintf('IMU 最大数据年龄：%.2f ms，最大图像匹配误差：%.2f ms\n', ...
        summary.max_imu_source_age_ms, summary.max_imu_match_error_ms);
    fprintf('滤波重新对齐：总计 %d，偏航 %d，俯仰 %d\n', ...
        summary.filter_reinitialization_count, summary.yaw_realignment_count, ...
        summary.pitch_realignment_count);
    fprintf('NIS 门限：%.1f；偏航 P95 %.2f/超限 %.1f%%，俯仰 P95 %.2f/超限 %.1f%%\n', ...
        cfg.innovation_nis_threshold, summary.yaw_nis_p95, ...
        summary.yaw_nis_exceed_percent, summary.pitch_nis_p95, ...
        summary.pitch_nis_exceed_percent);
    fprintf('最大指令：偏航 %.3f g，俯仰 %.3f g；饱和占比：%.1f%% / %.1f%%\n\n', ...
        summary.max_abs_command_yaw_g, summary.max_abs_command_pitch_g, ...
        summary.yaw_saturation_percent, summary.pitch_saturation_percent);
end

function write_summary_file(path, csv_path, summary, cfg)
    file_id = fopen(path, 'w');
    if file_id < 0
        warning('无法写入分析摘要：%s', path);
        return;
    end
    cleaner = onCleanup(@() fclose(file_id));
    fprintf(file_id, 'K230 飞行日志分析摘要\n');
    fprintf(file_id, '日志=%s\n', csv_path);
    fields = fieldnames(summary);
    for index = 1:numel(fields)
        value = summary.(fields{index});
        fprintf(file_id, '%s=%.9g\n', fields{index}, value);
    end
    fprintf(file_id, 'innovation_nis_threshold=%.9g\n', ...
        cfg.innovation_nis_threshold);
    clear cleaner;
end

function plot_overview(time_s, signals, output_dir)
    fig = figure('Color', 'w', 'Name', '飞行日志总览');

    subplot(3, 2, 1);
    stairs(time_s, signals.detected, 'LineWidth', 1.1); hold on;
    stairs(time_s, signals.guidance_valid, 'LineWidth', 1.1);
    stairs(time_s, signals.guidance_predicted, 'LineWidth', 1.1);
    stairs(time_s, signals.sensor_valid, 'LineWidth', 1.1);
    grid on; ylim([-0.1, 1.2]);
    xlabel('时间 (s)'); ylabel('状态'); title('检测与有效性');
    legend('检测', '制导有效', '预测', 'IMU 有效', 'Location', 'best');

    subplot(3, 2, 2);
    plot(time_s, signals.target_x, 'LineWidth', 1.1); hold on;
    plot(time_s, signals.target_y, 'LineWidth', 1.1);
    grid on; xlabel('时间 (s)'); ylabel('像素'); title('目标质心');
    legend('x', 'y', 'Location', 'best');

    subplot(3, 2, 3);
    plot(time_s, signals.target_area, 'LineWidth', 1.1);
    grid on; xlabel('时间 (s)'); ylabel('px^2'); title('目标面积');

    subplot(3, 2, 4);
    plot(time_s, signals.target_circularity, 'LineWidth', 1.1);
    grid on; ylim([0, 1.1]); xlabel('时间 (s)'); ylabel('圆形度');
    title('检测轮廓质量');

    subplot(3, 2, 5);
    plot(time_s, signals.fps, 'LineWidth', 1.1); hold on;
    plot(time_s, 1.0 ./ signals.dt_s, 'LineWidth', 0.9);
    grid on; xlabel('时间 (s)'); ylabel('Hz'); title('视觉循环频率');
    legend('EMA FPS', '1/dt', 'Location', 'best');

    subplot(3, 2, 6);
    plot(time_s, signals.imu_source_age_us / 1000.0, 'LineWidth', 1.1); hold on;
    plot(time_s, signals.imu_timestamp_error_us / 1000.0, 'LineWidth', 1.1);
    grid on; xlabel('时间 (s)'); ylabel('ms'); title('IMU 时间质量');
    legend('数据年龄', '图像匹配误差', 'Location', 'best');

    save_figure(fig, fullfile(output_dir, '01_overview.png'));
end

function plot_los(time_s, signals, output_dir)
    rad_to_deg = 180.0 / pi;
    fig = figure('Color', 'w', 'Name', 'LOS 角度与角速度');

    subplot(2, 2, 1);
    plot(time_s, signals.raw_yaw_angle * rad_to_deg, 'LineWidth', 0.9); hold on;
    plot(time_s, signals.yaw_predicted_angle * rad_to_deg, '--', 'LineWidth', 1.0);
    plot(time_s, signals.yaw_angle * rad_to_deg, 'LineWidth', 1.3);
    grid on; xlabel('时间 (s)'); ylabel('deg'); title('偏航 LOS 角');
    legend('视觉原始', 'Kalman 预测', '滤波后', 'Location', 'best');

    subplot(2, 2, 2);
    plot(time_s, signals.raw_pitch_angle * rad_to_deg, 'LineWidth', 0.9); hold on;
    plot(time_s, signals.pitch_predicted_angle * rad_to_deg, '--', 'LineWidth', 1.0);
    plot(time_s, signals.pitch_angle * rad_to_deg, 'LineWidth', 1.3);
    grid on; xlabel('时间 (s)'); ylabel('deg'); title('俯仰 LOS 角');
    legend('视觉原始', 'Kalman 预测', '滤波后', 'Location', 'best');

    subplot(2, 2, 3);
    plot(time_s, signals.raw_yaw_rate, 'LineWidth', 0.8); hold on;
    plot(time_s, signals.yaw_predicted_rate, '--', 'LineWidth', 1.0);
    plot(time_s, signals.yaw_rate, 'LineWidth', 1.3);
    plot(time_s, signals.gyro_yaw_correction, ':', 'LineWidth', 1.0);
    grid on; xlabel('时间 (s)'); ylabel('rad/s'); title('偏航 LOS rate');
    legend('原始', '预测', '滤波后', '陀螺补偿', 'Location', 'best');

    subplot(2, 2, 4);
    plot(time_s, signals.raw_pitch_rate, 'LineWidth', 0.8); hold on;
    plot(time_s, signals.pitch_predicted_rate, '--', 'LineWidth', 1.0);
    plot(time_s, signals.pitch_rate, 'LineWidth', 1.3);
    plot(time_s, signals.gyro_pitch_correction, ':', 'LineWidth', 1.0);
    grid on; xlabel('时间 (s)'); ylabel('rad/s'); title('俯仰 LOS rate');
    legend('原始', '预测', '滤波后', '陀螺补偿', 'Location', 'best');

    save_figure(fig, fullfile(output_dir, '02_los.png'));
end

function plot_kalman(time_s, signals, output_dir, cfg)
    rad_to_deg = 180.0 / pi;
    fig = figure('Color', 'w', 'Name', 'Kalman 创新与协方差');

    subplot(3, 2, 1);
    yaw_sigma = sqrt(max(signals.yaw_innovation_variance, 0.0));
    plot(time_s, signals.yaw_residual * rad_to_deg, 'LineWidth', 1.0); hold on;
    plot(time_s, cfg.innovation_gate_sigma * yaw_sigma * rad_to_deg, 'r--');
    plot(time_s, -cfg.innovation_gate_sigma * yaw_sigma * rad_to_deg, 'r--');
    grid on; xlabel('时间 (s)'); ylabel('deg'); title('偏航创新与门限');
    legend('残差', '+门限', '-门限', 'Location', 'best');

    subplot(3, 2, 2);
    pitch_sigma = sqrt(max(signals.pitch_innovation_variance, 0.0));
    plot(time_s, signals.pitch_residual * rad_to_deg, 'LineWidth', 1.0); hold on;
    plot(time_s, cfg.innovation_gate_sigma * pitch_sigma * rad_to_deg, 'r--');
    plot(time_s, -cfg.innovation_gate_sigma * pitch_sigma * rad_to_deg, 'r--');
    grid on; xlabel('时间 (s)'); ylabel('deg'); title('俯仰创新与门限');
    legend('残差', '+门限', '-门限', 'Location', 'best');

    subplot(3, 2, 3);
    plot(time_s, signals.yaw_nis, 'LineWidth', 1.0); hold on;
    yline_local(cfg.innovation_nis_threshold, 'r--');
    grid on; xlabel('时间 (s)'); ylabel('NIS'); title('偏航归一化创新');

    subplot(3, 2, 4);
    plot(time_s, signals.pitch_nis, 'LineWidth', 1.0); hold on;
    yline_local(cfg.innovation_nis_threshold, 'r--');
    grid on; xlabel('时间 (s)'); ylabel('NIS'); title('俯仰归一化创新');

    subplot(3, 2, 5);
    semilogy(time_s, sqrt(max(signals.yaw_p00, 0.0)) * rad_to_deg, ...
        'LineWidth', 1.1); hold on;
    semilogy(time_s, sqrt(max(signals.yaw_p11, 0.0)) * rad_to_deg, ...
        'LineWidth', 1.1);
    grid on; xlabel('时间 (s)'); ylabel('标准差'); title('偏航后验不确定度');
    legend('角度 deg', '角速度 deg/s', 'Location', 'best');

    subplot(3, 2, 6);
    semilogy(time_s, sqrt(max(signals.pitch_p00, 0.0)) * rad_to_deg, ...
        'LineWidth', 1.1); hold on;
    semilogy(time_s, sqrt(max(signals.pitch_p11, 0.0)) * rad_to_deg, ...
        'LineWidth', 1.1);
    grid on; xlabel('时间 (s)'); ylabel('标准差'); title('俯仰后验不确定度');
    legend('角度 deg', '角速度 deg/s', 'Location', 'best');

    save_figure(fig, fullfile(output_dir, '03_kalman.png'));
end

function plot_control_and_imu(time_s, signals, output_dir, cfg)
    fig = figure('Color', 'w', 'Name', '控制指令与 IMU');

    subplot(3, 2, 1);
    plot(time_s, signals.command_yaw, 'LineWidth', 1.3);
    yline_local(cfg.max_overload_g, 'r--');
    yline_local(-cfg.max_overload_g, 'r--');
    grid on; xlabel('时间 (s)'); ylabel('g'); title('偏航过载');
    legend('最终下发', 'Location', 'best');

    subplot(3, 2, 2);
    plot(time_s, signals.command_pitch, 'LineWidth', 1.3);
    yline_local(cfg.max_overload_g, 'r--');
    yline_local(-cfg.max_overload_g, 'r--');
    grid on; xlabel('时间 (s)'); ylabel('g'); title('俯仰过载');
    legend('最终下发', 'Location', 'best');

    subplot(3, 2, 3);
    plot(time_s, signals.roll_rad * 180.0 / pi, 'LineWidth', 1.1);
    grid on; xlabel('时间 (s)'); ylabel('deg'); title('滚转角');

    subplot(3, 2, 4);
    plot(time_s, signals.gyro_x, 'LineWidth', 1.0); hold on;
    plot(time_s, signals.gyro_y, 'LineWidth', 1.0);
    plot(time_s, signals.gyro_z, 'LineWidth', 1.0);
    grid on; xlabel('时间 (s)'); ylabel('rad/s'); title('机体系陀螺仪');
    legend('x', 'y', 'z', 'Location', 'best');

    subplot(3, 2, 5);
    stairs(time_s, mode_code(signals.yaw_mode), 'LineWidth', 1.1); hold on;
    stairs(time_s, mode_code(signals.pitch_mode), 'LineWidth', 1.1);
    grid on; xlabel('时间 (s)'); ylabel('模式编号'); title('Kalman 工作模式');
    set(gca, 'YTick', 0:5, 'YTickLabel', ...
        {'无', '角度初始化', '速度初始化', '更新', '重新对齐', '预测'});
    legend('偏航', '俯仰', 'Location', 'best');

    subplot(3, 2, 6);
    stairs(time_s, signals.filter_reinitialized, 'LineWidth', 1.1); hold on;
    stairs(time_s, signals.gyro_held, 'LineWidth', 1.1);
    stairs(time_s, signals.imu_fault, 'LineWidth', 1.1);
    grid on; ylim([-0.1, 1.2]); xlabel('时间 (s)'); ylabel('状态');
    title('异常与降级状态');
    legend('滤波重新对齐', '保持旧陀螺', 'IMU 故障', 'Location', 'best');

    save_figure(fig, fullfile(output_dir, '04_control_imu.png'));
end

function time_s = build_time_axis(data)
    timestamp_us = numeric_column(data, 'image_timestamp_us');
    valid_timestamp = find(isfinite(timestamp_us), 1, 'first');
    if ~isempty(valid_timestamp)
        time_s = (timestamp_us - timestamp_us(valid_timestamp)) / 1.0e6;
        if all(isfinite(time_s))
            return;
        end
    end

    dt_s = numeric_column(data, 'dt_s');
    dt_s(~isfinite(dt_s) | dt_s <= 0.0) = NaN;
    fallback_dt = finite_median(dt_s);
    if ~isfinite(fallback_dt) || fallback_dt <= 0.0
        fps = numeric_column(data, 'fps');
        fallback_fps = finite_median(fps);
        if ~isfinite(fallback_fps) || fallback_fps <= 0.0
            fallback_dt = 1.0;
        else
            fallback_dt = 1.0 / fallback_fps;
        end
    end
    dt_s(~isfinite(dt_s)) = fallback_dt;
    time_s = [0.0; cumsum(dt_s(2:end))];
end

function values = numeric_column(data, name)
    row_count = height(data);
    if ~ismember(name, data.Properties.VariableNames)
        values = NaN(row_count, 1);
        return;
    end
    raw = data.(name);
    if isnumeric(raw) || islogical(raw)
        values = double(raw);
    else
        values = str2double(string(raw));
    end
    values = reshape(values, [], 1);
end

function values = text_column(data, name)
    row_count = height(data);
    if ~ismember(name, data.Properties.VariableNames)
        values = strings(row_count, 1);
        return;
    end
    values = string(data.(name));
    values(ismissing(values)) = "";
    values = reshape(values, [], 1);
end

function count = count_mode(values, target)
    count = nnz(values == string(target));
end

function codes = mode_code(values)
    codes = zeros(size(values));
    names = ["angle_initialized", "rate_initialized", "updated", ...
        "realigned", "predicted"];
    for index = 1:numel(names)
        codes(values == names(index)) = index;
    end
end

function percent = flag_percent(values)
    valid = isfinite(values);
    if ~any(valid)
        percent = NaN;
    else
        percent = 100.0 * mean(values(valid) > 0.5);
    end
end

function percent = threshold_percent(values, threshold)
    valid = isfinite(values);
    if ~any(valid)
        percent = NaN;
    else
        percent = 100.0 * mean(values(valid) > threshold);
    end
end

function percent = saturation_percent(values, limit)
    valid = isfinite(values);
    if ~any(valid) || limit <= 0.0
        percent = NaN;
    else
        percent = 100.0 * mean(abs(values(valid)) >= 0.98 * limit);
    end
end

function value = finite_mean(values)
    values = values(isfinite(values));
    if isempty(values), value = NaN; else, value = mean(values); end
end

function value = finite_median(values)
    values = values(isfinite(values));
    if isempty(values), value = NaN; else, value = median(values); end
end

function value = finite_max(values)
    values = values(isfinite(values));
    if isempty(values), value = NaN; else, value = max(values); end
end

function value = finite_percentile(values, percentile)
    values = sort(values(isfinite(values)));
    if isempty(values)
        value = NaN;
        return;
    end
    if numel(values) == 1
        value = values(1);
        return;
    end
    position = 1.0 + (numel(values) - 1.0) * percentile / 100.0;
    lower_index = floor(position);
    upper_index = ceil(position);
    weight = position - lower_index;
    value = values(lower_index) * (1.0 - weight) + values(upper_index) * weight;
end

function value = finite_rmse(left, right)
    valid = isfinite(left) & isfinite(right);
    if ~any(valid)
        value = NaN;
    else
        difference = left(valid) - right(valid);
        value = sqrt(mean(difference .* difference));
    end
end

function save_figure(fig, path)
    set(fig, 'Position', [80, 80, 1400, 850]);
    saveas(fig, path);
    fprintf('保存图像：%s\n', path);
end

function yline_local(value, style)
    if exist('yline', 'file') == 2 || exist('yline', 'builtin') == 5
        yline(value, style);
        return;
    end
    limits = xlim;
    plot(limits, [value, value], style);
end
