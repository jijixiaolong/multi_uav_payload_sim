% ANALYZE_RESULT - 分析最新的仿真结果

clear; clc;

% 找到最新的结果文件
results_dir = '../results/';
files = dir(fullfile(results_dir, 'sim_*.mat'));
if isempty(files)
    error('没有找到结果文件');
end

[~, idx] = max([files.datenum]);
latest_file = fullfile(results_dir, files(idx).name);

fprintf('加载文件: %s\n', latest_file);
load(latest_file);

fprintf('\n=== 仿真统计 ===\n');
fprintf('时间范围: [%.2f, %.2f] s\n', t(1), t(end));
fprintf('数据点数: %d\n', length(t));

% 计算误差
error_vec = pL_hist - pd_hist;
error_norm = vecnorm(error_vec, 2, 2);

fprintf('\n=== 位置误差 ===\n');
fprintf('最大误差: %.4f m\n', max(error_norm));
fprintf('最终误差: %.4f m\n', error_norm(end));
fprintf('平均误差: %.4f m\n', mean(error_norm));

% 找到误差最大的时刻
[max_err, max_idx] = max(error_norm);
fprintf('\n=== 最大误差时刻 (t=%.3f) ===\n', t(max_idx));
fprintf('实际位置: [%.3f, %.3f, %.3f] m\n', pL_hist(max_idx,:));
fprintf('期望位置: [%.3f, %.3f, %.3f] m\n', pd_hist(max_idx,:));
fprintf('误差向量: [%.3f, %.3f, %.3f] m\n', error_vec(max_idx,:));

% 绘图
figure('Position', [100, 100, 1400, 900]);

% 位置对比
subplot(3,3,1);
plot(t, pL_hist(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X (m)');
legend('Actual', 'Desired', 'Location', 'best');
title('X Position'); grid on;

subplot(3,3,2);
plot(t, pL_hist(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Y (m)');
legend('Actual', 'Desired', 'Location', 'best');
title('Y Position'); grid on;

subplot(3,3,3);
plot(t, pL_hist(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,3), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Z (m)');
legend('Actual', 'Desired', 'Location', 'best');
title('Z Position (Height)'); grid on;

% 速度
subplot(3,3,4);
plot(t, vL_hist(:,1), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('vX (m/s)');
title('X Velocity'); grid on;

subplot(3,3,5);
plot(t, vL_hist(:,2), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('vY (m/s)');
title('Y Velocity'); grid on;

subplot(3,3,6);
plot(t, vL_hist(:,3), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('vZ (m/s)');
title('Z Velocity'); grid on;

% 位置误差
subplot(3,3,7);
plot(t, error_vec(:,1), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error X (m)');
title('X Position Error'); grid on;

subplot(3,3,8);
plot(t, error_vec(:,2), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error Y (m)');
title('Y Position Error'); grid on;

subplot(3,3,9);
plot(t, error_vec(:,3), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error Z (m)');
title('Z Position Error'); grid on;

% 误差范数
figure('Position', [100, 100, 800, 400]);
plot(t, error_norm, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position Error (m)');
title(sprintf('Position Tracking Error (max: %.3f m)', max_err));
grid on;
hold on;
plot(t(max_idx), max_err, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
legend('Error', 'Max Error', 'Location', 'best');
