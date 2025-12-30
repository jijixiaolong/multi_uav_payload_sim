% verify_results.m - 验证保存的仿真结果正确性
% 用法: 运行此脚本检查最近保存的仿真数据
clear; clc; close all;

% 添加路径
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(project_dir));

% 查找最新的结果文件
results_dir = fullfile(project_dir, 'results');
mat_files = dir(fullfile(results_dir, '*.mat'));

if isempty(mat_files)
    error('未找到仿真结果文件，请先运行 main.m');
end

% 按日期排序，取最新的
[~, idx] = sort([mat_files.datenum], 'descend');
latest_file = fullfile(results_dir, mat_files(idx(1)).name);
fprintf('加载文件: %s\n\n', latest_file);

% 加载数据
data = load(latest_file);

%% 1. 检查数据完整性
fprintf('================================================================================\n');
fprintf('1. 数据完整性检查\n');
fprintf('================================================================================\n');

required_vars = {'t', 'x', 'p', 'pL_hist', 'vL_hist', 'pd_hist', 'elapsed', 'sim_mode'};
missing_vars = {};
for i = 1:length(required_vars)
    if ~isfield(data, required_vars{i})
        missing_vars{end+1} = required_vars{i};
    end
end

if isempty(missing_vars)
    fprintf('✓ 所有必需变量都存在\n');
else
    fprintf('✗ 缺少变量: %s\n', strjoin(missing_vars, ', '));
end

fprintf('\n变量列表:\n');
vars = fieldnames(data);
for i = 1:length(vars)
    v = data.(vars{i});
    if isnumeric(v)
        fprintf('  %s: [%s] %s\n', vars{i}, num2str(size(v)), class(v));
    elseif isstruct(v)
        fprintf('  %s: struct with %d fields\n', vars{i}, length(fieldnames(v)));
    else
        fprintf('  %s: %s\n', vars{i}, class(v));
    end
end

%% 2. 时间序列检查
fprintf('\n================================================================================\n');
fprintf('2. 时间序列检查\n');
fprintf('================================================================================\n');

t = data.t;
fprintf('  仿真模式: %s\n', data.sim_mode);
fprintf('  计算耗时: %.2f 秒\n', data.elapsed);
fprintf('  时间范围: [%.4f, %.4f] 秒\n', t(1), t(end));
fprintf('  采样点数: %d\n', length(t));
fprintf('  平均步长: %.4f 秒\n', mean(diff(t)));
fprintf('  最小步长: %.4f 秒\n', min(diff(t)));
fprintf('  最大步长: %.4f 秒\n', max(diff(t)));

% 检查时间单调递增
if all(diff(t) > 0)
    fprintf('  ✓ 时间单调递增\n');
else
    fprintf('  ✗ 时间非单调递增!\n');
end

%% 3. 状态数据检查
fprintf('\n================================================================================\n');
fprintf('3. 状态数据检查\n');
fprintf('================================================================================\n');

x = data.x;
p = data.p;
fprintf('  状态向量维度: [%d × %d]\n', size(x, 1), size(x, 2));
fprintf('  期望维度: [%d × %d] (采样点 × 状态变量)\n', length(t), 9 + 18*p.n);

% 检查 NaN 和 Inf
nan_count = sum(isnan(x(:)));
inf_count = sum(isinf(x(:)));
fprintf('  NaN 数量: %d\n', nan_count);
fprintf('  Inf 数量: %d\n', inf_count);

if nan_count == 0 && inf_count == 0
    fprintf('  ✓ 状态数据无异常值\n');
else
    fprintf('  ✗ 状态数据存在异常值!\n');
end

%% 4. 载荷位置检查
fprintf('\n================================================================================\n');
fprintf('4. 载荷位置检查\n');
fprintf('================================================================================\n');

pL_hist = data.pL_hist;
pd_hist = data.pd_hist;

fprintf('  初始位置 pL(0): [%.4f, %.4f, %.4f] m\n', pL_hist(1,:));
fprintf('  最终位置 pL(T): [%.4f, %.4f, %.4f] m\n', pL_hist(end,:));
fprintf('  期望初始 pd(0): [%.4f, %.4f, %.4f] m\n', pd_hist(1,:));
fprintf('  期望最终 pd(T): [%.4f, %.4f, %.4f] m\n', pd_hist(end,:));

% 计算跟踪误差
error_hist = pL_hist - pd_hist;
error_norm = sqrt(sum(error_hist.^2, 2));

fprintf('\n  跟踪误差分析:\n');
fprintf('    初始误差: %.6f m\n', error_norm(1));
fprintf('    最终误差: %.6f m\n', error_norm(end));
fprintf('    最大误差: %.6f m\n', max(error_norm));
fprintf('    平均误差: %.6f m\n', mean(error_norm));

if error_norm(end) < error_norm(1) || max(error_norm) < 0.5
    fprintf('  ✓ 跟踪误差在合理范围内\n');
else
    fprintf('  ⚠ 跟踪误差可能较大\n');
end

%% 5. 速度检查
fprintf('\n================================================================================\n');
fprintf('5. 速度检查\n');
fprintf('================================================================================\n');

vL_hist = data.vL_hist;
fprintf('  初始速度 vL(0): [%.4f, %.4f, %.4f] m/s\n', vL_hist(1,:));
fprintf('  最终速度 vL(T): [%.4f, %.4f, %.4f] m/s\n', vL_hist(end,:));

% 期望速度 (从轨迹函数)
[~, dpd, ~, ~, ~] = trajectory(0);
fprintf('  期望速度 dpd:   [%.4f, %.4f, %.4f] m/s\n', dpd);

vL_norm = sqrt(sum(vL_hist.^2, 2));
fprintf('\n  速度模长:\n');
fprintf('    最大速度: %.4f m/s\n', max(vL_norm));
fprintf('    平均速度: %.4f m/s\n', mean(vL_norm));

if max(vL_norm) < 10
    fprintf('  ✓ 速度在合理范围内\n');
else
    fprintf('  ⚠ 速度可能过大\n');
end

%% 6. 系统稳定性检查
fprintf('\n================================================================================\n');
fprintf('6. 系统稳定性检查\n');
fprintf('================================================================================\n');

% 检查状态是否发散
x_max = max(abs(x(:)));
fprintf('  状态最大绝对值: %.4f\n', x_max);

% 检查最后 10% 的数据是否稳定
last_10_pct = round(0.9 * length(t)):length(t);
if length(last_10_pct) > 1
    error_last = error_norm(last_10_pct);
    error_trend = polyfit(1:length(error_last), error_last', 1);
    fprintf('  最后10%%误差趋势斜率: %.6f (负=收敛, 正=发散)\n', error_trend(1));

    if error_trend(1) < 0.01
        fprintf('  ✓ 系统趋于稳定\n');
    elseif error_trend(1) > 0.1
        fprintf('  ⚠ 系统可能发散\n');
    else
        fprintf('  ~ 系统基本稳定\n');
    end
end

if x_max < 1e6
    fprintf('  ✓ 状态未发散\n');
else
    fprintf('  ✗ 状态可能已发散!\n');
end

%% 7. 验证 pL_hist 与 x 的一致性
fprintf('\n================================================================================\n');
fprintf('7. pL_hist 与原始状态 x 的一致性\n');
fprintf('================================================================================\n');

% 从 x 重新解包几个点验证
check_indices = [1, round(length(t)/2), length(t)];
max_diff = 0;
for idx = check_indices
    [pL_check, ~, ~, ~, ~, ~, ~] = unpack_state(x(idx,:)', p.n);
    diff_val = norm(pL_check' - pL_hist(idx,:));
    max_diff = max(max_diff, diff_val);
    fprintf('  t=%.3f: pL差异 = %.2e\n', t(idx), diff_val);
end

if max_diff < 1e-10
    fprintf('  ✓ pL_hist 与原始状态一致\n');
else
    fprintf('  ✗ pL_hist 与原始状态不一致!\n');
end

%% 8. 绘制验证图
fprintf('\n================================================================================\n');
fprintf('8. 生成验证图\n');
fprintf('================================================================================\n');

figure('Name', '仿真结果验证', 'Position', [100, 100, 1200, 800]);

% 子图1: 载荷位置
subplot(2,3,1);
plot(t, pL_hist(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,1), 'r--', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('X (m)');
title('载荷 X 位置');
legend('实际', '期望', 'Location', 'best');
grid on;

subplot(2,3,2);
plot(t, pL_hist(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,2), 'r--', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('Y (m)');
title('载荷 Y 位置');
legend('实际', '期望', 'Location', 'best');
grid on;

subplot(2,3,3);
plot(t, pL_hist(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,3), 'r--', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('Z (m)');
title('载荷 Z 位置 (NED, 向下为正)');
legend('实际', '期望', 'Location', 'best');
grid on;

% 子图4: 跟踪误差
subplot(2,3,4);
plot(t, error_norm, 'k-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('误差 (m)');
title('位置跟踪误差 ||e||');
grid on;

% 子图5: 速度
subplot(2,3,5);
plot(t, vL_hist(:,1), 'r-', 'LineWidth', 1); hold on;
plot(t, vL_hist(:,2), 'g-', 'LineWidth', 1);
plot(t, vL_hist(:,3), 'b-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('速度 (m/s)');
title('载荷速度');
legend('v_x', 'v_y', 'v_z', 'Location', 'best');
grid on;

% 子图6: 3D 轨迹
subplot(2,3,6);
plot3(pL_hist(:,1), pL_hist(:,2), -pL_hist(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot3(pd_hist(:,1), pd_hist(:,2), -pd_hist(:,3), 'r--', 'LineWidth', 1);
plot3(pL_hist(1,1), pL_hist(1,2), -pL_hist(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(pL_hist(end,1), pL_hist(end,2), -pL_hist(end,3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('高度 (m)');
title('3D 轨迹 (绿圈=起点, 红叉=终点)');
legend('实际', '期望', 'Location', 'best');
grid on;
axis equal;
view(45, 30);

fprintf('✓ 验证图已生成\n');

%% 总结
fprintf('\n================================================================================\n');
fprintf('验证总结\n');
fprintf('================================================================================\n');
fprintf('文件: %s\n', mat_files(idx(1)).name);
fprintf('仿真时长: %.2f s, 采样点: %d, 计算耗时: %.2f s\n', t(end)-t(1), length(t), data.elapsed);
fprintf('最终跟踪误差: %.6f m\n', error_norm(end));
fprintf('================================================================================\n');
