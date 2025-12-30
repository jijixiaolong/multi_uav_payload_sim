% TEST_HOVER - 测试悬停状态，最简单的情况
% 期望轨迹：固定位置悬停

clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(project_dir);

% 修改轨迹函数为悬停
copyfile(fullfile(project_dir, 'trajectory.m'), fullfile(project_dir, 'trajectory_backup.m'));

% 创建悬停轨迹
fid = fopen(fullfile(project_dir, 'trajectory.m'), 'w');
fprintf(fid, 'function [pd, dpd, d2pd, d3pd, d4pd] = trajectory(t)\n');
fprintf(fid, '%% TRAJECTORY - 悬停测试\n');
fprintf(fid, 'pd = [0; 0; -2];\n');
fprintf(fid, 'dpd = [0; 0; 0];\n');
fprintf(fid, 'd2pd = [0; 0; 0];\n');
fprintf(fid, 'd3pd = [0; 0; 0];\n');
fprintf(fid, 'd4pd = [0; 0; 0];\n');
fprintf(fid, 'end\n');
fclose(fid);

% 加载参数
p = params();

% 初始化状态 - 使用原始初始化（计算正确的初始姿态）
x0 = init_state(p);

% 仿真
solver = @ode15s;
tspan = [0 5];
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5, 'MaxStep', 0.01);

fprintf('开始悬停测试...\n');
tic;
[t, x] = solver(@(t, x) system_dynamics(t, x, p), tspan, x0, options);
elapsed = toc;

% 提取结果
N = length(t);
pL_hist = zeros(N, 3);
vL_hist = zeros(N, 3);
for k = 1:N
    [pL_k, vL_k, ~, ~, ~, ~, ~] = unpack_state(x(k,:)', p.n);
    pL_hist(k,:) = pL_k';
    vL_hist(k,:) = vL_k';
end

fprintf('仿真完成，用时 %.2f 秒\n', elapsed);
fprintf('\n=== 最终状态 (t=%.2f) ===\n', t(end));
fprintf('位置: [%.3f, %.3f, %.3f] m\n', pL_hist(end,:));
fprintf('速度: [%.3f, %.3f, %.3f] m/s\n', vL_hist(end,:));
fprintf('位置误差: [%.3f, %.3f, %.3f] m\n', pL_hist(end,:) - [0, 0, -2]);

% 检查稳定性
pos_error = vecnorm(pL_hist - repmat([0, 0, -2], N, 1), 2, 2);
max_error = max(pos_error);
final_error = pos_error(end);

if max_error < 0.1
    fprintf('\n✓ 悬停稳定！最大误差 = %.3f m\n', max_error);
elseif max_error < 0.5
    fprintf('\n△ 悬停基本稳定，最大误差 = %.3f m\n', max_error);
else
    fprintf('\n✗ 悬停不稳定！最大误差 = %.3f m\n', max_error);
end

% 绘图
figure('Position', [100, 100, 1200, 800]);

subplot(2,3,1);
plot(t, pL_hist(:,1), 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X (m)');
title('X Position'); grid on;
legend('Actual', 'Desired');

subplot(2,3,2);
plot(t, pL_hist(:,2), 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Y (m)');
title('Y Position'); grid on;
legend('Actual', 'Desired');

subplot(2,3,3);
plot(t, pL_hist(:,3), 'LineWidth', 1.5); hold on;
yline(-2, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Z (m)');
title('Z Position (Height)'); grid on;
legend('Actual', 'Desired');

subplot(2,3,4);
plot(t, vL_hist(:,1), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('vX (m/s)');
title('X Velocity'); grid on;

subplot(2,3,5);
plot(t, vL_hist(:,2), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('vY (m/s)');
title('Y Velocity'); grid on;

subplot(2,3,6);
plot(t, vL_hist(:,3), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('vZ (m/s)');
title('Z Velocity'); grid on;

sgtitle('悬停测试');

% 恢复原始轨迹
movefile(fullfile(project_dir, 'trajectory_backup.m'), fullfile(project_dir, 'trajectory.m'));
