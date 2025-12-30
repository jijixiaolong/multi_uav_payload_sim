% TEST_FROM_REST - 测试从静止状态启动
% 修改轨迹使初始速度为0，看系统是否稳定

clear; clc; close all;

% 设置路径
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(genpath(fullfile(project_dir, 'plot')));
addpath(project_dir);

% 加载参数
p = params();

% 初始化状态 - 修改为静止
n = p.n;
pL0 = [0; 0; -2];
vL0 = [0; 0; 0];  % 从静止开始

% 期望缆绳方向 - 垂直向下
theta_d = p.theta_d;
q_all = zeros(3,n);
omega_all = zeros(3,n);

for i = 1:n
    psi_di = deg2rad(90 + (i-1)*120);
    q_di = [cos(psi_di)*sin(theta_d); sin(psi_di)*sin(theta_d); cos(theta_d)];
    q_all(:,i) = q_di / norm(q_di);
    omega_all(:,i) = [0; 0; 0];
end

% 计算初始旋转矩阵 - 推力垂直向上
e3 = [0; 0; 1];
R_all = repmat(eye(3), [1, 1, n]);

% 扰动估计初值
dL_hat0 = [0; 0; 0];
d_hat0 = zeros(3,n);

% 打包状态
x0 = pack_state(pL0, vL0, R_all, q_all, omega_all, dL_hat0, d_hat0);

% 仿真设置
solver = @ode15s;
tspan = [0 2];  % 2秒测试
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5, 'MaxStep', 0.01);

fprintf('开始仿真 (从静止状态)...\n');
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

% 计算期望轨迹
pd_hist = zeros(N, 3);
for k = 1:N
    [pd_k, ~, ~, ~, ~] = trajectory(t(k));
    pd_hist(k,:) = pd_k';
end

fprintf('仿真完成，用时 %.2f 秒\n', elapsed);
fprintf('\n=== 最终状态 (t=%.2f) ===\n', t(end));
fprintf('位置: [%.3f, %.3f, %.3f] m\n', pL_hist(end,:));
fprintf('速度: [%.3f, %.3f, %.3f] m/s\n', vL_hist(end,:));
fprintf('期望位置: [%.3f, %.3f, %.3f] m\n', pd_hist(end,:));
fprintf('位置误差: [%.3f, %.3f, %.3f] m\n', pL_hist(end,:) - pd_hist(end,:));

% 检查是否发散
max_pos_error = max(vecnorm(pL_hist - pd_hist, 2, 2));
if max_pos_error > 1.0
    fprintf('\n警告：系统发散！最大位置误差 = %.3f m\n', max_pos_error);
else
    fprintf('\n系统稳定，最大位置误差 = %.3f m\n', max_pos_error);
end

% 绘制结果
figure('Position', [100, 100, 1200, 400]);

subplot(1,3,1);
plot(t, pL_hist(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X (m)');
legend('Actual', 'Desired');
title('X Position');
grid on;

subplot(1,3,2);
plot(t, pL_hist(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Y (m)');
legend('Actual', 'Desired');
title('Y Position');
grid on;

subplot(1,3,3);
plot(t, pL_hist(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_hist(:,3), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Z (m)');
legend('Actual', 'Desired');
title('Z Position (Height)');
grid on;

sgtitle('从静止状态启动测试');
