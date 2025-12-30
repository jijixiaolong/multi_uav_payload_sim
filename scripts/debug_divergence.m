% DEBUG_DIVERGENCE - 调试长时间仿真的发散问题
clear; clc;

script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(project_dir);

p = params();
x0 = init_state(p);

% 仿真配置
solver = @ode15s;
tspan = [0 8];  % 测试到8秒（已知7秒左右会崩溃）
rel_tol = 1e-4;
abs_tol = 1e-5;
max_step = 0.01;

options = odeset('RelTol', rel_tol, 'AbsTol', abs_tol, 'MaxStep', max_step);

% 添加事件检测：当误差超过0.5m时停止
options = odeset(options, 'Events', @(t,x) error_event(t, x, p));

fprintf('开始仿真...\n');
tic;
[t, x, te, xe, ie] = solver(@(t, x) system_dynamics(t, x, p), tspan, x0, options);
elapsed = toc;

fprintf('仿真完成，用时 %.2f s\n', elapsed);
fprintf('仿真时长: %.2f s (共 %d 点)\n\n', t(end), length(t));

% 计算误差历史
N = length(t);
error_hist = zeros(N, 1);
vL_norm_hist = zeros(N, 1);
omega_max_hist = zeros(N, 1);

for k = 1:N
    [pL, vL, ~, ~, omega, ~, ~] = unpack_state(x(k,:)', p.n);
    [pd, ~, ~, ~, ~] = trajectory(t(k));
    error_hist(k) = norm(pL - pd);
    vL_norm_hist(k) = norm(vL);
    omega_max_hist(k) = max(vecnorm(omega, 2, 1));  % 最大角速度
end

fprintf('=== 误差统计 ===\n');
fprintf('最大误差: %.4f m (%.1f cm)\n', max(error_hist), max(error_hist)*100);
fprintf('最终误差: %.4f m (%.1f cm)\n', error_hist(end), error_hist(end)*100);
fprintf('平均误差: %.4f m (%.1f cm)\n\n', mean(error_hist), mean(error_hist)*100);

% 找到误差开始快速增长的时刻
error_rate = diff(error_hist) ./ diff(t);
[max_rate, idx_max_rate] = max(abs(error_rate));
t_diverge = t(idx_max_rate);

fprintf('=== 发散分析 ===\n');
fprintf('误差增长最快时刻: t = %.2f s\n', t_diverge);
fprintf('误差增长率: %.4f m/s\n', max_rate);
fprintf('此时误差: %.4f m\n', error_hist(idx_max_rate));
fprintf('最大角速度: %.2f rad/s\n\n', max(omega_max_hist));

% 详细分析发散时刻前后的状态
if t_diverge > 1.0
    % 找到发散前1秒、发散时、发散后（如果有）的状态
    idx_before = find(t >= t_diverge - 1.0, 1);
    idx_at = idx_max_rate;

    fprintf('=== 详细状态对比 ===\n');
    fprintf('时刻 t = %.2f s (发散前1秒):\n', t(idx_before));
    analyze_state(t(idx_before), x(idx_before,:)', p);

    fprintf('\n时刻 t = %.2f s (发散时刻):\n', t(idx_at));
    analyze_state(t(idx_at), x(idx_at,:)', p);
end

% 绘图
figure('Position', [100, 100, 1200, 800]);

subplot(3,1,1);
plot(t, error_hist*100, 'LineWidth', 1.5);
hold on;
plot([t_diverge, t_diverge], [0, max(error_hist)*100], 'r--', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('跟踪误差 (cm)');
title('跟踪误差随时间变化');
grid on;
legend('跟踪误差', '发散时刻');

subplot(3,1,2);
plot(t, vL_norm_hist, 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('载荷速度 (m/s)');
title('载荷速度模');
grid on;

subplot(3,1,3);
plot(t, omega_max_hist, 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
title('最大UAV角速度');
grid on;

saveas(gcf, fullfile(project_dir, 'results', 'divergence_analysis.png'));

quit;

function [value, isterminal, direction] = error_event(t, x, p)
    % 事件函数：当误差超过0.5m时触发
    [pL, ~, ~, ~, ~, ~, ~] = unpack_state(x, p.n);
    [pd, ~, ~, ~, ~] = trajectory(t);
    error = norm(pL - pd);
    value = 0.5 - error;  % 当误差>0.5m时，value<0，触发事件
    isterminal = 0;  % 不停止仿真
    direction = -1;  % 只在value从正变负时触发
end

function analyze_state(t, x, p)
    [pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x, p.n);
    [pd, dpd, d2pd, ~, ~] = trajectory(t);

    e = pL - pd;
    ev = vL - dpd;

    fprintf('  位置误差: [%.4f, %.4f, %.4f] m (模=%.4f)\n', e, norm(e));
    fprintf('  速度误差: [%.4f, %.4f, %.4f] m/s (模=%.4f)\n', ev, norm(ev));
    fprintf('  载荷速度: [%.4f, %.4f, %.4f] m/s (模=%.4f)\n', vL, norm(vL));
    fprintf('  角速度范围: [%.2f, %.2f] rad/s\n', min(vecnorm(omega, 2, 1)), max(vecnorm(omega, 2, 1)));

    % 检查缆绳角度
    e3 = [0;0;1];
    for i = 1:p.n
        theta_i = acosd(q(:,i)' * e3);  % 与垂直方向夹角
        fprintf('  UAV %d: 缆绳天顶角=%.1f° (期望=%.1f°)\n', i, theta_i, rad2deg(p.theta_d));
    end
end
