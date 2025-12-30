% MONITOR_SIM - 监控仿真过程，记录关键变量
% 使用 OutputFcn 在每个时间步记录状态

clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(project_dir);

% 加载参数
p = params();

% 初始化状态
x0 = init_state(p);

% 全局变量用于记录
global monitor_data;
monitor_data = struct();
monitor_data.t = [];
monitor_data.pL = [];
monitor_data.vL = [];
monitor_data.T = [];
monitor_data.f_total = [];
monitor_data.accel = [];

% Output function
function status = output_fcn(t, x, flag, p)
    global monitor_data;
    status = 0;

    if strcmp(flag, 'init') || strcmp(flag, '')
        % 记录当前状态
        if ~isempty(x)
            % 取最后一列（当前时间步）
            x_current = x(:,end);
            t_current = t(end);

            % 解包状态
            [pL, vL, R, q, omega, ~, ~] = unpack_state(x_current, p.n);

            % 获取轨迹
            [pd, dpd, d2pd, d3pd, d4pd] = trajectory(t_current);

            % 计算M
            M = p.mL * eye(3) + p.mi * (q * q') + 1e-8 * eye(3);

            % 计算控制量
            dL_hat = zeros(3,1);
            d_hat = zeros(3, p.n);
            [f_dL, e, ev] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
            f_qdi = force_allocation(p, f_dL, q, omega);
            [f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = ...
                cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
            [~, T] = attitude_ctrl(p, f_qdi, f_di_perp, omega_di, ...
                dot_omega_di, e_qi, e_omega_i, ev, e, M, R, q, omega);

            % 计算合推力
            e3 = [0;0;1];
            f_total = zeros(3,1);
            for i = 1:p.n
                f_total = f_total - T(i) * R(:,:,i) * e3;
            end

            % 计算加速度（根据动力学）
            f_proj = sum(q .* (-repmat(f_total, 1, p.n) / p.n), 1);
            omega_norm_sq = sum(omega.^2, 1);
            coeff = f_proj - p.mi * p.li * omega_norm_sq;
            sum_terms = q * coeff';
            accel = M \ sum_terms + p.g * e3;

            % 记录
            monitor_data.t(end+1) = t_current;
            monitor_data.pL(:,end+1) = pL;
            monitor_data.vL(:,end+1) = vL;
            monitor_data.T(:,end+1) = T';
            monitor_data.f_total(:,end+1) = f_total;
            monitor_data.accel(:,end+1) = accel;
        end
    end
end

% 设置求解器选项
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5, 'MaxStep', 0.01, ...
    'OutputFcn', @(t, x, flag) output_fcn(t, x, flag, p));

fprintf('开始仿真...\n');
tic;
[t, x] = ode15s(@(t, x) system_dynamics(t, x, p), [0 0.5], x0, options);
elapsed = toc;
fprintf('仿真完成，用时 %.2f 秒\n', elapsed);

% 分析记录的数据
t_mon = monitor_data.t;
pL_mon = monitor_data.pL;
vL_mon = monitor_data.vL;
T_mon = monitor_data.T;
f_total_mon = monitor_data.f_total;
accel_mon = monitor_data.accel;

fprintf('\n=== 监控数据统计 ===\n');
fprintf('记录点数: %d\n', length(t_mon));
fprintf('时间范围: [%.3f, %.3f] s\n', t_mon(1), t_mon(end));

% 找到问题时刻
z_pos = pL_mon(3,:);
z_accel = accel_mon(3,:);
total_thrust = sum(T_mon, 1);

% 找到Z加速度异常的时刻
abnormal_idx = find(abs(z_accel) > 1.0, 1);
if ~isempty(abnormal_idx)
    fprintf('\n=== 首次异常时刻 (t=%.3f) ===\n', t_mon(abnormal_idx));
    fprintf('Z位置: %.3f m\n', z_pos(abnormal_idx));
    fprintf('Z速度: %.3f m/s\n', vL_mon(3, abnormal_idx));
    fprintf('Z加速度: %.3f m/s² (异常！)\n', z_accel(abnormal_idx));
    fprintf('总推力: %.3f N\n', total_thrust(abnormal_idx));
end

% 绘图
figure('Position', [100, 100, 1400, 900]);

subplot(3,3,1);
plot(t_mon, pL_mon(3,:), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Z (m)');
title('Z Position'); grid on;

subplot(3,3,2);
plot(t_mon, vL_mon(3,:), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('vZ (m/s)');
title('Z Velocity'); grid on;

subplot(3,3,3);
plot(t_mon, accel_mon(3,:), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('aZ (m/s²)');
title('Z Acceleration'); grid on;
yline(0, 'r--');

subplot(3,3,4);
plot(t_mon, total_thrust, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Total Thrust (N)');
title('Total Thrust'); grid on;
yline(p.mL * p.g + p.n * p.mi * p.g, 'r--', 'Hover');

subplot(3,3,5);
plot(t_mon, T_mon(1,:), 'LineWidth', 1.5); hold on;
plot(t_mon, T_mon(2,:), 'LineWidth', 1.5);
plot(t_mon, T_mon(3,:), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Individual Thrust');
legend('UAV1', 'UAV2', 'UAV3');
grid on;

subplot(3,3,6);
plot(t_mon, f_total_mon(3,:), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('fZ (N)');
title('Total Force Z'); grid on;
yline(-(p.mL * p.g + p.n * p.mi * p.g), 'r--', 'Gravity');

subplot(3,3,7);
plot(t_mon, pL_mon(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X (m)');
title('X Position'); grid on;

subplot(3,3,8);
plot(t_mon, pL_mon(2,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Y (m)');
title('Y Position'); grid on;

subplot(3,3,9);
plot(t_mon, vecnorm(accel_mon, 2, 1), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('|a| (m/s²)');
title('Acceleration Magnitude'); grid on;

sgtitle('仿真监控数据');
