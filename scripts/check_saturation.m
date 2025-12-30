% CHECK_SATURATION - 检查控制力饱和情况
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

% 仿真到崩溃前
solver = @ode15s;
tspan = [0 7];
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5, 'MaxStep', 0.01);

[t, x] = solver(@(t, x) system_dynamics(t, x, p), tspan, x0, options);

fprintf('仿真完成: %.2f s (共 %d 点)\n\n', t(end), length(t));

% 分析几个关键时刻
t_check = [1.0, 3.0, 5.0, 6.0, t(end)];

for k = 1:length(t_check)
    [~, idx] = min(abs(t - t_check(k)));
    t_k = t(idx);
    x_k = x(idx,:)';

    fprintf('========== t = %.2f s ==========\n', t_k);

    [pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x_k, p.n);
    [pd, dpd, d2pd, d3pd, ~] = trajectory(t_k);

    e = pL - pd;
    ev = vL - dpd;

    % 计算控制力
    M = p.mL * eye(3) + p.mi * (q * q');
    [f_dL, ~, ~] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
    f_qdi = force_allocation(p, f_dL, q, omega);
    [f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);

    % 显示状态
    fprintf('位置误差: [%.4f, %.4f, %.4f] m (模=%.4f)\n', e, norm(e));
    fprintf('速度误差: [%.4f, %.4f, %.4f] m/s (模=%.4f)\n', ev, norm(ev));
    fprintf('载荷速度: [%.4f, %.4f, %.4f] m/s (vz=%.4f)\n', vL, vL(3));

    % 缆绳角度
    e3 = [0;0;1];
    fprintf('\n缆绳配置:\n');
    for i = 1:p.n
        theta_i = acosd(q(:,i)' * e3);
        fprintf('  UAV %d: 天顶角=%.1f° (期望=%.1f°, 误差=%.1f°)\n', ...
            i, theta_i, rad2deg(p.theta_d), theta_i - rad2deg(p.theta_d));
    end

    % 控制力分析
    f_perp_max = 1.5 * p.mi * p.g;
    fprintf('\n控制力 (垂直力限幅=%.2f N):\n', f_perp_max);
    for i = 1:p.n
        f_qdi_norm = norm(f_qdi(:,i));
        f_perp_norm = norm(f_di_perp(:,i));
        f_total_norm = norm(f_qdi(:,i) + f_di_perp(:,i));

        % 检查是否饱和
        is_saturated = (f_perp_norm >= f_perp_max * 0.99);
        sat_marker = '';
        if is_saturated
            sat_marker = ' *** 饱和！***';
        end

        fprintf('  UAV %d: |f_qdi|=%.2f N, |f_perp|=%.2f N, |f_total|=%.2f N%s\n', ...
            i, f_qdi_norm, f_perp_norm, f_total_norm, sat_marker);
    end

    fprintf('\n');
end

quit;
