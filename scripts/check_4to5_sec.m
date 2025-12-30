% CHECK_4TO5_SEC - 详细检查4-5秒之间的状态突变
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

% 仿真到5.5秒
solver = @ode15s;
tspan = [0 5.5];
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5, 'MaxStep', 0.01);

[t, x] = solver(@(t, x) system_dynamics(t, x, p), tspan, x0, options);

fprintf('仿真完成: %.2f s (共 %d 点)\n\n', t(end), length(t));

% 重点检查 4.0 ~ 5.0 秒之间
t_check = 4.0:0.1:5.0;

fprintf('时间     缆绳角度     位置误差(Z)    速度(Z)      推力合力      角速度\n');
fprintf('-------------------------------------------------------------------------------------\n');

for k = 1:length(t_check)
    [~, idx] = min(abs(t - t_check(k)));
    t_k = t(idx);
    x_k = x(idx,:)';

    [pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x_k, p.n);
    [pd, dpd, d2pd, d3pd, ~] = trajectory(t_k);

    e = pL - pd;
    ev = vL - dpd;

    % 计算控制力
    M = p.mL * eye(3) + p.mi * (q * q');
    [f_dL, ~, ~] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
    f_qdi = force_allocation(p, f_dL, q, omega);
    [f_di_perp, ~, ~, ~, ~] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
    [Omega_cmd, T] = attitude_ctrl(p, f_qdi, f_di_perp, zeros(3,p.n), zeros(3,p.n), zeros(3,p.n), zeros(3,p.n), ev, e, M, R_all, q, omega);

    % 缆绳角度
    e3 = [0;0;1];
    theta_avg = mean(acosd(sum(q .* e3, 1)));

    % 推力
    T_avg = mean(T);
    omega_max = max(vecnorm(omega, 2, 1));

    fprintf('%.2f s   %.1f°        %.4f m     %.4f m/s   %.2f N      %.2f rad/s\n', ...
        t_k, theta_avg, e(3), vL(3), T_avg, omega_max);
end

fprintf('\n');

% 检查推力指令的变化
fprintf('=== 推力指令详细分析 ===\n');
t_critical = [4.0, 4.5, 4.8, 4.9, 5.0];

for k = 1:length(t_critical)
    [~, idx] = min(abs(t - t_critical(k)));
    t_k = t(idx);
    x_k = x(idx,:)';

    [pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x_k, p.n);
    [pd, dpd, d2pd, d3pd, ~] = trajectory(t_k);

    e = pL - pd;
    ev = vL - dpd;

    % 计算控制量
    M = p.mL * eye(3) + p.mi * (q * q');
    [f_dL, ~, ~] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
    f_qdi = force_allocation(p, f_dL, q, omega);
    [f_di_perp, ~, ~, ~, ~] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);

    f_di = f_qdi + f_di_perp;
    [Omega_cmd, T] = attitude_ctrl(p, f_qdi, f_di_perp, zeros(3,p.n), zeros(3,p.n), zeros(3,p.n), zeros(3,p.n), ev, e, M, R_all, q, omega);

    fprintf('\nt = %.2f s:\n', t_k);
    for i = 1:p.n
        r_i = R_all(:,:,i) * [0;0;1];
        r_di = -f_di(:,i) / norm(f_di(:,i));
        dot_product = r_di' * r_i;
        T_di = norm(f_di(:,i));

        fprintf('  UAV %d: T_di=%.2f N, r_di·r_i=%.4f, T_i=%.2f N\n', ...
            i, T_di, dot_product, T(i));
    end
end

quit;
