% CHECK_OMEGA_CMD - 检查角速度指令是否过大
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

solver = @ode15s;
tspan = [0 4.5];
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5, 'MaxStep', 0.01);

[t, x] = solver(@(t, x) system_dynamics(t, x, p), tspan, x0, options);

fprintf('仿真完成: %.2f s\n\n', t(end));

% 检查关键时刻的角速度指令
t_check = [1.0, 2.0, 3.0, 4.0, t(end)];

fprintf('时间     缆绳角度   ||Omega_cmd|| (max)   ||omega|| (max)   推力(avg)\n');
fprintf('--------------------------------------------------------------------------\n');

for k = 1:length(t_check)
    [~, idx] = min(abs(t - t_check(k)));
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
    [f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
    [Omega_cmd, T] = attitude_ctrl(p, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R_all, q, omega);

    % 统计
    e3 = [0;0;1];
    theta_avg = mean(acosd(sum(q .* e3, 1)));
    omega_cmd_max = max(vecnorm(Omega_cmd, 2, 1));
    omega_max = max(vecnorm(omega, 2, 1));
    T_avg = mean(T);

    fprintf('%.2f s   %.1f°        %.2f rad/s          %.2f rad/s      %.2f N\n', ...
        t_k, theta_avg, omega_cmd_max, omega_max, T_avg);

    % 检查是否有异常大的角速度指令
    if omega_cmd_max > 10
        fprintf('  *** 警告：角速度指令过大！***\n');
        fprintf('  各UAV角速度指令:\n');
        for i = 1:p.n
            fprintf('    UAV %d: [%.2f, %.2f, %.2f] rad/s (模=%.2f)\n', ...
                i, Omega_cmd(:,i), norm(Omega_cmd(:,i)));
        end
    end
end

quit;
