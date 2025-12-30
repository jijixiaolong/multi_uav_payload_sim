% CHECK_FORCE_DECREASE - 追踪期望力降低的原因
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

% 仿真
solver = @ode15s;
tspan = [0 5.5];
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5, 'MaxStep', 0.01);

[t, x] = solver(@(t, x) system_dynamics(t, x, p), tspan, x0, options);

fprintf('分析期望力 f_di 的演化\n\n');
fprintf('时间     ||f_dL||   ||f_qdi||  ||f_perp||  ||f_di||   缆绳角   f_dL(z)\n');
fprintf('---------------------------------------------------------------------------------\n');

t_check = 4.0:0.2:5.0;

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
    [f_di_perp, ~, ~, ~, ~] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);

    f_di = f_qdi + f_di_perp;

    % 统计
    norm_f_dL = norm(f_dL);
    norm_f_qdi = mean(vecnorm(f_qdi, 2, 1));
    norm_f_perp = mean(vecnorm(f_di_perp, 2, 1));
    norm_f_di = mean(vecnorm(f_di, 2, 1));

    e3 = [0;0;1];
    theta_avg = mean(acosd(sum(q .* e3, 1)));

    fprintf('%.2f s   %.4f   %.4f    %.4f     %.4f    %.1f°    %.4f\n', ...
        t_k, norm_f_dL, norm_f_qdi, norm_f_perp, norm_f_di, theta_avg, f_dL(3));
end

fprintf('\n=== 详细分析 t=4.0s 和 t=5.0s ===\n\n');

for t_target = [4.0, 5.0]
    [~, idx] = min(abs(t - t_target));
    t_k = t(idx);
    x_k = x(idx,:)';

    [pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x_k, p.n);
    [pd, dpd, d2pd, d3pd, ~] = trajectory(t_k);

    e = pL - pd;
    ev = vL - dpd;

    fprintf('--- t = %.2f s ---\n', t_k);
    fprintf('位置误差 e = [%.6f, %.6f, %.6f] m\n', e);
    fprintf('速度误差 ev = [%.6f, %.6f, %.6f] m/s\n', ev);

    % 质量矩阵
    M = p.mL * eye(3) + p.mi * (q * q');
    fprintf('\nM矩阵对角元: [%.4f, %.4f, %.4f]\n', diag(M));

    % payload_ctrl计算
    [f_dL, ~, ~] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
    fprintf('\n载荷控制 f_dL = [%.4f, %.4f, %.4f] m/s²\n', f_dL);
    fprintf('  ||f_dL|| = %.4f m/s²\n', norm(f_dL));

    % force_allocation
    f_qdi = force_allocation(p, f_dL, q, omega);
    fprintf('\n力分配 f_qdi (UAV 1) = [%.4f, %.4f, %.4f] N\n', f_qdi(:,1));
    fprintf('  ||f_qdi|| = %.4f N\n', norm(f_qdi(:,1)));

    % cable_ctrl
    [f_di_perp, ~, ~, ~, ~] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
    fprintf('\n缆绳控制 f_perp (UAV 1) = [%.4f, %.4f, %.4f] N\n', f_di_perp(:,1));
    fprintf('  ||f_perp|| = %.4f N\n', norm(f_di_perp(:,1)));

    f_di = f_qdi(:,1) + f_di_perp(:,1);
    fprintf('\n总期望力 f_di (UAV 1) = [%.4f, %.4f, %.4f] N\n', f_di);
    fprintf('  ||f_di|| = %.4f N\n\n', norm(f_di));

    % 缆绳配置
    e3 = [0;0;1];
    fprintf('缆绳方向 q (UAV 1) = [%.4f, %.4f, %.4f]\n', q(:,1));
    fprintf('  天顶角 = %.2f°\n\n', acosd(q(:,1)' * e3));
end

quit;
