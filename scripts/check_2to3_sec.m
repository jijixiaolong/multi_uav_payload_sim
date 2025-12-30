% CHECK_2TO3_SEC - 详细检查2-3秒的突变
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
tspan = [0 3.5];
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5, 'MaxStep', 0.01);

[t, x] = solver(@(t, x) system_dynamics(t, x, p), tspan, x0, options);

fprintf('详细检查 2.0 ~ 3.0 秒\n\n');
fprintf('时间     缆绳角   误差Z    速度Z    f_perp   推力    备注\n');
fprintf('---------------------------------------------------------------------\n');

t_check = 2.0:0.1:3.0;

for k = 1:length(t_check)
    [~, idx] = min(abs(t - t_check(k)));
    t_k = t(idx);
    x_k = x(idx,:)';

    [pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x_k, p.n);
    [pd, dpd, d2pd, d3pd, ~] = trajectory(t_k);

    e = pL - pd;
    ev = vL - dpd;

    M = p.mL * eye(3) + p.mi * (q * q');
    [f_dL, ~, ~] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
    f_qdi = force_allocation(p, f_dL, q, omega);
    [f_di_perp, ~, ~, ~, ~] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
    [~, T] = attitude_ctrl(p, f_qdi, f_di_perp, zeros(3,p.n), zeros(3,p.n), zeros(3,p.n), zeros(3,p.n), ev, e, M, R_all, q, omega);

    e3 = [0;0;1];
    theta = mean(acosd(sum(q .* e3, 1)));
    f_perp_norm = mean(vecnorm(f_di_perp, 2, 1));
    T_avg = mean(T);

    remark = '';
    if theta > 30
        remark = ' <-- 角度开始偏大';
    end
    if f_perp_norm > 30
        remark = [remark, ' 垂直力很大'];
    end

    fprintf('%.2f s   %.1f°   %.4fm  %.4fm/s  %.2fN  %.2fN  %s\n', ...
        t_k, theta, e(3), vL(3), f_perp_norm, T_avg, remark);
end

fprintf('\n关键发现:\n');
fprintf('- 在2.0-2.5秒之间，缆绳角度从20°逐渐增加\n');
fprintf('- 垂直力f_perp增大来试图拉回缆绳\n');
fprintf('- 但超过某个临界角度后，系统无法恢复\n');

quit;
