% COMPARE_DVL - 对比手工计算和 system_dynamics 的 dvL
clear; clc;

script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(project_dir);

% 创建悬停轨迹
copyfile(fullfile(project_dir, 'trajectory.m'), fullfile(project_dir, 'trajectory_backup.m'));
fid = fopen(fullfile(project_dir, 'trajectory.m'), 'w');
fprintf(fid, 'function [pd, dpd, d2pd, d3pd, d4pd] = trajectory(t)\n');
fprintf(fid, 'pd = [0; 0; -2];\n');
fprintf(fid, 'dpd = [0; 0; 0];\n');
fprintf(fid, 'd2pd = [0; 0; 0];\n');
fprintf(fid, 'd3pd = [0; 0; 0];\n');
fprintf(fid, 'd4pd = [0; 0; 0];\n');
fprintf(fid, 'end\n');
fclose(fid);

p = params();
x0 = init_state(p);
[pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x0, p.n);

% === 手工计算 ===
[pd, dpd, d2pd, d3pd, d4pd] = trajectory(0);
M = p.mL * eye(3) + p.mi * (q * q');

[f_dL, e, ev] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
f_qdi = force_allocation(p, f_dL, q, omega);
[f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
[Omega_cmd_manual, T_manual] = attitude_ctrl(p, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R_all, q, omega);

fprintf('=== 手工计算 ===\n');
fprintf('T_manual = [%.4f, %.4f, %.4f] N\n', T_manual);

e3 = [0;0;1];
f_manual = zeros(3, p.n);
for i = 1:p.n
    f_manual(:,i) = -T_manual(i) * R_all(:,:,i) * e3;
end

f_qi_manual = zeros(3, p.n);
for i = 1:p.n
    f_qi_manual(:,i) = (q(:,i)' * f_manual(:,i)) * q(:,i);
end
sum_f_qi_manual = sum(f_qi_manual, 2);
dvL_manual = M \ sum_f_qi_manual + p.g * e3;

fprintf('Σ(f_qi) = [%.4f, %.4f, %.4f] N\n', sum_f_qi_manual);
fprintf('dvL = [%.6f, %.6f, %.6f] m/s²\n\n', dvL_manual);

% === system_dynamics 计算 ===
dx = system_dynamics(0, x0, p);
[~, dvL_sys, ~, ~, ~, ~, ~] = unpack_state(dx, p.n);

fprintf('=== system_dynamics 计算 ===\n');
fprintf('dvL = [%.6f, %.6f, %.6f] m/s²\n\n', dvL_sys);

% 对比
diff = dvL_sys - dvL_manual;
fprintf('=== 差异 ===\n');
fprintf('Δ dvL = [%.6f, %.6f, %.6f] m/s²\n', diff);
fprintf('差异模 = %.6f m/s²\n', norm(diff));

if norm(diff) > 0.01
    fprintf('\n✗ 错误：system_dynamics 的计算结果与手工计算不一致！\n');
    fprintf('需要检查 system_dynamics 内部的控制器调用。\n');
else
    fprintf('\n✓ 一致：两种计算方法结果相同。\n');
end

% 恢复轨迹
movefile(fullfile(project_dir, 'trajectory_backup.m'), fullfile(project_dir, 'trajectory.m'));
