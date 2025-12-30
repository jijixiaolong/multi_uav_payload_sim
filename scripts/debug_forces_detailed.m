% DEBUG_FORCES_DETAILED - 详细调试所有力的计算
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

% 参数
p = params();
[pd, dpd, d2pd, ~, ~] = trajectory(0);

% 初始化
x0 = init_state(p);
[pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x0, p.n);

% 计算控制量
M = p.mL * eye(3) + p.mi * (q * q');
[f_dL, e, ev] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);

fprintf('=== Step 1: Payload Control ===\n');
fprintf('f_dL = [%.4f, %.4f, %.4f] m/s²\n', f_dL);
fprintf('分解: f_dL(z) = %.4f = σ(e) + k2·σ(...) + g - d2pd\n', f_dL(3));
fprintf('      其中 g = %.4f\n\n', p.g);

% 力分配
f_qdi = force_allocation(p, f_dL, q, omega);

fprintf('=== Step 2: Force Allocation ===\n');
fprintf('目标: Σ(f_qdi) = -M·f_dL\n');
sum_f_qdi = sum(f_qdi, 2);
target = -M * f_dL;
fprintf('实际 Σ(f_qdi) = [%.4f, %.4f, %.4f] N\n', sum_f_qdi);
fprintf('目标 -M·f_dL  = [%.4f, %.4f, %.4f] N\n', target);
fprintf('误差 = %.6f N\n\n', norm(sum_f_qdi - target));

% 投影到缆绳方向
f_qdi_proj = zeros(1, p.n);
for i = 1:p.n
    f_qdi_proj(i) = q(:,i)' * f_qdi(:,i);
end
fprintf('各缆绳平行力投影 f_qdi·qi:\n');
for i = 1:p.n
    fprintf('  UAV %d: %.4f N\n', i, f_qdi_proj(i));
end
fprintf('\n');

% 缆绳配置控制
[f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, trajectory(0), q, omega, d_hat);

fprintf('=== Step 3: Cable Configuration Control ===\n');
for i = 1:p.n
    fprintf('  UAV %d: f_perp = [%.4f, %.4f, %.4f] N (模=%.4f)\n', i, f_di_perp(:,i), norm(f_di_perp(:,i)));
end
fprintf('\n');

% 总期望力
f_di = f_qdi + f_di_perp;
fprintf('=== Step 4: Total Desired Force ===\n');
for i = 1:p.n
    T_di = norm(f_di(:,i));
    fprintf('  UAV %d: f_di = [%.4f, %.4f, %.4f] N (模=%.4f)\n', i, f_di(:,i), T_di);
end
fprintf('\n');

% 姿态控制
[Omega_cmd, T] = attitude_ctrl(p, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R_all, q, omega);

fprintf('=== Step 5: Attitude Control (Eq. 29) ===\n');
e3 = [0;0;1];
for i = 1:p.n
    R_i = R_all(:,:,i);
    r_i = R_i(:,3);
    r_di = -f_di(:,i) / norm(f_di(:,i));
    dot_product = r_di' * r_i;
    T_di = norm(f_di(:,i));
    fprintf('  UAV %d:\n', i);
    fprintf('    T_di = %.4f N\n', T_di);
    fprintf('    r_di·r_i = %.6f\n', dot_product);
    fprintf('    T_i = T_di · (r_di·r_i) = %.4f N\n', T(i));
end
fprintf('\n');

% 实际推力向量
f_actual = zeros(3, p.n);
for i = 1:p.n
    f_actual(:,i) = -T(i) * R_all(:,:,i) * e3;
end

fprintf('=== Step 6: Actual Thrust Forces ===\n');
for i = 1:p.n
    fprintf('  UAV %d: f_i = [%.4f, %.4f, %.4f] N (模=%.4f)\n', i, f_actual(:,i), norm(f_actual(:,i)));
end
fprintf('\n');

% 推力在缆绳方向的投影
f_qi_actual = zeros(3, p.n);
for i = 1:p.n
    f_qi_actual(:,i) = (q(:,i)' * f_actual(:,i)) * q(:,i);
end

fprintf('=== Step 7: Actual Parallel Forces f_qi ===\n');
for i = 1:p.n
    fprintf('  UAV %d: f_qi = [%.4f, %.4f, %.4f] N\n', i, f_qi_actual(:,i));
end
sum_f_qi = sum(f_qi_actual, 2);
fprintf('Σ(f_qi) = [%.4f, %.4f, %.4f] N\n\n', sum_f_qi);

% 最终动力学验证 - 使用正确的 Eq. 8
% v̇_L = M^(-1)[Σ(f_qi + d_qi - mi·li·||ωi||²·qi) + d_L] + g·e3

% 计算 Σ(f_qi + d_qi - mi·li·||ωi||²·qi)
omega_norm_sq = sum(omega.^2, 1);  % 1×n
sum_terms = sum_f_qi;  % 已经计算过的 Σf_qi
% 注意：这里忽略扰动，因为 p.d_L_true 和 p.d_i_true 默认是零

% 按 Eq. 8 计算加速度
dvL_correct = M \ sum_terms + p.g * e3;

fprintf('=== Final Dynamics Verification (使用Eq. 8) ===\n');
fprintf('M矩阵:\n');
disp(M);
fprintf('Σ(f_qi) = [%.4f, %.4f, %.4f] N\n', sum_f_qi);
fprintf('M^(-1)·Σ(f_qi) = [%.6f, %.6f, %.6f] m/s²\n', M \ sum_f_qi);
fprintf('g·e3 = [%.6f, %.6f, %.6f] m/s²\n', p.g * e3);
fprintf('最终加速度 dvL = [%.6f, %.6f, %.6f] m/s²\n', dvL_correct);

if norm(dvL_correct) < 0.01
    fprintf('\n✓ 悬停力平衡！\n');
elseif dvL_correct(3) > 0.01
    fprintf('\n✗ 系统会向下加速！ (NED: Z+向下)\n');
elseif dvL_correct(3) < -0.01
    fprintf('\n✗ 系统会向上加速！\n');
else
    fprintf('\n△ 垂直方向平衡，但水平方向有误差\n');
end

% 恢复轨迹
movefile(fullfile(project_dir, 'trajectory_backup.m'), fullfile(project_dir, 'trajectory.m'));
