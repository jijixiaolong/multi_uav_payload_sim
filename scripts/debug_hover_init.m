% DEBUG_HOVER_INIT - 检查悬停初始化状态
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

% 加载参数
p = params();

% 初始化状态
x0 = init_state(p);

% 解包
[pL0, vL0, R_all, q_all, omega_all, dL_hat0, d_hat0] = unpack_state(x0, p.n);

fprintf('=== 悬停初始化状态 ===\n\n');
fprintf('载荷位置: [%.3f, %.3f, %.3f] m\n', pL0);
fprintf('载荷速度: [%.3f, %.3f, %.3f] m/s\n', vL0);
fprintf('速度模: %.4f m/s\n\n', norm(vL0));

fprintf('缆绳配置:\n');
for i = 1:p.n
    fprintf('  UAV %d: q = [%.3f, %.3f, %.3f], ω = [%.3f, %.3f, %.3f]\n', ...
        i, q_all(:,i), omega_all(:,i));
end

fprintf('\n姿态配置:\n');
for i = 1:p.n
    R_i = R_all(:,:,i);
    r_i = R_i(:,3);
    % 提取欧拉角 (roll, pitch, yaw)
    roll = atan2(R_i(3,2), R_i(3,3));
    pitch = asin(-R_i(3,1));
    yaw = atan2(R_i(2,1), R_i(1,1));
    fprintf('  UAV %d: r_i (推力方向) = [%.3f, %.3f, %.3f]\n', i, r_i);
    fprintf('         欧拉角 = [roll=%.1f°, pitch=%.1f°, yaw=%.1f°]\n', ...
        rad2deg(roll), rad2deg(pitch), rad2deg(yaw));
end

% 计算初始控制输出
[pd0, dpd0, d2pd0, d3pd0, ~] = trajectory(0);
M = p.mL * eye(3) + p.mi * (q_all * q_all');
[f_dL, e, ev] = payload_ctrl(p, pd0, dpd0, d2pd0, M, pL0, vL0, q_all, dL_hat0, d_hat0);
f_qdi = force_allocation(p, f_dL, q_all, omega_all);
[f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(p, f_dL, e, ev, dpd0, d2pd0, d3pd0, q_all, omega_all, d_hat0);
f_di = f_qdi + f_di_perp;
[Omega_cmd, T] = attitude_ctrl(p, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R_all, q_all, omega_all);

fprintf('\n=== 初始控制输出 ===\n');
fprintf('载荷误差: e = [%.4f, %.4f, %.4f]\n', e);
fprintf('载荷速度误差: ev = [%.4f, %.4f, %.4f]\n', ev);
fprintf('载荷期望力: f_dL = [%.3f, %.3f, %.3f] m/s²\n\n', f_dL);

fprintf('各UAV控制量:\n');
for i = 1:p.n
    fprintf('  UAV %d:\n', i);
    fprintf('    f_qdi  = [%6.3f, %6.3f, %6.3f] N\n', f_qdi(:,i));
    fprintf('    f_perp = [%6.3f, %6.3f, %6.3f] N\n', f_di_perp(:,i));
    fprintf('    f_di   = [%6.3f, %6.3f, %6.3f] N (模=%.3f)\n', f_di(:,i), norm(f_di(:,i)));
    fprintf('    T_d = %.3f N, T = %.3f N (比值=%.4f)\n', norm(f_di(:,i)), T(i), T(i)/norm(f_di(:,i)));
    fprintf('    Ω_cmd  = [%6.3f, %6.3f, %6.3f] rad/s\n\n', Omega_cmd(:,i));
end

% 验证力平衡
e3 = [0;0;1];
total_thrust = zeros(3,1);
for i = 1:p.n
    R_i = R_all(:,:,i);
    f_i = -T(i) * R_i * e3;
    total_thrust = total_thrust + f_i;
end
gravity_force = (p.mL + p.n * p.mi) * p.g * e3;
net_force = total_thrust + gravity_force;
net_accel = net_force / (p.mL + p.n * p.mi);

fprintf('=== 力平衡验证 ===\n');
fprintf('总推力: [%.3f, %.3f, %.3f] N\n', total_thrust);
fprintf('重力: [%.3f, %.3f, %.3f] N\n', gravity_force);
fprintf('净力: [%.3f, %.3f, %.3f] N\n', net_force);
fprintf('净加速度: [%.4f, %.4f, %.4f] m/s²\n', net_accel);

if norm(net_accel) < 0.01
    fprintf('\n✓ 初始力平衡！\n');
else
    fprintf('\n✗ 初始力不平衡！净加速度 = %.4f m/s²\n', norm(net_accel));
end

% 恢复轨迹
movefile(fullfile(project_dir, 'trajectory_backup.m'), fullfile(project_dir, 'trajectory.m'));
