% debug_control.m - 验证控制器在初始状态的输出
clc; clear; close all;

% 添加路径
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(project_dir));

% 加载参数
p = params();

% 初始化状态
x0 = init_state(p);
[pL, vL, R, q, omega, dL_hat, d_hat] = unpack_state(x0, p.n);

% 期望轨迹
t = 0;
[pd, dpd, d2pd, d3pd, d4pd] = trajectory(t);

% 质量矩阵
M = p.mL * eye(3) + p.mi * (q * q') + 1e-8 * eye(3);

fprintf('=== 初始状态验证 ===\n\n');

% 1. 检查初始误差
fprintf('1. 初始误差:\n');
ep = pL - pd;
ev = vL - dpd;
fprintf('   位置误差 ep = [%.4f, %.4f, %.4f] m\n', ep);
fprintf('   速度误差 ev = [%.4f, %.4f, %.4f] m/s\n', ev);

% 2. 载荷位置控制输出
[f_dL, e, ev_out] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
fprintf('\n2. payload_ctrl 输出:\n');
fprintf('   f_dL = [%.4f, %.4f, %.4f]\n', f_dL);
fprintf('   ||f_dL|| = %.4f\n', norm(f_dL));

% 3. 力分配输出
f_qdi = force_allocation(p, f_dL, q, omega);
fprintf('\n3. force_allocation 输出:\n');
fprintf('   f_qdi(:,1) = [%.4f, %.4f, %.4f]\n', f_qdi(:,1));
fprintf('   f_qdi(:,2) = [%.4f, %.4f, %.4f]\n', f_qdi(:,2));
fprintf('   f_qdi(:,3) = [%.4f, %.4f, %.4f]\n', f_qdi(:,3));
fprintf('   sum(f_qdi) = [%.4f, %.4f, %.4f]\n', sum(f_qdi, 2));

% 4. 缆绳控制输出
[f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(p, f_dL, e, ev_out, dpd, d2pd, d3pd, q, omega, d_hat);
fprintf('\n4. cable_ctrl 输出:\n');
fprintf('   f_di_perp(:,1) = [%.4f, %.4f, %.4f]\n', f_di_perp(:,1));
fprintf('   ||f_di_perp(:,1)|| = %.4f\n', norm(f_di_perp(:,1)));

% 5. 姿态控制输出
[Omega_cmd, T] = attitude_ctrl(p, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev_out, e, M, R, q, omega);
fprintf('\n5. attitude_ctrl 输出:\n');
fprintf('   T = [%.4f, %.4f, %.4f] N\n', T);
fprintf('   sum(T) = %.4f N\n', sum(T));

% 6. 预期值
total_mass = p.mL + p.n * p.mi;
expected_thrust = total_mass * p.g;
fprintf('\n6. 预期值:\n');
fprintf('   系统总质量 = %.4f kg\n', total_mass);
fprintf('   预期总推力 = %.4f N (悬停)\n', expected_thrust);

% 7. 计算实际推力产生的力
fprintf('\n7. 推力产生的力:\n');
e3 = [0;0;1];
f_total = zeros(3,1);
for i = 1:p.n
    f_i = -T(i) * R(:,:,i) * e3;
    fprintf('   f_%d = [%.4f, %.4f, %.4f] N\n', i, f_i);
    f_total = f_total + f_i;
end
fprintf('   f_total = [%.4f, %.4f, %.4f] N\n', f_total);

% 8. 检查力平衡
fprintf('\n8. 力平衡检查:\n');
% 在缆绳方向的投影
f_proj_total = zeros(3,1);
for i = 1:p.n
    f_i = -T(i) * R(:,:,i) * e3;
    f_proj = (q(:,i)' * f_i) * q(:,i);
    f_proj_total = f_proj_total + f_proj;
end
fprintf('   缆绳方向总力 = [%.4f, %.4f, %.4f] N\n', f_proj_total);
fprintf('   需要的重力补偿 = [%.4f, %.4f, %.4f] N\n', -total_mass * p.g * e3);
