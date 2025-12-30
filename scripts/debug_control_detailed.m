% DEBUG_CONTROL_DETAILED - 详细调试控制输出，检查数值稳定性
% 检查控制器在初始状态和小扰动下的输出

clear; clc;

% 设置路径
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(project_dir);

% 加载参数
params = params();
n = params.n;

% 加载初始状态
x0 = init_state(params);
t = 0;

% 提取初始状态
[pL, vL, R, q, omega, dL_hat, d_hat] = unpack_state(x0, n);

% 获取期望轨迹
[pd, dpd, d2pd, d3pd, d4pd] = trajectory(t);

% 计算矩阵 M
M = params.mL * eye(3) + params.mi * (q * q') + 1e-8 * eye(3);

fprintf('=== 初始状态 (t=0) ===\n');
fprintf('载荷位置: [%.3f, %.3f, %.3f]\n', pL);
fprintf('载荷速度: [%.3f, %.3f, %.3f]\n', vL);
fprintf('期望位置: [%.3f, %.3f, %.3f]\n', pd);
fprintf('位置误差: [%.3f, %.3f, %.3f]\n', pL - pd);

% 调用各控制器
fprintf('\n=== 1. Payload Control ===\n');
[f_dL, e, ev] = payload_ctrl(params, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
fprintf('耦合误差 e: [%.4f, %.4f, %.4f], norm = %.4f\n', e, norm(e));
fprintf('速度误差 ev: [%.4f, %.4f, %.4f], norm = %.4f\n', ev, norm(ev));
fprintf('期望力 f_dL: [%.4f, %.4f, %.4f], norm = %.4f\n', f_dL, norm(f_dL));

fprintf('\n=== 2. Force Allocation ===\n');
f_qdi = force_allocation(params, f_dL, q, omega);
for i = 1:n
    fprintf('UAV%d f_qdi: [%.4f, %.4f, %.4f], norm = %.4f\n', i, f_qdi(:,i), norm(f_qdi(:,i)));
end

fprintf('\n=== 3. Cable Control ===\n');
[f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(params, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
for i = 1:n
    fprintf('UAV%d f_di_perp: [%.4f, %.4f, %.4f], norm = %.4f\n', i, f_di_perp(:,i), norm(f_di_perp(:,i)));
    fprintf('  e_qi: [%.4f, %.4f, %.4f], norm = %.4f\n', e_qi(:,i), norm(e_qi(:,i)));
    fprintf('  e_omega_i: [%.4f, %.4f, %.4f], norm = %.4f\n', e_omega_i(:,i), norm(e_omega_i(:,i)));
end

fprintf('\n=== 4. Attitude Control ===\n');
[Omega_cmd, T] = attitude_ctrl(params, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R, q, omega);
total_thrust = sum(T);
expected_hover = params.mL * params.g + params.n * params.mi * params.g;
fprintf('总推力: %.4f N (悬停预期: %.4f N)\n', total_thrust, expected_hover);
for i = 1:n
    fprintf('UAV%d: T = %.4f N, Omega_cmd = [%.4f, %.4f, %.4f] rad/s, norm = %.4f\n', ...
        i, T(i), Omega_cmd(:,i), norm(Omega_cmd(:,i)));
end

% 计算合力验证
e3 = [0;0;1];
f_total = zeros(3,1);
for i = 1:n
    f_i = -T(i) * R(:,:,i) * e3;
    f_total = f_total + f_i;
end
fprintf('\n合力 f_total: [%.4f, %.4f, %.4f] N\n', f_total);
fprintf('期望合力 (抵消重力): [0, 0, -%.4f] N\n', expected_hover);

% 检查是否有异常大的值
fprintf('\n=== 数值检查 ===\n');
max_thrust = max(T);
max_omega_cmd = max(vecnorm(Omega_cmd, 2, 1));
max_f_perp = max(vecnorm(f_di_perp, 2, 1));
max_f_qdi = max(vecnorm(f_qdi, 2, 1));

fprintf('最大推力: %.4f N\n', max_thrust);
fprintf('最大角速度指令: %.4f rad/s\n', max_omega_cmd);
fprintf('最大垂直力: %.4f N\n', max_f_perp);
fprintf('最大平行力: %.4f N\n', max_f_qdi);

% 推力合理性检查
if max_thrust > 10
    warning('推力过大！可能导致发散');
end
if max_omega_cmd > 10
    warning('角速度指令过大！可能导致发散');
end

%% 测试小扰动情况
fprintf('\n\n=== 小扰动测试 (位置偏移 0.1m) ===\n');
pL_pert = pL + [0.1; 0; 0];
vL_pert = vL;

[f_dL_pert, e_pert, ev_pert] = payload_ctrl(params, pd, dpd, d2pd, M, pL_pert, vL_pert, q, dL_hat, d_hat);
fprintf('扰动后 f_dL: [%.4f, %.4f, %.4f], norm = %.4f\n', f_dL_pert, norm(f_dL_pert));
fprintf('扰动后误差 e: [%.4f, %.4f, %.4f], norm = %.4f\n', e_pert, norm(e_pert));

f_qdi_pert = force_allocation(params, f_dL_pert, q, omega);
[f_di_perp_pert, omega_di_pert, dot_omega_di_pert, e_qi_pert, e_omega_i_pert] = ...
    cable_ctrl(params, f_dL_pert, e_pert, ev_pert, dpd, d2pd, d3pd, q, omega, d_hat);
[Omega_cmd_pert, T_pert] = attitude_ctrl(params, f_qdi_pert, f_di_perp_pert, omega_di_pert, ...
    dot_omega_di_pert, e_qi_pert, e_omega_i_pert, ev_pert, e_pert, M, R, q, omega);

fprintf('扰动后总推力: %.4f N\n', sum(T_pert));
fprintf('扰动后最大角速度指令: %.4f rad/s\n', max(vecnorm(Omega_cmd_pert, 2, 1)));
for i = 1:n
    fprintf('UAV%d: ΔT = %.4f N, ΔOmega = %.4f rad/s\n', ...
        i, T_pert(i) - T(i), norm(Omega_cmd_pert(:,i) - Omega_cmd(:,i)));
end

if max(T_pert) > 15
    warning('扰动后推力过大！');
end
if max(vecnorm(Omega_cmd_pert, 2, 1)) > 15
    warning('扰动后角速度指令过大！');
end
