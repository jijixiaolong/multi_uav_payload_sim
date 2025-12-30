% DEBUG_FORCES - 调试力的平衡和方向
% 详细检查初始状态下的力平衡

clear; clc;

% 设置路径
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(project_dir);

% 加载参数和初始状态
params = params();
n = params.n;
x0 = init_state(params);
t = 0;

% 提取状态
[pL, vL, R, q, omega, dL_hat, d_hat] = unpack_state(x0, n);

% 期望轨迹
[pd, dpd, d2pd, d3pd, d4pd] = trajectory(t);

% 计算 M
M = params.mL * eye(3) + params.mi * (q * q') + 1e-8 * eye(3);

fprintf('=== 系统参数 ===\n');
fprintf('载荷质量 mL: %.3f kg\n', params.mL);
fprintf('无人机质量 mi: %.3f kg\n', params.mi);
fprintf('总质量: %.3f kg\n', params.mL + n * params.mi);
fprintf('重力加速度 g: %.3f m/s²\n', params.g);
fprintf('总重力: %.3f N\n', (params.mL + n * params.mi) * params.g);
fprintf('每机悬停推力: %.3f N\n', (params.mL + n * params.mi) * params.g / n);

fprintf('\n=== 初始状态 ===\n');
fprintf('载荷位置 pL: [%.3f, %.3f, %.3f] m\n', pL);
fprintf('载荷速度 vL: [%.3f, %.3f, %.3f] m/s\n', vL);
fprintf('期望位置 pd: [%.3f, %.3f, %.3f] m\n', pd);
fprintf('期望速度 dpd: [%.3f, %.3f, %.3f] m/s\n', dpd);

fprintf('\n=== 缆绳配置 ===\n');
for i = 1:n
    fprintf('UAV%d 缆绳方向 q: [%.4f, %.4f, %.4f], norm = %.4f\n', ...
        i, q(:,i), norm(q(:,i)));
end

% 计算控制量
[f_dL, e, ev] = payload_ctrl(params, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
f_qdi = force_allocation(params, f_dL, q, omega);
[f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = ...
    cable_ctrl(params, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
[Omega_cmd, T] = attitude_ctrl(params, f_qdi, f_di_perp, omega_di, ...
    dot_omega_di, e_qi, e_omega_i, ev, e, M, R, q, omega);

fprintf('\n=== 控制输出 ===\n');
fprintf('期望力 f_dL: [%.4f, %.4f, %.4f] N, norm = %.4f\n', f_dL, norm(f_dL));
fprintf('总推力: %.4f N\n', sum(T));

% 计算每个无人机的推力向量
e3 = [0;0;1];
f_thrust = zeros(3, n);
for i = 1:n
    r_i = R(:,:,i) * e3;  % 当前推力方向（机体z轴在世界坐标系中的方向）
    f_thrust(:,i) = -T(i) * r_i;  % 推力向量（向上为负z）
    fprintf('\nUAV%d:\n', i);
    fprintf('  推力大小 T: %.4f N\n', T(i));
    fprintf('  推力方向 r: [%.4f, %.4f, %.4f]\n', r_i);
    fprintf('  推力向量 f: [%.4f, %.4f, %.4f] N\n', f_thrust(:,i));
    fprintf('  f_qdi: [%.4f, %.4f, %.4f] N, norm = %.4f\n', f_qdi(:,i), norm(f_qdi(:,i)));
    fprintf('  f_di_perp: [%.4f, %.4f, %.4f] N, norm = %.4f\n', f_di_perp(:,i), norm(f_di_perp(:,i)));
    fprintf('  f_di (total): [%.4f, %.4f, %.4f] N, norm = %.4f\n', ...
        f_qdi(:,i) + f_di_perp(:,i), norm(f_qdi(:,i) + f_di_perp(:,i)));
end

% 合力
f_total = sum(f_thrust, 2);
fprintf('\n=== 力平衡 ===\n');
fprintf('合推力向量: [%.4f, %.4f, %.4f] N\n', f_total);
fprintf('总重力向量 (NED): [0, 0, %.4f] N\n', (params.mL + n * params.mi) * params.g);
fprintf('净力: [%.4f, %.4f, %.4f] N\n', f_total(1), f_total(2), f_total(3) + (params.mL + n * params.mi) * params.g);

% 根据动力学方程计算期望加速度
% dvL = M \ (sum_terms + d_L_true) + g * e3
% 这里 sum_terms 应该等于合推力在缆绳方向的投影

% 计算 sum_terms（来自 system_dynamics.m）
f_proj = sum(q .* f_thrust, 1);  % 推力在缆绳方向的投影
omega_norm_sq = sum(omega.^2, 1);
coeff = f_proj - params.mi * params.li * omega_norm_sq;
sum_terms = q * coeff';

fprintf('\n=== 动力学检查 ===\n');
fprintf('推力投影和 sum_terms: [%.4f, %.4f, %.4f] N\n', sum_terms);
expected_accel = M \ sum_terms + params.g * e3;
fprintf('期望加速度（根据动力学）: [%.4f, %.4f, %.4f] m/s²\n', expected_accel);

% 如果期望加速度的 z 分量是正的（向下），系统会下落
% 如果是负的（向上），系统会上升
if expected_accel(3) < -0.1
    fprintf('警告：期望加速度向上 (%.3f m/s²)，系统会上升！\n', expected_accel(3));
elseif expected_accel(3) > 0.1
    fprintf('警告：期望加速度向下 (%.3f m/s²)，系统会下落！\n', expected_accel(3));
else
    fprintf('OK：期望加速度接近零 (%.3f m/s²)\n', expected_accel(3));
end
