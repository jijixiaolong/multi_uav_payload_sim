% CHECK_INIT_VELOCITY - 检查初始状态的速度是否正确
clear; clc;

script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(project_dir);

% 获取轨迹初始速度
[pd0, dpd0, d2pd0, ~, ~] = trajectory(0);
fprintf('=== 轨迹期望初始条件 ===\n');
fprintf('位置 pd0 = [%.4f, %.4f, %.4f] m\n', pd0);
fprintf('速度 dpd0 = [%.4f, %.4f, %.4f] m/s\n', dpd0);
fprintf('加速度 d2pd0 = [%.4f, %.4f, %.4f] m/s²\n\n', d2pd0);

% 获取初始化的状态
p = params();
x0 = init_state(p);
[pL0, vL0, R_all, q, omega, dL_hat, d_hat] = unpack_state(x0, p.n);

fprintf('=== 实际初始化状态 ===\n');
fprintf('位置 pL0 = [%.4f, %.4f, %.4f] m\n', pL0);
fprintf('速度 vL0 = [%.4f, %.4f, %.4f] m/s\n', vL0);

% 速度误差
v_error = vL0 - dpd0;
fprintf('\n=== 速度误差 ===\n');
fprintf('Δv = vL0 - dpd0 = [%.6f, %.6f, %.6f] m/s\n', v_error);
fprintf('误差模 = %.6f m/s\n', norm(v_error));

if norm(v_error) < 1e-6
    fprintf('\n✓ 速度初始化正确\n');
else
    fprintf('\n✗ 速度初始化错误！\n');
end

% 检查初始加速度
fprintf('\n=== 检查初始动力学 ===\n');
dx0 = system_dynamics(0, x0, p);
[~, dvL0, ~, ~, ~, ~, ~] = unpack_state(dx0, p.n);
fprintf('初始加速度 dvL0 = [%.6f, %.6f, %.6f] m/s²\n', dvL0);
fprintf('期望加速度 d2pd0 = [%.6f, %.6f, %.6f] m/s²\n', d2pd0);
fprintf('加速度误差 = [%.6f, %.6f, %.6f] m/s²\n', dvL0 - d2pd0);
fprintf('加速度误差模 = %.6f m/s²\n', norm(dvL0 - d2pd0));

quit;
