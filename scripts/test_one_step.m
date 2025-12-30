% TEST_ONE_STEP - 测试单步积分
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

% 初始状态
x0 = init_state(p);

% 计算 t=0 时的导数
t0 = 0;
dx0 = system_dynamics(t0, x0, p);

[pL0, vL0, ~, ~, omega0, ~, ~] = unpack_state(x0, p.n);
[dpL0, dvL0, ~, ~, domega0, ~, ~] = unpack_state(dx0, p.n);

fprintf('=== t = 0 时刻 ===\n');
fprintf('载荷位置: pL = [%.6f, %.6f, %.6f] m\n', pL0);
fprintf('载荷速度: vL = [%.6f, %.6f, %.6f] m/s\n', vL0);
fprintf('载荷加速度: dvL = [%.6f, %.6f, %.6f] m/s²\n', dvL0);
fprintf('缆绳角速度模: ||ω|| = [%.6f, %.6f, %.6f] rad/s\n', ...
    norm(omega0(:,1)), norm(omega0(:,2)), norm(omega0(:,3)));
fprintf('\n');

% 手动前向Euler积分一小步
dt = 0.001;  % 1ms
x1 = x0 + dx0 * dt;

[pL1, vL1, ~, ~, omega1, ~, ~] = unpack_state(x1, p.n);

fprintf('=== t = %.3f 时刻（手动Euler）===\n', dt);
fprintf('载荷位置: pL = [%.6f, %.6f, %.6f] m\n', pL1);
fprintf('载荷速度: vL = [%.6f, %.6f, %.6f] m/s\n', vL1);
fprintf('位置变化: Δz = %.6f m\n', pL1(3) - pL0(3));
fprintf('速度变化: Δvz = %.6f m/s\n', vL1(3) - vL0(3));
fprintf('\n');

% 用ode15s积分一小步
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-7, 'MaxStep', dt);
[t_ode, x_ode] = ode15s(@(t,x) system_dynamics(t,x,p), [0, dt], x0, options);

[pL_ode, vL_ode, ~, ~, ~, ~, ~] = unpack_state(x_ode(end,:)', p.n);

fprintf('=== t = %.3f 时刻（ode15s）===\n', dt);
fprintf('载荷位置: pL = [%.6f, %.6f, %.6f] m\n', pL_ode);
fprintf('载荷速度: vL = [%.6f, %.6f, %.6f] m/s\n', vL_ode);
fprintf('位置变化: Δz = %.6f m\n', pL_ode(3) - pL0(3));
fprintf('速度变化: Δvz = %.6f m/s\n', vL_ode(3) - vL0(3));
fprintf('\n');

if vL_ode(3) > 0.001
    fprintf('⚠ 警告：系统开始向下加速（NED中vz>0）\n');
elseif vL_ode(3) < -0.001
    fprintf('⚠ 警告：系统开始向上加速（NED中vz<0）\n');
else
    fprintf('✓ 速度变化很小，系统稳定\n');
end

% 恢复轨迹
movefile(fullfile(project_dir, 'trajectory_backup.m'), fullfile(project_dir, 'trajectory.m'));
