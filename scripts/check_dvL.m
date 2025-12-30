% CHECK_DVL - 检查初始加速度为什么不是零
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

fprintf('调用 system_dynamics(0, x0, p)...\n');
dx = system_dynamics(0, x0, p);

[dpL, dvL, dR, dq, domega, ~, ~] = unpack_state(dx, p.n);

fprintf('\n=== 状态导数 ===\n');
fprintf('dpL (应该≈0) = [%.6f, %.6f, %.6f]\n', dpL);
fprintf('dvL (应该≈0) = [%.6f, %.6f, %.6f]\n', dvL);
fprintf('||dq1|| = %.6f\n', norm(dq(:,1)));
fprintf('||domega1|| = %.6f\n', norm(domega(:,1)));

if norm(dvL) > 0.1
    fprintf('\n✗ 错误：初始加速度过大！应该接近零。\n');
    fprintf('这说明初始状态不是平衡点。\n');
end

% 恢复轨迹
movefile(fullfile(project_dir, 'trajectory_backup.m'), fullfile(project_dir, 'trajectory.m'));
