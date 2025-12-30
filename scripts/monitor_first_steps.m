% MONITOR_FIRST_STEPS - 监控仿真前几个步骤的详细状态
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

% 初始化
x0 = init_state(p);

% 仿真：只模拟前0.1秒
tspan = [0 0.1];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-7, 'MaxStep', 0.001, ...
    'OutputFcn', @monitor_output);

fprintf('开始监控前0.1秒...\n\n');

global monitor_data;
monitor_data = struct('t', [], 'pL', [], 'vL', [], 'T', [], 'rdi_dot_r', []);

try
    [t, x] = ode15s(@(t, x) system_dynamics(t, x, p), tspan, x0, options);
    fprintf('\n仿真成功完成！\n');
catch e
    fprintf('\n仿真失败: %s\n', e.message);
end

% 分析
if ~isempty(monitor_data.t)
    N = length(monitor_data.t);
    fprintf('\n=== 收集了 %d 个时间点 ===\n', N);

    % 显示前10个和最后5个
    show_idx = min(10, N);
    fprintf('\n前%d个时间点:\n', show_idx);
    for k = 1:show_idx
        fprintf('t=%.4f: pL_z=%.4f, vL_z=%.4f, T_avg=%.2f, min(rdi·ri)=%.4f\n', ...
            monitor_data.t(k), monitor_data.pL(k,3), monitor_data.vL(k,3), ...
            mean(monitor_data.T(k,:)), min(monitor_data.rdi_dot_r(k,:)));
    end

    if N > 10
        fprintf('\n最后5个时间点:\n');
        for k = max(1, N-4):N
            fprintf('t=%.4f: pL_z=%.4f, vL_z=%.4f, T_avg=%.2f, min(rdi·ri)=%.4f\n', ...
                monitor_data.t(k), monitor_data.pL(k,3), monitor_data.vL(k,3), ...
                mean(monitor_data.T(k,:)), min(monitor_data.rdi_dot_r(k,:)));
        end
    end

    % 检查是否有负推力或点积过小
    min_T = min(monitor_data.T(:));
    min_dot = min(monitor_data.rdi_dot_r(:));
    fprintf('\n最小推力: %.3f N\n', min_T);
    fprintf('最小点积 rdi·ri: %.4f\n', min_dot);

    if min_T < 0
        fprintf('⚠ 警告：出现负推力！\n');
    end
    if min_dot < 0.5
        fprintf('⚠ 警告：姿态偏差过大（点积<0.5）！\n');
    end
end

% 恢复轨迹
movefile(fullfile(project_dir, 'trajectory_backup.m'), fullfile(project_dir, 'trajectory.m'));

% OutputFcn 回调
function status = monitor_output(t, x, flag)
    global monitor_data;
    status = 0;

    if isempty(flag)
        % 在每个输出点调用
        script_dir = fileparts(mfilename('fullpath'));
        project_dir = fileparts(script_dir);
        addpath(genpath(fullfile(project_dir, 'dynamics')));
        addpath(genpath(fullfile(project_dir, 'control')));
        addpath(genpath(fullfile(project_dir, 'math')));
        addpath(genpath(fullfile(project_dir, 'utils')));
        addpath(project_dir);

        p = params();

        for k = 1:length(t)
            [pL, vL, R_all, q, omega, dL_hat, d_hat] = unpack_state(x(:,k), p.n);

            % 计算控制
            [pd, dpd, d2pd, d3pd, ~] = trajectory(t(k));
            M = p.mL * eye(3) + p.mi * (q * q');
            [f_dL, e, ev] = payload_ctrl(p, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
            f_qdi = force_allocation(p, f_dL, q, omega);
            [f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(p, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
            [~, T] = attitude_ctrl(p, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R_all, q, omega);

            % 计算 rdi·ri
            f_di = f_qdi + f_di_perp;
            T_d = sqrt(sum(f_di.^2, 1));
            T_d_safe = T_d;
            T_d_safe(T_d < 1e-6) = 1;
            r_di = -f_di ./ T_d_safe;
            r = squeeze(R_all(:,3,:));
            rdi_dot_r = sum(r_di .* r, 1);

            % 记录
            monitor_data.t(end+1) = t(k);
            monitor_data.pL(end+1, :) = pL';
            monitor_data.vL(end+1, :) = vL';
            monitor_data.T(end+1, :) = T;
            monitor_data.rdi_dot_r(end+1, :) = rdi_dot_r;
        end
    end
end
