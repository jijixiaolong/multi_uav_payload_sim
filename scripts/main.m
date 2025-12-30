% Main Simulation Script
clear; clc; close all;

% Add paths
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);  % 上级目录为项目根目录
addpath(genpath(fullfile(project_dir, 'dynamics')));
addpath(genpath(fullfile(project_dir, 'control')));
addpath(genpath(fullfile(project_dir, 'math')));
addpath(genpath(fullfile(project_dir, 'utils')));
addpath(genpath(fullfile(project_dir, 'plot')));
addpath(project_dir);  % 添加根目录以访问 params.m 和 trajectory.m

% Load parameters
p = params();

% Initialize state
x0 = init_state(p);

% Simulation configuration
% 模式选择: 'debug' (最快), 'fast', 'accurate'
% 默认使用 accurate 以便长时间运行稳定性测试
sim_mode = 'accurate';
plot_results = true;
save_results = true;       % 是否保存结果
save_format = 'mat';       % 'mat' | 'csv' | 'both'

switch sim_mode
    case 'debug'
        % 调试模式：快速验证（矩形轨迹周期约9.7秒）
        solver = @ode15s;          % 刚性求解器，更稳定
        tspan = [0 1];             % 约半圈矩形
        rel_tol = 1e-3;
        abs_tol = 1e-4;
        max_step = 0.01;
    case 'fast'
        % 快速模式：优化设置以增加稳定性
        solver = @ode15s;          % 刚性求解器
        tspan = [0 2];
        rel_tol = 1e-5;            % 提高精度
        abs_tol = 1e-6;            % 提高精度
        max_step = 0.005;          % 减小最大步长
    otherwise
        % 精确模式：高精度，完整两圈
        solver = @ode15s;          % 仍用刚性求解器避免卡死
        tspan = [0 20];
        rel_tol = 1e-6;
        abs_tol = 1e-6;
        max_step = 0.005;
end

options = odeset('RelTol', rel_tol, 'AbsTol', abs_tol);
if ~isempty(max_step)
    options = odeset(options, 'MaxStep', max_step);
end

tic;
[t, x] = solver(@(t, x) system_dynamics(t, x, p), tspan, x0, options);
elapsed = toc;

% Plot results
if plot_results
    % % 主图：所有结果在一个 figure 中（带缆绳的3D轨迹）
    plot_all_in_one(t, x, p);

    % % 可选：详细图（取消注释以显示）
    % plot_positions(t, x, p);  % UAV和载荷位置曲线
    % plot_payload(t, x, p);
    % plot_cable(t, x, p);
    % plot_disturb(t, x, p);
    % plot_input(t, x, p);
end

fprintf('Simulation complete in %.2f s (%s mode).\n', elapsed, sim_mode);

% Save results
if save_results
    % 创建 results 目录
    results_dir = fullfile(project_dir, 'results');
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end

    % 生成带时间戳的文件名
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    base_name = sprintf('sim_%s_%s', sim_mode, timestamp);

    % 解包状态用于保存
    N = length(t);
    pL_hist = zeros(N, 3);
    vL_hist = zeros(N, 3);
    for k = 1:N
        [pL_k, vL_k, ~, ~, ~, ~, ~] = unpack_state(x(k,:)', p.n);
        pL_hist(k,:) = pL_k';
        vL_hist(k,:) = vL_k';
    end

    % 计算期望轨迹
    pd_hist = zeros(N, 3);
    for k = 1:N
        [pd_k, ~, ~, ~, ~] = trajectory(t(k));
        pd_hist(k,:) = pd_k';
    end

    % 保存为 .mat 格式
    if strcmp(save_format, 'mat') || strcmp(save_format, 'both')
        mat_file = fullfile(results_dir, [base_name '.mat']);
        save(mat_file, 't', 'x', 'p', 'pL_hist', 'vL_hist', 'pd_hist', 'elapsed', 'sim_mode');
        fprintf('Results saved to: %s\n', mat_file);
    end

    % 保存为 .csv 格式 (便于其他软件读取)
    if strcmp(save_format, 'csv') || strcmp(save_format, 'both')
        csv_file = fullfile(results_dir, [base_name '.csv']);
        % 创建表格: t, pL_x, pL_y, pL_z, pd_x, pd_y, pd_z, vL_x, vL_y, vL_z
        T = table(t, pL_hist(:,1), pL_hist(:,2), pL_hist(:,3), ...
            pd_hist(:,1), pd_hist(:,2), pd_hist(:,3), ...
            vL_hist(:,1), vL_hist(:,2), vL_hist(:,3), ...
            'VariableNames', {'t', 'pL_x', 'pL_y', 'pL_z', 'pd_x', 'pd_y', 'pd_z', 'vL_x', 'vL_y', 'vL_z'});
        writetable(T, csv_file);
        fprintf('Results saved to: %s\n', csv_file);
    end
end
