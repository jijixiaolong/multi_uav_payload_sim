% 多无人机协同吊挂载荷运输仿真
% 主程序入口
clear; clc; close all;

%% 初始化设置
addpath_project();
p = params();
x0 = init_state(p);

%% 仿真配置
sim_mode = 'accurate';  % 'debug' (快速调试) | 'fast' (中速) | 'accurate' (高精度)
plot_results = true;    % 是否绘图
save_results = true;    % 是否保存结果
save_format = 'mat';    % 保存格式: 'mat' | 'csv' | 'both'

%% 运行仿真
[solver_opts, tspan] = get_solver_config(sim_mode);
tic;
[t, x] = solver_opts.solver(@(t, x) system_dynamics(t, x, p), tspan, x0, solver_opts.options);
elapsed = toc;

fprintf('仿真完成，耗时 %.2f 秒 (%s 模式)。\n', elapsed, sim_mode);

%% 绘制结果
if plot_results
    plot_all_in_one(t, x, p);
end

%% 保存结果
if save_results
    save_simulation_results(t, x, p, sim_mode, elapsed, save_format);
end

%% 辅助函数
function addpath_project()
    script_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(script_dir);
    addpath(genpath(fullfile(project_dir, 'dynamics')));
    addpath(genpath(fullfile(project_dir, 'control')));
    addpath(genpath(fullfile(project_dir, 'math')));
    addpath(genpath(fullfile(project_dir, 'utils')));
    addpath(genpath(fullfile(project_dir, 'plot')));
    addpath(project_dir);
end

function [config, tspan] = get_solver_config(sim_mode)
    switch sim_mode
        case 'debug'
            config.solver = @ode15s;
            tspan = [0 1];
            config.options = odeset('RelTol', 1e-3, 'AbsTol', 1e-4, 'MaxStep', 0.01);
        case 'fast'
            config.solver = @ode15s;
            tspan = [0 2];
            config.options = odeset('RelTol', 1e-5, 'AbsTol', 1e-6, 'MaxStep', 0.005);
        otherwise  % accurate
            config.solver = @ode45;
            tspan = [0 20];
            config.options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6, 'MaxStep', 0.005);
    end
end

function save_simulation_results(t, x, p, sim_mode, elapsed, save_format)
    % 保存仿真结果
    script_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(script_dir);
    results_dir = fullfile(project_dir, 'results');
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end

    % 提取轨迹数据
    N = length(t);
    [pL_hist, vL_hist, pd_hist] = extract_trajectory_data(t, x, p.n);

    % 生成文件名
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    base_name = sprintf('sim_%s_%s', sim_mode, timestamp);

    % 保存 MAT 文件
    if contains(save_format, 'mat')
        mat_file = fullfile(results_dir, [base_name '.mat']);
        save(mat_file, 't', 'x', 'p', 'pL_hist', 'vL_hist', 'pd_hist', 'elapsed', 'sim_mode');
        fprintf('结果已保存至: %s\n', mat_file);
    end

    % 保存 CSV 文件
    if contains(save_format, 'csv')
        csv_file = fullfile(results_dir, [base_name '.csv']);
        T = table(t, pL_hist(:,1), pL_hist(:,2), pL_hist(:,3), ...
            pd_hist(:,1), pd_hist(:,2), pd_hist(:,3), ...
            vL_hist(:,1), vL_hist(:,2), vL_hist(:,3), ...
            'VariableNames', {'t', 'pL_x', 'pL_y', 'pL_z', 'pd_x', 'pd_y', 'pd_z', 'vL_x', 'vL_y', 'vL_z'});
        writetable(T, csv_file);
        fprintf('结果已保存至: %s\n', csv_file);
    end
end

function [pL_hist, vL_hist, pd_hist] = extract_trajectory_data(t, x, n)
    N = length(t);
    pL_hist = zeros(N, 3);
    vL_hist = zeros(N, 3);
    pd_hist = zeros(N, 3);

    for k = 1:N
        [pL_k, vL_k, ~, ~, ~, ~, ~] = unpack_state(x(k,:)', n);
        pL_hist(k,:) = pL_k';
        vL_hist(k,:) = vL_k';

        [pd_k, ~, ~, ~, ~] = trajectory(t(k));
        pd_hist(k,:) = pd_k';
    end
end
