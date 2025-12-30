% plot_trajectory_animation.m - 可视化无人机和载荷的运动轨迹
% 显示整个仿真过程中的位置变化
clear; clc; close all;

% 添加路径
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(genpath(project_dir));

% 加载最新结果
results_dir = fullfile(project_dir, 'results');
mat_files = dir(fullfile(results_dir, '*.mat'));
if isempty(mat_files)
    error('未找到仿真结果，请先运行 main.m');
end
[~, idx] = sort([mat_files.datenum], 'descend');
latest_file = fullfile(results_dir, mat_files(idx(1)).name);
fprintf('加载文件: %s\n\n', latest_file);
data = load(latest_file);

t = data.t;      % 时间数组
x = data.x;      % 状态数据
p = data.p;      % 参数
N = length(t);   % 总时间步数

fprintf('仿真时长: %.2f 秒\n', t(end));
fprintf('数据点数: %d\n', N);

%% 提取所有时刻的位置数据
% 预分配内存
pL_all = zeros(N, 3);           % 载荷位置 [N x 3]
p_uav_all = zeros(N, 3, p.n);   % 无人机位置 [N x 3 x 3]
pd_all = zeros(N, 3);           % 期望轨迹 [N x 3]

fprintf('正在提取轨迹数据...\n');
for k = 1:N
    % 提取第k个时刻的状态
    [pL, ~, ~, q, ~, ~, ~] = unpack_state(x(k,:)', p.n);
    pL_all(k,:) = pL';

    % 计算无人机位置
    for i = 1:p.n
        p_uav_all(k,:,i) = (pL - p.li * q(:,i))';
    end

    % 计算期望轨迹
    [pd, ~, ~, ~, ~] = trajectory(t(k));
    pd_all(k,:) = pd';
end
fprintf('数据提取完成!\n\n');

%% 图1: 带缆绳的多时刻快照
figure('Name', '轨迹快照（带缆绳）', 'Position', [50, 50, 1400, 600]);

% 选择要显示的时刻（均匀取5-8个时刻）
num_snapshots = 6;
snapshot_idx = round(linspace(1, N, num_snapshots));

% 子图1: 3D视图带缆绳
subplot(1,2,1);
hold on;

% 先画轨迹线（淡色背景）
plot3(pL_all(:,1), pL_all(:,2), pL_all(:,3), 'k-', 'LineWidth', 1, 'Color', [0.7 0.7 0.7]);
plot3(pd_all(:,1), pd_all(:,2), pd_all(:,3), 'r--', 'LineWidth', 1, 'Color', [1 0.7 0.7]);

% 画各无人机轨迹（淡色）
uav_colors_light = {[0.7 0.7 1], [0.7 1 0.7], [1 0.7 1]};
for i = 1:p.n
    plot3(p_uav_all(:,1,i), p_uav_all(:,2,i), p_uav_all(:,3,i), ...
          '-', 'LineWidth', 1, 'Color', uav_colors_light{i});
end

% 在选定的时刻画无人机、载荷和缆绳
uav_colors = {'b', 'g', 'm'};
for s = 1:num_snapshots
    k = snapshot_idx(s);
    alpha = 0.3 + 0.7 * (s / num_snapshots);  % 越新的时刻颜色越深

    pL_now = pL_all(k,:);

    % 画载荷
    plot3(pL_now(1), pL_now(2), pL_now(3), 'ko', 'MarkerSize', 10, ...
          'MarkerFaceColor', [0 0 0]*alpha + [1 1 1]*(1-alpha));

    % 画无人机和缆绳
    for i = 1:p.n
        p_uav_now = squeeze(p_uav_all(k,:,i));

        % 无人机
        plot3(p_uav_now(1), p_uav_now(2), p_uav_now(3), 'o', 'MarkerSize', 8, ...
              'MarkerFaceColor', uav_colors{i}, 'MarkerEdgeColor', uav_colors{i});

        % 缆绳（黑线）
        plot3([pL_now(1), p_uav_now(1)], [pL_now(2), p_uav_now(2)], ...
              [pL_now(3), p_uav_now(3)], 'k-', 'LineWidth', 1.5 * alpha);
    end

    % 画无人机之间的三角形
    for i = 1:p.n
        j = mod(i, p.n) + 1;
        p1 = squeeze(p_uav_all(k,:,i));
        p2 = squeeze(p_uav_all(k,:,j));
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              'b--', 'LineWidth', 0.5, 'Color', [0.5 0.5 1]*alpha);
    end
end

% 标注起点和终点
text(pL_all(1,1), pL_all(1,2), pL_all(1,3)-0.15, '起点', 'FontSize', 10, 'Color', 'g');
text(pL_all(end,1), pL_all(end,2), pL_all(end,3)-0.15, '终点', 'FontSize', 10, 'Color', 'r');

xlabel('X - 北 [m]');
ylabel('Y - 东 [m]');
zlabel('Z - 地 [m]');
title(sprintf('3D 轨迹快照 (%d个时刻)', num_snapshots));
grid on;
axis equal;
set(gca, 'ZDir', 'reverse');
view(45, 30);

% 子图2: 侧视图带缆绳 (XZ平面)
subplot(1,2,2);
hold on;

% 画轨迹线
plot(pL_all(:,1), pL_all(:,3), 'k-', 'LineWidth', 1, 'Color', [0.7 0.7 0.7]);
plot(pd_all(:,1), pd_all(:,3), 'r--', 'LineWidth', 1, 'Color', [1 0.7 0.7]);
for i = 1:p.n
    plot(p_uav_all(:,1,i), p_uav_all(:,3,i), '-', 'LineWidth', 1, 'Color', uav_colors_light{i});
end

% 画快照
for s = 1:num_snapshots
    k = snapshot_idx(s);
    alpha = 0.3 + 0.7 * (s / num_snapshots);

    pL_now = pL_all(k,:);

    % 载荷
    plot(pL_now(1), pL_now(3), 'ko', 'MarkerSize', 10, ...
         'MarkerFaceColor', [0 0 0]*alpha + [1 1 1]*(1-alpha));

    % 无人机和缆绳
    for i = 1:p.n
        p_uav_now = squeeze(p_uav_all(k,:,i));
        plot(p_uav_now(1), p_uav_now(3), 'o', 'MarkerSize', 8, ...
             'MarkerFaceColor', uav_colors{i});
        plot([pL_now(1), p_uav_now(1)], [pL_now(3), p_uav_now(3)], ...
             'k-', 'LineWidth', 1.5 * alpha);
    end
end

xlabel('X - 北 [m]');
ylabel('Z - 地 [m]');
title('侧视图 (XZ平面) - 显示缆绳');
grid on;
axis equal;
set(gca, 'YDir', 'reverse');

%% 图2: 初始和最终状态对比
figure('Name', '初始 vs 最终状态', 'Position', [100, 100, 1000, 500]);

% 左图：初始状态
subplot(1,2,1);
hold on;
k = 1;  % 初始时刻
pL_now = pL_all(k,:);

% 载荷
plot3(pL_now(1), pL_now(2), pL_now(3), 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'k');
text(pL_now(1), pL_now(2), pL_now(3)+0.1, '载荷', 'HorizontalAlignment', 'center');

% 无人机和缆绳
for i = 1:p.n
    p_uav_now = squeeze(p_uav_all(k,:,i));
    plot3(p_uav_now(1), p_uav_now(2), p_uav_now(3), 'o', 'MarkerSize', 12, ...
          'MarkerFaceColor', uav_colors{i}, 'MarkerEdgeColor', uav_colors{i});
    plot3([pL_now(1), p_uav_now(1)], [pL_now(2), p_uav_now(2)], ...
          [pL_now(3), p_uav_now(3)], 'k-', 'LineWidth', 2);
    text(p_uav_now(1), p_uav_now(2), p_uav_now(3)-0.1, sprintf('UAV_%d', i), ...
         'HorizontalAlignment', 'center');
end

% 三角形
uav_x = [squeeze(p_uav_all(k,1,:))', p_uav_all(k,1,1)];
uav_y = [squeeze(p_uav_all(k,2,:))', p_uav_all(k,2,1)];
uav_z = [squeeze(p_uav_all(k,3,:))', p_uav_all(k,3,1)];
plot3(uav_x, uav_y, uav_z, 'b--', 'LineWidth', 1);

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('初始状态 (t = %.2f s)', t(k)));
grid on; axis equal;
set(gca, 'ZDir', 'reverse');
view(45, 30);

% 右图：最终状态
subplot(1,2,2);
hold on;
k = N;  % 最终时刻
pL_now = pL_all(k,:);

% 载荷
plot3(pL_now(1), pL_now(2), pL_now(3), 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'k');
text(pL_now(1), pL_now(2), pL_now(3)+0.1, '载荷', 'HorizontalAlignment', 'center');

% 无人机和缆绳
for i = 1:p.n
    p_uav_now = squeeze(p_uav_all(k,:,i));
    plot3(p_uav_now(1), p_uav_now(2), p_uav_now(3), 'o', 'MarkerSize', 12, ...
          'MarkerFaceColor', uav_colors{i}, 'MarkerEdgeColor', uav_colors{i});
    plot3([pL_now(1), p_uav_now(1)], [pL_now(2), p_uav_now(2)], ...
          [pL_now(3), p_uav_now(3)], 'k-', 'LineWidth', 2);
    text(p_uav_now(1), p_uav_now(2), p_uav_now(3)-0.1, sprintf('UAV_%d', i), ...
         'HorizontalAlignment', 'center');
end

% 三角形
uav_x = [squeeze(p_uav_all(k,1,:))', p_uav_all(k,1,1)];
uav_y = [squeeze(p_uav_all(k,2,:))', p_uav_all(k,2,1)];
uav_z = [squeeze(p_uav_all(k,3,:))', p_uav_all(k,3,1)];
plot3(uav_x, uav_y, uav_z, 'b--', 'LineWidth', 1);

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('最终状态 (t = %.2f s)', t(k)));
grid on; axis equal;
set(gca, 'ZDir', 'reverse');
view(45, 30);

%% 图2: 位置随时间变化
figure('Name', '位置-时间曲线', 'Position', [50, 100, 1200, 400]);

subplot(1,3,1);
plot(t, pL_all(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_all(:,1), 'r--', 'LineWidth', 1);
xlabel('时间 [s]');
ylabel('X 位置 [m]');
title('X (北) 方向');
legend('实际', '期望');
grid on;

subplot(1,3,2);
plot(t, pL_all(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_all(:,2), 'r--', 'LineWidth', 1);
xlabel('时间 [s]');
ylabel('Y 位置 [m]');
title('Y (东) 方向');
legend('实际', '期望');
grid on;

subplot(1,3,3);
plot(t, pL_all(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd_all(:,3), 'r--', 'LineWidth', 1);
xlabel('时间 [s]');
ylabel('Z 位置 [m]');
title('Z (地) 方向');
legend('实际', '期望');
grid on;

%% 图3: 跟踪误差
figure('Name', '跟踪误差', 'Position', [50, 150, 800, 400]);

error_all = pL_all - pd_all;  % 误差 = 实际 - 期望
error_norm = sqrt(sum(error_all.^2, 2));  % 误差的模

subplot(1,2,1);
plot(t, error_all(:,1), 'r-', 'LineWidth', 1); hold on;
plot(t, error_all(:,2), 'g-', 'LineWidth', 1);
plot(t, error_all(:,3), 'b-', 'LineWidth', 1);
xlabel('时间 [s]');
ylabel('误差 [m]');
title('各方向跟踪误差');
legend('e_X', 'e_Y', 'e_Z');
grid on;

subplot(1,2,2);
plot(t, error_norm, 'k-', 'LineWidth', 1.5);
xlabel('时间 [s]');
ylabel('误差 [m]');
title('跟踪误差的模 ||e||');
grid on;

fprintf('最大跟踪误差: %.4f m\n', max(error_norm));
fprintf('最终跟踪误差: %.4f m\n', error_norm(end));

%% 图4: 动画（可选）
fprintf('\n是否播放动画? (需要等待)\n');
fprintf('按任意键开始动画，或按 Ctrl+C 跳过...\n');
pause;

figure('Name', '运动动画', 'Position', [100, 100, 800, 600]);

% 每隔几帧显示一次（加速）
skip = max(1, floor(N/100));  % 最多显示100帧

for k = 1:skip:N
    clf;
    hold on;

    % 提取当前时刻数据
    pL_now = pL_all(k,:)';
    pd_now = pd_all(k,:)';

    % 画历史轨迹（淡色）
    plot3(pL_all(1:k,1), pL_all(1:k,2), pL_all(1:k,3), 'k-', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
    plot3(pd_all(1:k,1), pd_all(1:k,2), pd_all(1:k,3), 'r--', 'LineWidth', 1, 'Color', [1 0.5 0.5]);

    % 画载荷当前位置
    plot3(pL_now(1), pL_now(2), pL_now(3), 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'k');

    % 画无人机当前位置和缆绳
    uav_colors = {'r', 'g', 'b'};
    for i = 1:p.n
        p_uav_now = p_uav_all(k,:,i)';

        % 无人机
        plot3(p_uav_now(1), p_uav_now(2), p_uav_now(3), 'o', ...
              'MarkerSize', 12, 'MarkerFaceColor', uav_colors{i});

        % 缆绳
        plot3([pL_now(1), p_uav_now(1)], [pL_now(2), p_uav_now(2)], ...
              [pL_now(3), p_uav_now(3)], 'k-', 'LineWidth', 2);
    end

    % 画无人机之间的三角形
    for i = 1:p.n
        j = mod(i, p.n) + 1;
        p1 = p_uav_all(k,:,i)';
        p2 = p_uav_all(k,:,j)';
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'b--', 'LineWidth', 1);
    end

    % 设置坐标轴
    xlabel('X - 北 [m]');
    ylabel('Y - 东 [m]');
    zlabel('Z - 地 [m]');
    title(sprintf('t = %.3f s', t(k)));

    % 固定坐标轴范围
    x_range = [min(pL_all(:,1))-0.5, max(pL_all(:,1))+0.5];
    y_range = [min(pL_all(:,2))-0.5, max(pL_all(:,2))+0.5];
    z_range = [min(pL_all(:,3))-0.5, max(pL_all(:,3))+0.5];
    xlim(x_range);
    ylim(y_range);
    zlim(z_range);

    grid on;
    axis equal;
    set(gca, 'ZDir', 'reverse');
    view(45, 30);

    drawnow;
    pause(0.02);  % 控制动画速度
end

fprintf('\n动画播放完成!\n');
