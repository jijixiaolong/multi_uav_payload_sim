function plot_all_in_one(t, x, params)
% PLOT_ALL_IN_ONE 将所有仿真结果绘制在一个 figure 中
%   包含: 3D轨迹(带缆绳)、位置曲线、跟踪误差等
%
%   输入:
%   t: 时间向量
%   x: 状态矩阵
%   params: 参数结构体

n = params.n;
N = length(t);

% 预分配
pL_all = zeros(N, 3);           % 载荷位置
p_uav_all = zeros(N, 3, n);     % 无人机位置
pd_all = zeros(N, 3);           % 期望轨迹

% 解包数据
for k = 1:N
    [pL_k, ~, ~, q_k, ~, ~, ~] = unpack_state(x(k,:)', n);
    pL_all(k,:) = pL_k';

    for i = 1:n
        p_uav_all(k,:,i) = (pL_k - params.li * q_k(:,i))';
    end

    [pd_k, ~, ~, ~, ~] = trajectory(t(k));
    pd_all(k,:) = pd_k';
end

% 颜色设置
uav_colors = {'b', 'g', 'm'};
uav_colors_light = {[0.7 0.7 1], [0.7 1 0.7], [1 0.7 1]};

%% 创建大图
figure('Name', '仿真结果总览', 'Position', [50, 50, 1400, 900]);

%% 子图1: 3D轨迹带缆绳 (左上)
subplot(2,3,1);
hold on;

% 画轨迹线（淡色）
plot3(pL_all(:,1), pL_all(:,2), pL_all(:,3), 'k-', 'LineWidth', 1.5);
plot3(pd_all(:,1), pd_all(:,2), pd_all(:,3), 'r--', 'LineWidth', 1);

% 选择要显示缆绳的时刻
num_snapshots = 5;
snapshot_idx = round(linspace(1, N, num_snapshots));

% 画各时刻的无人机、载荷和缆绳
for s = 1:num_snapshots
    k = snapshot_idx(s);
    alpha = 0.3 + 0.7 * (s / num_snapshots);

    pL_now = pL_all(k,:);

    % 载荷
    plot3(pL_now(1), pL_now(2), pL_now(3), 'ko', 'MarkerSize', 8, ...
          'MarkerFaceColor', [0 0 0]*alpha + [1 1 1]*(1-alpha));

    % 无人机和缆绳
    for i = 1:n
        p_uav_now = squeeze(p_uav_all(k,:,i));

        % 无人机
        plot3(p_uav_now(1), p_uav_now(2), p_uav_now(3), 'o', 'MarkerSize', 6, ...
              'MarkerFaceColor', uav_colors{i}, 'MarkerEdgeColor', uav_colors{i});

        % 缆绳
        plot3([pL_now(1), p_uav_now(1)], [pL_now(2), p_uav_now(2)], ...
              [pL_now(3), p_uav_now(3)], 'k-', 'LineWidth', 1.2 * alpha);
    end

    % 三角形
    for i = 1:n
        j = mod(i, n) + 1;
        p1 = squeeze(p_uav_all(k,:,i));
        p2 = squeeze(p_uav_all(k,:,j));
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              '--', 'LineWidth', 0.5, 'Color', [0.5 0.5 1]*alpha);
    end
end

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D 轨迹 (带缆绳)');
grid on; axis equal;
set(gca, 'ZDir', 'reverse');
view(45, 30);
legend('载荷轨迹', '期望轨迹', 'Location', 'best');

%% 子图2: 俯视图 (中上)
subplot(2,3,2);
hold on;

% 轨迹
plot(pL_all(:,1), pL_all(:,2), 'k-', 'LineWidth', 1.5);
plot(pd_all(:,1), pd_all(:,2), 'r--', 'LineWidth', 1);

% 无人机轨迹
for i = 1:n
    plot(p_uav_all(:,1,i), p_uav_all(:,2,i), '-', 'LineWidth', 1, 'Color', uav_colors_light{i});
end

% 起点和终点标记
plot(pL_all(1,1), pL_all(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(pL_all(end,1), pL_all(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

xlabel('X [m]'); ylabel('Y [m]');
title('俯视图 (XY平面)');
grid on; axis equal;
legend('实际', '期望', 'Location', 'best');

%% 子图3: 侧视图带缆绳 (右上)
subplot(2,3,3);
hold on;

% 轨迹
plot(pL_all(:,1), pL_all(:,3), 'k-', 'LineWidth', 1.5);
plot(pd_all(:,1), pd_all(:,3), 'r--', 'LineWidth', 1);

% 画快照
for s = 1:num_snapshots
    k = snapshot_idx(s);
    alpha = 0.3 + 0.7 * (s / num_snapshots);

    pL_now = pL_all(k,:);

    % 载荷
    plot(pL_now(1), pL_now(3), 'ko', 'MarkerSize', 8, ...
         'MarkerFaceColor', [0 0 0]*alpha + [1 1 1]*(1-alpha));

    % 无人机和缆绳
    for i = 1:n
        p_uav_now = squeeze(p_uav_all(k,:,i));
        plot(p_uav_now(1), p_uav_now(3), 'o', 'MarkerSize', 6, ...
             'MarkerFaceColor', uav_colors{i});
        plot([pL_now(1), p_uav_now(1)], [pL_now(3), p_uav_now(3)], ...
             'k-', 'LineWidth', 1.2 * alpha);
    end
end

xlabel('X [m]'); ylabel('Z [m]');
title('侧视图 (XZ平面)');
grid on; axis equal;
set(gca, 'YDir', 'reverse');

%% 子图4: X方向位置 (左下)
subplot(2,3,4);
hold on;
plot(t, pL_all(:,1), 'b-', 'LineWidth', 1.5);
plot(t, pd_all(:,1), 'r--', 'LineWidth', 1);
xlabel('时间 [s]'); ylabel('X [m]');
title('X 方向');
legend('实际', '期望', 'Location', 'best');
grid on;

%% 子图5: Y方向位置 (中下)
subplot(2,3,5);
hold on;
plot(t, pL_all(:,2), 'b-', 'LineWidth', 1.5);
plot(t, pd_all(:,2), 'r--', 'LineWidth', 1);
xlabel('时间 [s]'); ylabel('Y [m]');
title('Y 方向');
legend('实际', '期望', 'Location', 'best');
grid on;

%% 子图6: Z方向位置和跟踪误差 (右下)
subplot(2,3,6);
hold on;

% 计算误差
error_all = pL_all - pd_all;
error_norm = sqrt(sum(error_all.^2, 2));

yyaxis left;
plot(t, pL_all(:,3), 'b-', 'LineWidth', 1.5);
plot(t, pd_all(:,3), 'b--', 'LineWidth', 1);
ylabel('Z [m]');

yyaxis right;
plot(t, error_norm * 100, 'r-', 'LineWidth', 1.5);  % 转换为 cm
ylabel('误差 [cm]');

xlabel('时间 [s]');
title('Z 方向 & 跟踪误差');
legend('Z实际', 'Z期望', '误差', 'Location', 'best');
grid on;

%% 添加总标题
sgtitle(sprintf('多无人机载荷运输仿真 (t = 0 ~ %.2f s, %d 架UAV)', t(end), n));

%% 打印统计信息
fprintf('\n========== 仿真结果统计 ==========\n');
fprintf('仿真时长: %.2f s\n', t(end));
fprintf('数据点数: %d\n', N);
fprintf('最大跟踪误差: %.4f m (%.2f cm)\n', max(error_norm), max(error_norm)*100);
fprintf('最终跟踪误差: %.4f m (%.2f cm)\n', error_norm(end), error_norm(end)*100);
fprintf('平均跟踪误差: %.4f m (%.2f cm)\n', mean(error_norm), mean(error_norm)*100);
fprintf('===================================\n');

end
