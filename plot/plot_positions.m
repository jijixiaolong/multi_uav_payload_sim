function plot_positions(t, x, params)
% PLOT_POSITIONS 绘制无人机和载荷的位置曲线
%   分别显示 x, y, z 三个方向的位置随时间变化
%
%   输入:
%   t: 时间向量
%   x: 状态矩阵
%   params: 参数结构体

n = params.n;
len = length(t);

% 预分配
pL = zeros(3, len);      % 载荷位置
p_uav = zeros(3, n, len); % 各无人机位置
pd = zeros(3, len);       % 期望位置

% 解包数据
for k = 1:len
    [pL_k, ~, ~, q_k, ~, ~, ~] = unpack_state(x(k,:)', n);
    pL(:,k) = pL_k;

    % 计算无人机位置: p_i = pL - li * q_i
    for i = 1:n
        p_uav(:,i,k) = pL_k - params.li * q_k(:,i);
    end

    [pd_k, ~, ~, ~, ~] = trajectory(t(k));
    pd(:,k) = pd_k;
end

% 颜色设置
colors_uav = {'r', 'g', 'b'};
labels_uav = {'UAV 1', 'UAV 2', 'UAV 3'};

%% 图1: 载荷位置 vs 期望位置
figure('Name', 'Payload Position', 'Position', [100, 100, 800, 600]);

subplot(3,1,1);
plot(t, pL(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd(1,:), 'k--', 'LineWidth', 1.5);
ylabel('X (m)');
title('Payload Position vs Desired');
legend('Actual', 'Desired', 'Location', 'best');
grid on;

subplot(3,1,2);
plot(t, pL(2,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd(2,:), 'k--', 'LineWidth', 1.5);
ylabel('Y (m)');
grid on;

subplot(3,1,3);
plot(t, pL(3,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pd(3,:), 'k--', 'LineWidth', 1.5);
ylabel('Z (m)');
xlabel('Time (s)');
grid on;

%% 图2: 各无人机 X 位置
figure('Name', 'UAV X Positions', 'Position', [150, 100, 800, 400]);

subplot(2,1,1);
hold on;
for i = 1:n
    plot(t, squeeze(p_uav(1,i,:)), colors_uav{i}, 'LineWidth', 1.5);
end
plot(t, pL(1,:), 'k--', 'LineWidth', 1.5);
ylabel('X (m)');
title('UAV X Positions');
legend([labels_uav, {'Payload'}], 'Location', 'best');
grid on;

subplot(2,1,2);
hold on;
for i = 1:n
    plot(t, squeeze(p_uav(1,i,:)) - pL(1,:), colors_uav{i}, 'LineWidth', 1.5);
end
ylabel('\Delta X (m)');
xlabel('Time (s)');
title('UAV X relative to Payload');
legend(labels_uav, 'Location', 'best');
grid on;

%% 图3: 各无人机 Y 位置
figure('Name', 'UAV Y Positions', 'Position', [200, 100, 800, 400]);

subplot(2,1,1);
hold on;
for i = 1:n
    plot(t, squeeze(p_uav(2,i,:)), colors_uav{i}, 'LineWidth', 1.5);
end
plot(t, pL(2,:), 'k--', 'LineWidth', 1.5);
ylabel('Y (m)');
title('UAV Y Positions');
legend([labels_uav, {'Payload'}], 'Location', 'best');
grid on;

subplot(2,1,2);
hold on;
for i = 1:n
    plot(t, squeeze(p_uav(2,i,:)) - pL(2,:), colors_uav{i}, 'LineWidth', 1.5);
end
ylabel('\Delta Y (m)');
xlabel('Time (s)');
title('UAV Y relative to Payload');
legend(labels_uav, 'Location', 'best');
grid on;

%% 图4: 各无人机 Z 位置（高度）
figure('Name', 'UAV Z Positions (Altitude)', 'Position', [250, 100, 800, 400]);

subplot(2,1,1);
hold on;
for i = 1:n
    plot(t, squeeze(p_uav(3,i,:)), colors_uav{i}, 'LineWidth', 1.5);
end
plot(t, pL(3,:), 'k--', 'LineWidth', 1.5);
ylabel('Z (m) [NED: down positive]');
title('UAV Z Positions');
legend([labels_uav, {'Payload'}], 'Location', 'best');
grid on;

subplot(2,1,2);
hold on;
for i = 1:n
    plot(t, squeeze(p_uav(3,i,:)) - pL(3,:), colors_uav{i}, 'LineWidth', 1.5);
end
ylabel('\Delta Z (m)');
xlabel('Time (s)');
title('UAV Z relative to Payload (should be negative)');
legend(labels_uav, 'Location', 'best');
grid on;

%% 图5: 综合位置图（所有方向在一个图中）
figure('Name', 'All Positions', 'Position', [300, 100, 1000, 700]);

% X 方向
subplot(3,1,1);
hold on;
plot(t, pL(1,:), 'k-', 'LineWidth', 2);
for i = 1:n
    plot(t, squeeze(p_uav(1,i,:)), colors_uav{i}, 'LineWidth', 1);
end
plot(t, pd(1,:), 'k--', 'LineWidth', 1);
ylabel('X (m)');
title('Position X');
legend(['Payload', labels_uav, {'Desired'}], 'Location', 'best');
grid on;

% Y 方向
subplot(3,1,2);
hold on;
plot(t, pL(2,:), 'k-', 'LineWidth', 2);
for i = 1:n
    plot(t, squeeze(p_uav(2,i,:)), colors_uav{i}, 'LineWidth', 1);
end
plot(t, pd(2,:), 'k--', 'LineWidth', 1);
ylabel('Y (m)');
title('Position Y');
grid on;

% Z 方向
subplot(3,1,3);
hold on;
plot(t, pL(3,:), 'k-', 'LineWidth', 2);
for i = 1:n
    plot(t, squeeze(p_uav(3,i,:)), colors_uav{i}, 'LineWidth', 1);
end
plot(t, pd(3,:), 'k--', 'LineWidth', 1);
ylabel('Z (m)');
xlabel('Time (s)');
title('Position Z (NED: down positive)');
grid on;

end
