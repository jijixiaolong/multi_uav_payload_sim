function plot_payload(t, x, params)
% PLOT_PAYLOAD Plot payload position and velocity errors

n = params.n;
len = length(t);
pL = zeros(3, len);
vL = zeros(3, len);
pd = zeros(3, len);
dpd = zeros(3, len);

for k = 1:len
    [pL_k, vL_k, ~, ~, ~, ~, ~] = unpack_state(x(k,:)', n);
    pL(:,k) = pL_k;
    vL(:,k) = vL_k;

    [pd_k, dpd_k, ~, ~, ~] = trajectory(t(k));
    pd(:,k) = pd_k;
    dpd(:,k) = dpd_k;
end

ep = pL - pd;
ev = vL - dpd;

figure;
subplot(2,1,1);
plot(t, ep');
title('Payload Position Error');
legend('x', 'y', 'z');
grid on;

subplot(2,1,2);
plot(t, ev');
title('Payload Velocity Error');
legend('vx', 'vy', 'vz');
grid on;

% 3D Trajectory Plot
figure;
hold on;
% Plot Payload Trajectory
plot3(pL(1,:), pL(2,:), pL(3,:), 'k--', 'LineWidth', 2);

% Plot UAV Trajectories
% q 定义为从 UAV 指向 Payload 的方向（NED坐标系）
% 所以 UAV 位置 = Payload 位置 - 缆绳长度 * q
% 在 NED 中，q 的 Z 分量为正表示向下，所以 UAV 在 Payload 上方（Z 值更小）
colors = ['r', 'g', 'b'];
for i = 1:n
    pi_traj = zeros(3, len);
    for k = 1:len
        [pL_k, ~, ~, q_k, ~, ~, ~] = unpack_state(x(k,:)', n);
        % UAV 位置 = 载荷位置 - 缆绳向量（q 从 UAV 指向载荷）
        pi_traj(:,k) = pL_k - params.li * q_k(:,i);
    end
    plot3(pi_traj(1,:), pi_traj(2,:), pi_traj(3,:), colors(i), 'LineWidth', 1.5);
end

title('3D Trajectories (NED: Z-down)');
xlabel('North (X)');
ylabel('East (Y)');
zlabel('Down (Z)');
legend('Payload', 'UAV 1', 'UAV 2', 'UAV 3');
grid on;
axis equal;
set(gca, 'ZDir', 'reverse'); % Important for NED visualization
view(3);
end
