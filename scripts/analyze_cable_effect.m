% ANALYZE_CABLE_EFFECT - 分析缆绳角度对垂直升力的影响
clear; clc;

fprintf('=== 缆绳角度对垂直升力的影响 ===\n\n');

% 假设推力恒定
T_constant = 15;  % 恒定推力 15N

% 不同缆绳天顶角
theta_list = [20, 30, 40, 50, 60, 70, 80, 90];  % 度

fprintf('假设每架UAV推力恒定 T = %.2f N\n', T_constant);
fprintf('3架UAV总推力 = %.2f N\n\n', 3*T_constant);

fprintf('缆绳角度   缆绳垂直分量   单机垂直升力   总垂直升力   重力   净升力\n');
fprintf('------------------------------------------------------------------------\n');

mL = 0.4;  % 载荷质量
mi = 1.5;  % UAV质量
n = 3;     % UAV数量
g = 9.81;  % 重力加速度

total_weight = (mL + n*mi) * g;  % 总重量

for theta_deg = theta_list
    theta_rad = deg2rad(theta_deg);

    % 缆绳垂直分量 (天顶角的余弦)
    q_z = cos(theta_rad);

    % 推力在缆绳方向的投影（假设推力向上，与缆绳方向相反）
    % f_qi = (q^T · f_i) · q
    % 如果 f_i 完全向上 = [0, 0, -T]，q = [qx, qy, qz]
    % 则 q^T · f_i = -T * qz
    % f_qi = -T * qz * q
    % f_qi 的垂直分量 = -T * qz * qz = -T * qz²

    % 简化：假设推力方向就是缆绳反方向（理想情况）
    % 则 f_qi = T * q，垂直分量 = T * qz

    vertical_force_per_uav = T_constant * q_z;  % 单机垂直升力
    total_vertical = n * vertical_force_per_uav;  % 总垂直升力

    net_force = total_vertical - total_weight;  % 净升力

    fprintf('%6.0f°      %.4f          %.2f N         %.2f N     %.2f N   %+.2f N\n', ...
        theta_deg, q_z, vertical_force_per_uav, total_vertical, total_weight, net_force);
end

fprintf('\n');
fprintf('结论:\n');
fprintf('1. 缆绳角度从20°增加到66°时，垂直分量从0.94降到0.40 (降低57%%)\n');
fprintf('2. 如果推力保持15N，垂直升力从42.3N降到18N (不足以抵消重力44.5N)\n');
fprintf('3. 系统会开始下坠，缆绳角度进一步增大，形成恶性循环！\n\n');

fprintf('正确的响应应该是:\n');
fprintf('- 当缆绳角度增大时，控制器应该**增大推力**来补偿垂直分量的减少\n');
fprintf('- 但实际观察到推力在**减小**（从15.8N降到12.6N）\n');
fprintf('- 这说明控制器没有正确处理缆绳角度的变化\n');

quit;
