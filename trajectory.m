function [pd, dpd, d2pd, d3pd, d4pd] = trajectory(t)
% TRAJECTORY 生成期望轨迹
%   可选择不同轨迹类型
%
%   轨迹类型:
%   'line'      - 直线轨迹（沿X轴）
%   'rectangle' - 矩形轨迹（圆角）
%   'circle'    - 圆形轨迹
%   'clelia'    - Clelia曲线（论文Wang et al. 2024, Eq.39）

% ========== 选择轨迹类型 ==========
traj_type = 'clelia';  % 'line', 'rectangle', 'circle', 'clelia'

% 公共参数
z0 = -2;  % 高度 (NED: 负值表示向上, -2表示2m高度)

switch traj_type
    case 'line'
        % 直线轨迹：沿 X 轴正方向匀速运动
        vx = 0.05;  % X 方向速度 (m/s) - 超慢速度确保稳定

        pd = [vx * t; 0; z0];
        dpd = [vx; 0; 0];
        d2pd = [0; 0; 0];
        d3pd = [0; 0; 0];
        d4pd = [0; 0; 0];

    case 'circle'
        % 圆形轨迹
        % 设计: 非常慢的圆，保证稳定
        % 周期 T = 20s → 角速度 ω = 2π/T = 0.314 rad/s
        % 半径 R = 1.0m → 切向速度 v = R*ω = 0.314 m/s
        % 向心加速度 a = v²/R = R*ω² = 0.1 m/s² (很小)

        R_circle = 1.0;           % 圆半径 (m)
        T_period = 20.0;          % 周期 (s) - 慢速
        omega_c = 2*pi/T_period;  % 角速度 (rad/s) = 0.314

        cx = R_circle;     % 圆心X (使起点在原点)
        cy = 0;            % 圆心Y

        theta = omega_c * t;
        v_t = R_circle * omega_c;  % 切向速度 = 0.314 m/s

        % 起点在 (0, 0, z0)，逆时针旋转
        pd = [cx + R_circle*cos(theta - pi); cy + R_circle*sin(theta - pi); z0];
        dpd = v_t * [-sin(theta - pi); cos(theta - pi); 0];
        d2pd = (v_t * omega_c) * [-cos(theta - pi); -sin(theta - pi); 0];
        d3pd = (v_t * omega_c^2) * [sin(theta - pi); -cos(theta - pi); 0];
        d4pd = (v_t * omega_c^3) * [cos(theta - pi); sin(theta - pi); 0];

    case 'rectangle'
        % 矩形轨迹（圆角过渡）
        [pd, dpd, d2pd, d3pd, d4pd] = rectangle_trajectory(t, z0);

    case 'clelia'
        % Clelia曲线（论文 Wang et al. 2024, Eq.39）
        % 3D螺旋曲线，恒定速度 3 m/s
        [pd, dpd, d2pd, d3pd, d4pd] = trajectory_clelia(t);

    otherwise
        error('Unknown trajectory type: %s', traj_type);
end

end

%% 矩形轨迹子函数
function [pd, dpd, d2pd, d3pd, d4pd] = rectangle_trajectory(t, z0)
% 矩形轨迹：在XY平面上沿矩形路径运动
% 使用圆角过渡，保证速度连续
%
% 设计: 小矩形，约8秒一圈（保守设计，保证稳定）
% 周长 ≈ 2*0.6 + 2*0.4 + 2*pi*0.2 = 1.2 + 0.8 + 1.26 = 3.26 m
% 速度 = 0.4 m/s → 周期 = 3.26/0.4 = 8.15 s
% 向心加速度 = 0.4^2/0.2 = 0.8 m/s^2 (可接受)

% 轨迹参数
Lx = 1.0;       % X方向边长 (m)
Ly = 0.8;       % Y方向边长 (m)
r = 0.2;        % 圆角半径 (m) - 较大圆角减小加速度
v = 0.4;        % 运动速度 (m/s)

% 向心加速度 = v^2/r = 0.4^2/0.2 = 0.8 m/s^2

% 计算各段长度
L_straight_x = Lx - 2*r;
L_straight_y = Ly - 2*r;
L_arc = (pi/2) * r;

% 总周长和周期
L_total = 2 * L_straight_x + 2 * L_straight_y + 4 * L_arc;
T_period = L_total / v;

% 归一化时间
t_mod = mod(t, T_period);
s = v * t_mod;

% 各段累积长度
s1 = L_straight_x;
s2 = s1 + L_arc;
s3 = s2 + L_straight_y;
s4 = s3 + L_arc;
s5 = s4 + L_straight_x;
s6 = s5 + L_arc;
s7 = s6 + L_straight_y;

% 起点位置
x0 = r;
y0 = 0;

% 根据弧长计算位置和导数
if s < s1
    % 段1: 底边 (向右)
    pd = [x0 + s; y0; z0];
    dpd = [v; 0; 0];
    d2pd = [0; 0; 0];
    d3pd = [0; 0; 0];
    d4pd = [0; 0; 0];

elseif s < s2
    % 段2: 右下圆角
    ds = s - s1;
    theta = ds / r;
    cx = x0 + L_straight_x;
    cy = y0 + r;

    pd = [cx + r*cos(-pi/2 + theta); cy + r*sin(-pi/2 + theta); z0];
    dpd = v * [-sin(-pi/2 + theta); cos(-pi/2 + theta); 0];
    d2pd = (v^2/r) * [-cos(-pi/2 + theta); -sin(-pi/2 + theta); 0];
    d3pd = (v^3/r^2) * [sin(-pi/2 + theta); -cos(-pi/2 + theta); 0];
    d4pd = (v^4/r^3) * [cos(-pi/2 + theta); sin(-pi/2 + theta); 0];

elseif s < s3
    % 段3: 右边 (向上)
    ds = s - s2;
    pd = [x0 + L_straight_x + r; y0 + r + ds; z0];
    dpd = [0; v; 0];
    d2pd = [0; 0; 0];
    d3pd = [0; 0; 0];
    d4pd = [0; 0; 0];

elseif s < s4
    % 段4: 右上圆角
    ds = s - s3;
    theta = ds / r;
    cx = x0 + L_straight_x;
    cy = y0 + r + L_straight_y;

    pd = [cx + r*cos(theta); cy + r*sin(theta); z0];
    dpd = v * [-sin(theta); cos(theta); 0];
    d2pd = (v^2/r) * [-cos(theta); -sin(theta); 0];
    d3pd = (v^3/r^2) * [sin(theta); -cos(theta); 0];
    d4pd = (v^4/r^3) * [cos(theta); sin(theta); 0];

elseif s < s5
    % 段5: 顶边 (向左)
    ds = s - s4;
    pd = [x0 + L_straight_x - ds; y0 + r + L_straight_y + r; z0];
    dpd = [-v; 0; 0];
    d2pd = [0; 0; 0];
    d3pd = [0; 0; 0];
    d4pd = [0; 0; 0];

elseif s < s6
    % 段6: 左上圆角
    ds = s - s5;
    theta = ds / r;
    cx = x0;
    cy = y0 + r + L_straight_y;

    pd = [cx + r*cos(pi/2 + theta); cy + r*sin(pi/2 + theta); z0];
    dpd = v * [-sin(pi/2 + theta); cos(pi/2 + theta); 0];
    d2pd = (v^2/r) * [-cos(pi/2 + theta); -sin(pi/2 + theta); 0];
    d3pd = (v^3/r^2) * [sin(pi/2 + theta); -cos(pi/2 + theta); 0];
    d4pd = (v^4/r^3) * [cos(pi/2 + theta); sin(pi/2 + theta); 0];

elseif s < s7
    % 段7: 左边 (向下)
    ds = s - s6;
    pd = [x0 - r; y0 + r + L_straight_y - ds; z0];
    dpd = [0; -v; 0];
    d2pd = [0; 0; 0];
    d3pd = [0; 0; 0];
    d4pd = [0; 0; 0];

else
    % 段8: 左下圆角
    ds = s - s7;
    theta = ds / r;
    cx = x0;
    cy = y0 + r;

    pd = [cx + r*cos(pi + theta); cy + r*sin(pi + theta); z0];
    dpd = v * [-sin(pi + theta); cos(pi + theta); 0];
    d2pd = (v^2/r) * [-cos(pi + theta); -sin(pi + theta); 0];
    d3pd = (v^3/r^2) * [sin(pi + theta); -cos(pi + theta); 0];
    d4pd = (v^4/r^3) * [cos(pi + theta); sin(pi + theta); 0];
end

end
