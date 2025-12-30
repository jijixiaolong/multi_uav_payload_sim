function [pd, dpd, d2pd, d3pd, d4pd] = trajectory(t)
% TRAJECTORY 生成期望轨迹
%   可选择不同轨迹类型
%
%   轨迹类型:
%   'line'      - 直线轨迹（沿X轴）
%   'rectangle' - 矩形轨迹（圆角）
%   'circle'    - 圆形轨迹
%   'figure8'   - 8字形轨迹（平面）
%   'helix'     - 螺旋轨迹（3D上升/下降）
%   'sine_wave' - 正弦波轨迹（沿X轴）
%   'polygon'   - 正六边形轨迹
%   'hover'     - 悬停（固定点）
%   'clelia'    - Clelia曲线（论文Wang et al. 2024, Eq.39）

% ========== 选择轨迹类型 ==========
traj_type = 'circle';  % 可选: 'line', 'rectangle', 'circle', 'figure8',
                       %       'helix', 'sine_wave', 'polygon', 'hover', 'clelia'
% 注意: 'clelia' 轨迹过于激进，会导致系统不稳定
% 建议使用 'circle', 'figure8', 'sine_wave' 等轨迹

% 公共参数
z0 = -2;  % 高度 (NED: 负值表示向上, -2表示2m高度)

switch traj_type
    case 'line'
        % 直线轨迹：沿 X 轴正方向匀速运动
        vx = 0.05;  % X 方向速度 (m/s)

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

    case 'figure8'
        % 8字形轨迹（平面）
        [pd, dpd, d2pd, d3pd, d4pd] = figure8_trajectory(t, z0);

    case 'helix'
        % 螺旋轨迹（3D上升）
        [pd, dpd, d2pd, d3pd, d4pd] = helix_trajectory(t, z0);

    case 'sine_wave'
        % 正弦波轨迹（沿X轴）
        [pd, dpd, d2pd, d3pd, d4pd] = sine_wave_trajectory(t, z0);

    case 'polygon'
        % 正六边形轨迹
        [pd, dpd, d2pd, d3pd, d4pd] = polygon_trajectory(t, z0);

    case 'hover'
        % 悬停（固定点）
        pd = [0; 0; z0];
        dpd = [0; 0; 0];
        d2pd = [0; 0; 0];
        d3pd = [0; 0; 0];
        d4pd = [0; 0; 0];

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

%% 8字形轨迹子函数
function [pd, dpd, d2pd, d3pd, d4pd] = figure8_trajectory(t, z0)
% 8字形轨迹（利萨如曲线）：x = A*sin(ωt), y = B*sin(2ωt)
% 设计: 慢速8字形，保证稳定
% 周期 T = 20s → 角速度 ω = 2π/T = 0.314 rad/s
% 最大速度约 0.4 m/s，最大加速度约 0.25 m/s²

A = 1.0;              % X方向振幅 (m)
B = 0.6;              % Y方向振幅 (m)
T_period = 20.0;      % 周期 (s)
omega = 2*pi/T_period;  % 角速度 (rad/s)

theta = omega * t;

% 位置: x = A*sin(θ), y = B*sin(2θ)
pd = [A*sin(theta); B*sin(2*theta); z0];

% 速度: dx = A*ω*cos(θ), dy = 2B*ω*cos(2θ)
dpd = [A*omega*cos(theta); 2*B*omega*cos(2*theta); 0];

% 加速度: d²x = -A*ω²*sin(θ), d²y = -4B*ω²*sin(2θ)
d2pd = [-A*omega^2*sin(theta); -4*B*omega^2*sin(2*theta); 0];

% 三阶导数
d3pd = [-A*omega^3*cos(theta); -8*B*omega^3*cos(2*theta); 0];

% 四阶导数
d4pd = [A*omega^4*sin(theta); 16*B*omega^4*sin(2*theta); 0];

end

%% 螺旋轨迹子函数
function [pd, dpd, d2pd, d3pd, d4pd] = helix_trajectory(t, z0)
% 螺旋轨迹：在XY平面做圆周运动，同时Z方向上升/下降
% 设计: 慢速螺旋，保证稳定
% XY平面: 圆形轨迹，半径 R = 0.8m，周期 T = 20s
% Z方向: 缓慢上升，速度 vz = 0.05 m/s（每20s上升1m）

R = 0.8;              % 螺旋半径 (m)
T_period = 20.0;      % XY平面旋转周期 (s)
vz = 0.05;            % Z方向速度 (m/s，向上为负)
omega = 2*pi/T_period;  % 角速度 (rad/s)

theta = omega * t;
z = z0 - vz * t;      % NED坐标系，向上为负

% 位置
pd = [R*cos(theta); R*sin(theta); z];

% 速度
dpd = [-R*omega*sin(theta); R*omega*cos(theta); -vz];

% 加速度
d2pd = [-R*omega^2*cos(theta); -R*omega^2*sin(theta); 0];

% 三阶导数
d3pd = [R*omega^3*sin(theta); -R*omega^3*cos(theta); 0];

% 四阶导数
d4pd = [R*omega^4*cos(theta); R*omega^4*sin(theta); 0];

end

%% 正弦波轨迹子函数
function [pd, dpd, d2pd, d3pd, d4pd] = sine_wave_trajectory(t, z0)
% 正弦波轨迹：沿X轴前进，Y方向做正弦振荡
% 设计: x = vx*t, y = A*sin(kx) = A*sin(k*vx*t)
% 参数: vx = 0.1 m/s（前进速度）
%       A = 0.5 m（振幅）
%       λ = 2 m（波长） → k = 2π/λ = π

vx = 0.1;             % X方向前进速度 (m/s)
A = 0.5;              % Y方向振幅 (m)
wavelength = 2.0;     % 波长 (m)
k = 2*pi/wavelength;  % 波数 (rad/m)

x = vx * t;
kx = k * x;           % k*vx*t
omega = k * vx;       % 有效角频率

% 位置
pd = [x; A*sin(kx); z0];

% 速度: dx = vx, dy = A*k*vx*cos(kx)
dpd = [vx; A*omega*cos(kx); 0];

% 加速度: d²y = -A*(k*vx)²*sin(kx)
d2pd = [0; -A*omega^2*sin(kx); 0];

% 三阶导数
d3pd = [0; -A*omega^3*cos(kx); 0];

% 四阶导数
d4pd = [0; A*omega^4*sin(kx); 0];

end

%% 正多边形轨迹子函数
function [pd, dpd, d2pd, d3pd, d4pd] = polygon_trajectory(t, z0)
% 正六边形轨迹：圆角六边形
% 设计: 外接圆半径 R = 0.8m，速度 v = 0.3 m/s
%       圆角半径 r = 0.15m，确保速度连续

R = 0.8;              % 外接圆半径 (m)
n_sides = 6;          % 边数
v = 0.3;              % 运动速度 (m/s)
r_corner = 0.15;      % 圆角半径 (m)

% 正多边形参数
alpha = 2*pi/n_sides;           % 中心角 (rad) = 60°
side_length = 2*R*sin(alpha/2); % 边长

% 直边长度（减去两端圆角）
L_straight = side_length - 2*r_corner*tan(alpha/2);
L_arc = alpha * r_corner;       % 圆角弧长

% 总周长和周期
L_total = n_sides * (L_straight + L_arc);
T_period = L_total / v;

% 归一化时间
t_mod = mod(t, T_period);
s = v * t_mod;

% 单段长度
L_segment = L_straight + L_arc;

% 确定当前在哪条边
segment = floor(s / L_segment);
segment = min(segment, n_sides - 1);  % 限制在 [0, n_sides-1]
s_local = s - segment * L_segment;

% 当前边的起始角度（从X轴正方向开始，逆时针）
theta_start = pi/2 - segment * alpha;

% 边的方向向量
edge_dir = [cos(theta_start - alpha); sin(theta_start - alpha)];

% 边的起点（外接圆上）
corner_angle = theta_start - alpha/2;
p_start = R * [cos(corner_angle); sin(corner_angle)];

% 沿边向内移动到直边起点（考虑圆角）
offset = r_corner * tan(alpha/2);
p_straight_start = p_start + offset * edge_dir;

if s_local < L_straight
    % 在直边上
    pd = [p_straight_start + s_local * edge_dir; z0];
    dpd = [v * edge_dir; 0];
    d2pd = [0; 0; 0];
    d3pd = [0; 0; 0];
    d4pd = [0; 0; 0];
else
    % 在圆角上
    ds = s_local - L_straight;
    theta_arc = ds / r_corner;  % 圆角转过的角度

    % 圆角圆心（在下一个顶点内侧）
    next_corner_angle = corner_angle - alpha;
    p_corner_center = R * [cos(next_corner_angle); sin(next_corner_angle)] + ...
                      r_corner * [cos(next_corner_angle + pi/2); sin(next_corner_angle + pi/2)];

    % 圆角起始角度（从边的方向开始）
    arc_start_angle = theta_start - alpha + pi;
    arc_angle = arc_start_angle - theta_arc;

    pd = [p_corner_center + r_corner * [cos(arc_angle); sin(arc_angle)]; z0];
    dpd = v * [-sin(arc_angle); cos(arc_angle); 0];
    d2pd = (v^2/r_corner) * [-cos(arc_angle); -sin(arc_angle); 0];
    d3pd = (v^3/r_corner^2) * [sin(arc_angle); -cos(arc_angle); 0];
    d4pd = (v^4/r_corner^3) * [cos(arc_angle); sin(arc_angle); 0];
end

end
