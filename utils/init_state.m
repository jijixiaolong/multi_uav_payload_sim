function x0 = init_state(p_in)
% INIT_STATE 初始化状态向量（与模型/论文约定一致）
%   x0 = init_state(p) 接受参数结构体 p（包含 n, li 等）。
%   通过迭代计算精确的初始姿态，使系统从平衡状态启动。

% 兼容输入
if isstruct(p_in)
    p = p_in;
else
    p = params();
    p.n = p_in;
end

n = p.n;

% 载荷初始状态：对齐期望轨迹，使初始误差为零
[pd0, dpd0, d2pd0, d3pd0, ~] = trajectory(0);
pL0 = pd0;
vL0 = dpd0;

% 期望/初始缆绳方向
% 三架无人机均匀分布在载荷正上方，呈等边三角形
% 方位角: 90°, 210°, 330° (或等效 90°, -150°, -30°)
% 这样载荷在三角形的正中心
theta_d = p.theta_d;  % 天顶角：缆绳与垂直方向的夹角
q_all = zeros(3,n);
omega_all = zeros(3,n);

for i = 1:n
    % 方位角: 90° + (i-1)*120° = [90°, 210°, 330°]
    % 使载荷位于三角形中心
    psi_di = deg2rad(90 + (i-1)*120);
    q_di = [cos(psi_di)*sin(theta_d); sin(psi_di)*sin(theta_d); cos(theta_d)];
    q_all(:,i) = q_di / norm(q_di);
    omega_all(:,i) = [0; 0; 0];
end

% 计算质量矩阵
M = p.mL * eye(3) + p.mi * (q_all * q_all') + 1e-8 * eye(3);

% 扰动估计初值
dL_hat0 = [0; 0; 0];
d_hat0 = zeros(3,n);

% 初始误差为零
e = [0; 0; 0];
ev = [0; 0; 0];

% 调用控制器计算初始期望力
[f_dL, ~, ~] = payload_ctrl(p, pd0, dpd0, d2pd0, M, pL0, vL0, q_all, dL_hat0, d_hat0);
f_qdi = force_allocation(p, f_dL, q_all, omega_all);
[f_di_perp, ~, ~, ~, ~] = cable_ctrl(p, f_dL, e, ev, dpd0, d2pd0, d3pd0, q_all, omega_all, d_hat0);

% 计算每架无人机的期望力和期望推力方向
f_di = f_qdi + f_di_perp;

e3 = [0; 0; 1];
R_all = zeros(3,3,n);

for i = 1:n
    % 期望推力方向: r_di = -f_di / ||f_di||
    f_di_norm = norm(f_di(:,i));
    if f_di_norm > 1e-6
        r_di = -f_di(:,i) / f_di_norm;
    else
        r_di = e3;
    end

    % 构建旋转矩阵，使 R_i * e3 = r_di
    R_all(:,:,i) = rotation_from_z_to_target(e3, r_di);
end

% 打包状态
x0 = pack_state(pL0, vL0, R_all, q_all, omega_all, dL_hat0, d_hat0);
end

function R = rotation_from_z_to_target(z_axis, target)
% 计算将 z_axis 旋转到 target 的旋转矩阵
% 使用 Rodrigues 公式

z_axis = z_axis / norm(z_axis);
target = target / norm(target);

% 旋转轴 k = z × target
k = cross(z_axis, target);
k_norm = norm(k);

if k_norm < 1e-6
    % z_axis 和 target 平行
    if dot(z_axis, target) > 0
        R = eye(3);  % 同向
    else
        % 反向，绕 x 轴旋转 180°
        R = diag([1, -1, -1]);
    end
    return;
end

k = k / k_norm;

% 旋转角
cos_theta = dot(z_axis, target);
sin_theta = k_norm;

% Rodrigues 公式: R = I + sin(θ)*K + (1-cos(θ))*K²
K = hat(k);
R = eye(3) + sin_theta * K + (1 - cos_theta) * (K * K);
end
