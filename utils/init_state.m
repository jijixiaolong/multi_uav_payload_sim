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

% 初始缆绳方向：直接使用期望配置（确保对称性和高度一致）
% q_i 指向：载荷 → 无人机
% NED坐标系：无人机在上方（z更负），但 q_z = (pL_z - p_i_z)/li > 0
% 配置：水平均匀分布（相差120°），天顶角40°
% 策略：直接从期望配置开始，避免初始大幅调整导致的不对称
theta_init = p.theta_d;  % 使用期望天顶角40°作为初值
q_all = zeros(3,n);
omega_all = zeros(3,n);

for i = 1:n
    % 方位角: ψ_i = (i-1) × 120° = [0°, 120°, 240°]（水平均匀分布）
    psi_i = deg2rad((i-1) * 120);
    % 球坐标转笛卡尔：q = [sin(θ)cos(ψ), sin(θ)sin(ψ), cos(θ)]
    q_i = [sin(theta_init)*cos(psi_i);
           sin(theta_init)*sin(psi_i);
           cos(theta_init)];
    q_all(:,i) = q_i / norm(q_i);
    omega_all(:,i) = [0; 0; 0];
end

% 计算质量矩阵
M = p.mL * eye(3) + p.mi * (q_all * q_all') + 1e-8 * eye(3);

% 扰动估计初值
dL_hat0 = [0; 0; 0];
d_hat0 = zeros(3,n);

% 简化模型：直接使用单位姿态矩阵（因为姿态不参与动力学）
% 姿态矩阵在简化模型中仅用于记录，不影响动力学计算
e3 = [0; 0; 1];
R_all = repmat(eye(3), [1, 1, n]);  % 所有UAV初始姿态为单位阵

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
