function dx = system_dynamics(t, x, params)
% SYSTEM_DYNAMICS 完整系统状态微分 dx = f(x,u)
%
%   输入:
%   t: 时间
%   x: 状态向量
%   params: 参数结构体
%
%   输出:
%   dx: 状态导数

% 解包状态（状态中不包含 Omega）
n = params.n;

% 检查输入状态是否有 NaN 或 Inf（ODE 求解器可能在探索时产生）
if any(~isfinite(x))
    % 返回零导数，让 ODE 求解器减小步长
    dx = zeros(size(x));
    return;
end

[pL, vL, R, q, omega, dL_hat, d_hat] = unpack_state(x, n);

% 保持旋转矩阵正交，抑制积分漂移导致的数值爆炸
R = reorthonormalize_R(R);

% 归一化每根缆绳方向，防止数值漂移（向量化）
q_norms = sqrt(sum(q.^2, 1));
% 防止除以零
q_norms(q_norms < 1e-10) = 1;
q = q ./ q_norms;

% 期望轨迹与高阶导数
[pd, dpd, d2pd, d3pd, d4pd] = trajectory(t);

% 预计算矩阵 M（向量化计算）

M = params.mL * eye(3) + params.mi * (q * q') + 1e-8 * eye(3);

% 调用控制器（传递所有预解包的状态变量，避免重复 unpack_state）
% 控制器返回：
% T: 各机推力大小 (1xn)
% Omega_cmd: 命令角速度 (3xn)
% dL_hat_dot, d_hat_dot: 扰动估计导数
[T, Omega_cmd, dL_hat_dot, d_hat_dot] = controller_wrapper(params, pd, dpd, d2pd, d3pd, d4pd, M, pL, vL, R, q, omega, dL_hat, d_hat);

% 动力学计算

% 1. 载荷动力学 (Eq 8)
% M * vL_dot = sum( (q_i' * f_i) * q_i ) + d_L + mL * g * e3
% f_i = -T_i * R_i * e3（推力向量）

% 向量化推力计算
e3 = [0;0;1];
f = zeros(3, n);
for i = 1:n
    f(:,i) = -T(i) * R(:,:,i) * e3;
end

% 向量化力分量计算（无循环）
% 计算各力在缆绳方向的投影系数: (q_i' * f_i) 和 (q_i' * d_i)
f_proj = sum(q .* f, 1);                    % 1×n: 推力投影系数
d_proj = sum(q .* params.d_i_true, 1);      % 1×n: 扰动投影系数
omega_norm_sq = sum(omega.^2, 1);           % 1×n: ||omega_i||^2

% 合并系数: (f_proj + d_proj - mi*li*||omega||^2)
mi = params.mi; li = params.li;
coeff = f_proj + d_proj - mi * li * omega_norm_sq;  % 1×n

% 求和: sum_i(coeff_i * q_i)
sum_terms = q * coeff';                     % 3×1

d_L_true = params.d_L_true;

% 按 Eq 8 求解 vL_dot，重力项在逆矩阵外
% NED 中重力为 [0;0;g]
dvL = M \ (sum_terms + d_L_true) + params.g * e3;
dpL = vL;

% 2. 缆绳动力学 (Eq 9) - 向量化
% 利用 cross() 支持批量列计算: S(q)*v = cross(q, v)

g_e3 = params.g * e3;
dvL_minus_g = dvL - g_e3;

% term1: (1/li) * cross(q_i, dvL_minus_g) 对所有 i
dvL_minus_g_mat = repmat(dvL_minus_g, 1, n);        % 3×n
term1_all = (1/li) * cross(q, dvL_minus_g_mat);     % 3×n

% term2: (1/(mi*li)) * cross(q_i, f_i + d_i)
f_plus_d = f + params.d_i_true;                     % 3×n
term2_all = (1/(mi*li)) * cross(q, f_plus_d);       % 3×n

domega = term1_all - term2_all;                     % 3×n: 缆绳角加速度
dq = cross(omega, q);                               % 3×n: q̇ = ω × q

% 3. 姿态运动学 (Eq 1c)
% R_dot = R * S(Omega)
% 这里假设 Omega_cmd 可被完美跟踪
dR = zeros(3,3,n);
for i = 1:n
    dR(:,:,i) = R(:,:,i) * hat(Omega_cmd(:,i));
end

% 打包导数（无 Omega 导数）
dx = pack_state(dpL, dvL, dR, dq, domega, dL_hat_dot, d_hat_dot);

end

function [T, Omega_cmd, dL_hat_dot, d_hat_dot] = controller_wrapper(params, pd, dpd, d2pd, d3pd, d4pd, M, pL, vL, R, q, omega, dL_hat, d_hat)
% 传递所有预解包的状态变量，避免重复 unpack_state

% 1. 载荷位置控制
[f_dL, e, ev] = payload_ctrl(params, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);

% 2. 力分配
f_qdi = force_allocation(params, f_dL, q, omega);

% 3. 缆绳姿态控制
[f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(params, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);

% 4. 姿态控制
[Omega_cmd, T] = attitude_ctrl(params, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R, q, omega);

% 5. 扰动估计
[dL_hat_dot, d_hat_dot] = disturbance_est(params, e, ev, e_omega_i, M, q, omega, dL_hat, d_hat);
end

function R_out = reorthonormalize_R(R_in)
% 使用极分解保持旋转矩阵正交，防止长期仿真时的漂移
n = size(R_in, 3);
R_out = R_in;
for i = 1:n
    [U, ~, V] = svd(R_in(:,:,i));
    Ri = U * V';
    if det(Ri) < 0
        U(:,3) = -U(:,3);
        Ri = U * V';
    end
    R_out(:,:,i) = Ri;
end
end
