function dx = system_dynamics(t, x, params)
% 系统动力学状态微分方程 dx = f(x, u)
%
% 输入:
%   t      - 当前时间
%   x      - 状态向量 [pL; vL; R(:); q; omega; dL_hat; d_hat]
%   params - 系统参数结构体
%
% 输出:
%   dx - 状态导数向量

n = params.n;

% 数值安全检查
if any(~isfinite(x))
    dx = zeros(size(x));
    return;
end

% 解包状态
[pL, vL, R, q, omega, dL_hat, d_hat] = unpack_state(x, n);

% 数值稳定性处理
R = reorthonormalize_R(R);
q = normalize_cable_directions(q);

% 获取期望轨迹
[pd, dpd, d2pd, d3pd, d4pd] = trajectory(t);

% 计算惯性矩阵 M
M = params.mL * eye(3) + params.mi * (q * q') + 1e-8 * eye(3);

% 控制律
[~, ~, f_di, dL_hat_dot, d_hat_dot] = compute_control(...
    params, pd, dpd, d2pd, d3pd, d4pd, M, pL, vL, R, q, omega, dL_hat, d_hat);

% 动力学计算
[dpL, dvL] = payload_dynamics(params, M, q, omega, f_di, vL);
[dq, domega] = cable_dynamics(params, q, omega, f_di, dvL);
dR = attitude_dynamics(n, f_di);

% 打包导数
dx = pack_state(dpL, dvL, dR, dq, domega, dL_hat_dot, d_hat_dot);

end

%% 动力学子模块

function [dpL, dvL] = payload_dynamics(params, M, q, omega, f_di, vL)
% 载荷动力学 (Eq 8): M * dvL = sum((q'*f)*q) + d_L + m_L*g*e3

    e3 = [0; 0; 1];

    % 向量化力投影计算
    f_proj = sum(q .* f_di, 1);                      % 1×n
    d_proj = sum(q .* params.d_i_true, 1);           % 1×n
    omega_norm_sq = sum(omega.^2, 1);                % 1×n

    % 合成力系数
    coeff = f_proj + d_proj - params.mi * params.li * omega_norm_sq;
    sum_force = q * coeff';                          % 3×1

    % 求解加速度 (NED坐标系，重力向下为正)
    dvL = M \ (sum_force + params.d_L_true) + params.g * e3;
    dpL = vL;
end

function [dq, domega] = cable_dynamics(params, q, omega, f_di, dvL)
% 缆绳动力学 (Eq 9): domega = (1/l)*cross(q, dvL-g*e3) - (1/(m*l))*cross(q, f+d)

    e3 = [0; 0; 1];

    % 向量化交叉积计算
    dvL_minus_g = dvL - params.g * e3;
    dvL_mat = repmat(dvL_minus_g, 1, params.n);

    term1 = (1/params.li) * cross(q, dvL_mat);
    term2 = (1/(params.mi * params.li)) * cross(q, f_di + params.d_i_true);

    domega = term1 - term2;
    dq = cross(omega, q);
end

function dR = attitude_dynamics(n, f_di)
% 姿态动力学 (简化: 瞬时跟踪推力方向)

    dR = zeros(3, 3, n);
    % 简化模型: 姿态瞬间对齐推力方向，变化率为零
end

%% 控制模块

function [T, Omega_cmd, f_di, dL_hat_dot, d_hat_dot] = compute_control(...
    params, pd, dpd, d2pd, d3pd, d4pd, M, pL, vL, R, q, omega, dL_hat, d_hat)
% 分层控制架构

    % 1. 载荷位置控制
    [f_dL, e, ev] = payload_ctrl(params, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);

    % 2. 力分配
    f_qdi = force_allocation(params, f_dL, q, omega);

    % 3. 缆绳姿态控制
    [f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = ...
        cable_ctrl(params, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);

    % 4. 姿态控制
    [Omega_cmd, T, f_di] = attitude_ctrl(params, f_qdi, f_di_perp, ...
        omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R, q, omega);

    % 5. 扰动估计 (简化模型中禁用)
    dL_hat_dot = zeros(3, 1);
    d_hat_dot = zeros(3, params.n);
end

%% 辅助函数

function q_norm = normalize_cable_directions(q)
% 归一化缆绳方向向量，防止数值漂移

    q_norms = sqrt(sum(q.^2, 1));
    q_norms(q_norms < 1e-10) = 1;  % 防止除零
    q_norm = q ./ q_norms;
end

function R_out = reorthonormalize_R(R_in)
% 使用SVD极分解保持旋转矩阵正交性

    n = size(R_in, 3);
    R_out = R_in;

    for i = 1:n
        [U, ~, V] = svd(R_in(:,:,i));
        Ri = U * V';

        % 确保行列式为+1 (右手系)
        if det(Ri) < 0
            U(:,3) = -U(:,3);
            Ri = U * V';
        end

        R_out(:,:,i) = Ri;
    end
end
