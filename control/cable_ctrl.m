function [f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(params, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat)
% CABLE_CTRL 缆绳姿态控制 (论文公式 Eq 24, 28)
%   计算各缆绳的垂直分力 f_di_perp，用于调整缆绳方向
%
%   输入:
%   params:     参数结构体
%   f_dL:       负载期望力 (用于计算 vLn)
%   e, ev:      负载误差
%   dpd, d2pd, d3pd: 轨迹导数
%   q:          缆绳方向 (3×n)
%   omega:      缆绳角速度 (3×n)
%   d_hat:      扰动估计 (3×n)
%
%   输出:
%   f_di_perp:    各缆绳垂直分力 (3×n)
%   omega_di:     期望角速度 (3×n)
%   dot_omega_di: 期望角加速度 (3×n)
%   e_qi:         缆绳方向误差 (3×n)
%   e_omega_i:    缆绳角速度误差 (3×n)

n = params.n;

% 期望缆绳配置 q_di（论文 Wang et al. 2024）- 向量化
% q_i 指向: 载荷 → 无人机
% NED坐标系中，无人机在载荷上方（z坐标更负），但 q_z = (pL_z - p_i_z)/li > 0
% 配置：水平均匀分布（相差120°），天顶角θ_d = 40°
theta_d = params.theta_d;
psi_di = deg2rad((0:n-1) * 120);           % 1×n: [0°, 120°, 240°]（均匀分布）
q_di = [sin(theta_d) * cos(psi_di);        % 3×n: 期望缆绳方向
        sin(theta_d) * sin(psi_di);
        cos(theta_d) * ones(1, n)];        % z分量为正

% vLn 名义加速度 (Eq 26) - 严格按论文公式
% dot_vLn := -σ(e) - k2·σ(β·σ(e) + ev) + p̈_d
sat_e = sat(e);
dot_vLn = -sat_e - params.k2 * sat(params.beta * sat_e + ev) + d2pd;

% 预计算常用系数
kq_hq = params.kq / params.hq;
hq_homega = params.hq / params.homega;
komega_homega = params.komega / params.homega;
inv_li = 1 / params.li;
inv_mi_li = 1 / (params.mi * params.li);
mi_li = params.mi * params.li;

% 缆绳方向误差 (向量化)
e_qi = q - q_di;                        % 3×n

% 缆绳方向变化率: dq_i = ω_i × q_i
dq = cross(omega, q);                   % 3×n

% 期望角速度 (Eq 24) - 向量化
% omega_di = S(q_di)*dq_di + (kq/hq)*S(q_i)*q_di
% 由于 dq_di = 0（固定配置），第一项为零
% omega_di = (kq/hq) * cross(q_i, q_di)
omega_di = kq_hq * cross(q, q_di);      % 3×n

% 角速度误差: e_omega_i = S(q_i)*(omega_i - omega_di)
omega_err = omega - omega_di;           % 3×n
e_omega_i = cross(q, omega_err);        % 3×n

% 期望角加速度导数 (dq_di = 0 时简化)
% dot_omega_di = (kq/hq) * (S(dq_i)*q_di + S(q_i)*dq_di)
%              = (kq/hq) * cross(dq_i, q_di)
dot_omega_di = kq_hq * cross(dq, q_di); % 3×n

% 控制律 (Eq 28) - 逐列计算（S² 难以完全向量化）
f_di_perp = zeros(3, n);
% 按论文 Eq 28: (dot_vLn - g*c3)
g_c3 = params.g * [0;0;1];
dot_vLn_minus_g = dot_vLn - g_c3;

for i = 1:n
    S_qi = hat(q(:,i));
    S2_qi = S_qi * S_qi;                % S²(q_i) = q_i*q_i^T - I

    % 括号内各项（移除扰动补偿 term3）
    term1 = inv_li * S2_qi * dot_vLn_minus_g;       % 动力学项
    term2 = -S_qi * dot_omega_di(:,i);              % 角加速度前馈
    term4 = cross(dq(:,i), omega_err(:,i));         % 科氏力项
    term5 = hq_homega * e_qi(:,i);                  % 方向误差反馈
    term6 = komega_homega * e_omega_i(:,i);         % 角速度误差反馈

    bracket = term1 + term2 + term4 + term5 + term6;
    f_di_perp(:,i) = mi_li * S2_qi * bracket;
end

% 限幅 f_di_perp，防止垂直力过大导致姿态控制饱和
% 大幅增大限幅以允许强恢复力
f_perp_max = 5.0 * params.mi * params.g;  % 最大垂直力 (提高到5倍)
for i = 1:n
    f_perp_norm = norm(f_di_perp(:,i));
    if f_perp_norm > f_perp_max
        f_di_perp(:,i) = f_di_perp(:,i) * (f_perp_max / f_perp_norm);
    end
end
end
