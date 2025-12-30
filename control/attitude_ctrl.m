function [Omega_cmd, T] = attitude_ctrl(params, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R, q, omega)
% ATTITUDE_CTRL 姿态控制 (论文公式 Eq 33)
%   计算各无人机的期望角速度 Omega_cmd 和实际推力大小 T (Eq 29)
%
%   输入:
%   params:     参数结构体
%   f_qdi:      缆绳方向期望力 (3×n)
%   f_di_perp:  缆绳垂直期望力 (3×n)
%   omega_di, dot_omega_di: 期望缆绳角速度及导数 (3×n)
%   e_qi, e_omega_i: 缆绳误差 (3×n)
%   ev:         负载速度误差 (3×1)
%   e:          负载耦合误差 (3×1)
%   M:          预计算质量矩阵 (3×3)
%   R:          各机旋转矩阵 (3×3×n)
%   q:          缆绳方向单位向量 (3×n)
%   omega:      缆绳角速度 (3×n)
%
%   输出:
%   Omega_cmd:  命令角速度 (3×n)
%   T:          实际推力大小 (1×n)，按 Eq 29: T_i = T_di * r_di^T * r_i

n = params.n;

% 期望总力 f_di = f_qdi + f_di_perp
f_di = f_qdi + f_di_perp;

% dV1/dev 计算
dV1_dev = params.beta * sat(e) + ev;

% sum_term 向量化: Σ (hω/li) * S(ω_i)^T * e_ω_i
% 利用 S(a)^T * b = -a × b = b × a
% 所以 S(ω_i)^T * e_ω_i = cross(e_omega_i, omega)
homega_li = params.homega / params.li;
cross_terms = cross(e_omega_i, omega);      % 3×n
sum_term = homega_li * sum(cross_terms, 2); % 3×1

dV3_dev = dV1_dev - sum_term;

% 预计算 M \ dV3_dev
M_inv_dV3_dev = M \ dV3_dev;

% 预计算常量
e3 = [0;0;1];
S_e3 = hat(e3);
kr_hr = params.kr / params.hr;
inv_hr = 1 / params.hr;

% 向量化推力大小计算
T_d = sqrt(sum(f_di.^2, 1));                % 1×n: ||f_di||

% 期望推力方向 r_di (3×n)
% f_di = -T_di * r_di，所以 r_di = -f_di / ||f_di||
small_thrust = T_d < 1e-6;
T_d_safe = T_d;
T_d_safe(small_thrust) = 1;                 % 防止除零
r_di = -f_di ./ T_d_safe;
r_di(:, small_thrust) = repmat(-e3, 1, sum(small_thrust));  % 默认推力向上（-z in NED）

% 向量化偏航角计算
% psi_i = atan2(R_i(2,1), R_i(1,1))
R_21 = squeeze(R(2,1,:))';                  % 1×n
R_11 = squeeze(R(1,1,:))';                  % 1×n
psi = atan2(R_21, R_11);                    % 1×n: 当前偏航角

% 期望偏航角: 所有无人机统一朝向 x 轴正方向
psi_d = zeros(1, n);                        % 1×n: 全部为0
z = -params.kpsi * (psi - psi_d);           % 1×n: 偏航控制量

% 提取各机 r_i = R_i(:,3) 和预计算 M^(-1)*dV3_dev 在 q 上的投影
r = squeeze(R(:,3,:));                      % 3×n: 各机当前推力方向
q_proj = sum(q .* M_inv_dV3_dev, 1);        % 1×n: q_i^T * M^(-1)*dV3_dev

rdi_dot_r = sum(r_di .* r, 1);              % 1×n: 期望与实际方向点积
T = T_d .* rdi_dot_r;                        % 1×n: 实际推力

% 推力上限保护：避免数值爆炸引起刚性
T_max = 60;                                  % N, 保守上限
T = min(T, T_max);

% 推力下限保护：防止推力降到重力以下导致系统下坠/缆绳松弛
% 每架UAV至少需要抵消自身重力 + 1/n的载荷重力
T_min = params.mi * params.g + (params.mL * params.g) / n;  % 最小推力
T = max(T, T_min);                           % 应用下限

% 主循环 - 保留必要的矩阵运算
Omega_cmd = zeros(3, n);
omega_cmd_limit = params.omega_cmd_limit;
for i = 1:n
    R_i_t = R(:,:,i)';

    % 稳定项: -kr/hr * S(e3) * R_i^T * S(r_di) * r_i
    term_stab = -kr_hr * S_e3 * R_i_t * hat(r_di(:,i)) * r(:,i);

    % 反馈项1: T_d * /hr * S(e3) * R_i^T * hω * e_ω_i
    term_fb1 = (T_d(i) * inv_hr * params.homega) * S_e3 * R_i_t * e_omega_i(:,i);

    % 反馈项2: T_d / hr * S(e3) * R_i^T * q_i * (q_i^T * M^(-1) * dV3_dev)
    term_fb2 = (T_d(i) * inv_hr * q_proj(i)) * S_e3 * R_i_t * q(:,i);

    % 合并: z_i * e3 + 各项 (Eq 33)
    Omega_cmd(:,i) = z(i) * e3 + term_stab + term_fb1 + term_fb2;

    % 限幅角速度命令，避免求解器因过大刚度崩溃
    omega_norm = norm(Omega_cmd(:,i));
    if omega_norm > omega_cmd_limit
        Omega_cmd(:,i) = Omega_cmd(:,i) * (omega_cmd_limit / omega_norm);
    end
end
end
