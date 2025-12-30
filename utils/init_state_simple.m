function x0 = init_state_simple(p_in)
% INIT_STATE_SIMPLE 简单初始化：所有无人机垂直向上悬停
%   不计算复杂的初始姿态，使用最简单的配置

% 兼容输入
if isstruct(p_in)
    p = p_in;
else
    p = params();
    p.n = p_in;
end

n = p.n;

% 载荷初始状态：固定悬停
pL0 = [0; 0; -2];
vL0 = [0; 0; 0];  % 静止

% 缆绳方向：期望配置
theta_d = p.theta_d;
q_all = zeros(3,n);
omega_all = zeros(3,n);

for i = 1:n
    psi_di = deg2rad(90 + (i-1)*120);
    q_di = [cos(psi_di)*sin(theta_d); sin(psi_di)*sin(theta_d); cos(theta_d)];
    q_all(:,i) = q_di / norm(q_di);
    omega_all(:,i) = [0; 0; 0];
end

% 所有无人机姿态：垂直向上（单位矩阵）
R_all = repmat(eye(3), [1, 1, n]);

% 扰动估计初值
dL_hat0 = [0; 0; 0];
d_hat0 = zeros(3,n);

% 打包状态
x0 = pack_state(pL0, vL0, R_all, q_all, omega_all, dL_hat0, d_hat0);
end
