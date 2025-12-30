function params = params()
% PARAMS Define simulation parameters
%   Returns a struct with all physical parameters and control gains.
%   Based on: Wang et al., "Robust Cooperative Transportation of a
%   Cable-Suspended Payload by Multiple Quadrotors", IEEE TITS 2024
%   Parameters from Section VI (Simulation Results), Page 8

% ========== Physical Parameters (论文 Section VI) ==========
params.mL = 0.06;  % Payload mass (kg) - 论文值
params.mi = 0.21;  % Quadrotor mass (kg) - 论文值
params.li = 0.6;   % Cable length (m)
params.g = 9.81;   % Gravity (m/s^2)
params.n = 3;      % Number of quadrotors

% 论文中的期望缆绳配置: θd = 40°, ψdi = (i-2)*60°
params.theta_d = deg2rad(40);  % Desired cable angle (zenith) - 论文值 40°

% ========== Control Gains (论文 Section VI) ==========
params.beta = 0.5;     % 耦合参数 - 论文值
params.k1 = 5;         % 位置增益 - 论文值
params.k2 = 2;         % 速度增益 - 论文值
params.kq = 5;         % 缆绳方向增益 - 论文值
params.komega = 2;     % 缆绳角速度增益 - 论文值
params.kr = 180;       % 姿态增益 - 论文值
params.kpsi = 1;       % Yaw gain (论文未明确给出，保持默认)

% ========== Adaptive Parameters (论文 Section VI) ==========
params.hq = 10;        % 缆绳自适应 - 论文值
params.homega = 1;     % 角速度自适应 - 论文值
params.hr = 30;        % 姿态自适应 - 论文值
params.hdL = 0.5;      % 载荷扰动估计 - 论文值
params.hdi = 0.2;      % 无人机扰动估计 - 论文值

% ========== Disturbance Bounds (for projection) ==========
% 论文中使用的扰动界: dL_max 和 di_max
% 论文实际扰动: dL = 0.1*mL*g*[2/3,2/3,1/3], di = 0.1*mi*g*[2/3,2/3,1/3]
% 但我们先去除扰动验证稳定性
params.dL_max = 0.1 * params.mL * params.g;  % 约0.059 N
params.di_max = 0.1 * params.mi * params.g;  % 约0.206 N

% ========== Numerical Safety / Regularization ==========
params.force_alloc_reg = 1e-2;   % Regularization for QQt inversion in force allocation
params.omega_cmd_limit = 30;     % Cap commanded angular rate to avoid solver step collapse (rad/s)

% ========== True Disturbances (for simulation) ==========
% 论文 Section VI 使用: dL = 0.1*mL*g*[2/3,2/3,1/3]', di = 0.1*mi*g*[2/3,2/3,1/3]'
% 按你的要求，先去除扰动
params.d_L_true = zeros(3, 1);
params.d_i_true = zeros(3, params.n);

% 如需添加论文中的扰动，取消下面两行注释：
% params.d_L_true = 0.1 * params.mL * params.g * [2/3; 2/3; 1/3];
% params.d_i_true = 0.1 * params.mi * params.g * repmat([2/3; 2/3; 1/3], 1, params.n);

% Initial Conditions (handled in init_state)
end
