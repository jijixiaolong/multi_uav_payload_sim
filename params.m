function params = params()
% PARAMS Define simulation parameters
%   Returns a struct with all physical parameters and control gains.

% Physical Parameters (重载荷系统 - 更稳定)
params.mL = 0.4;  % Payload mass (kg)
params.mi = 1.5;  % Quadrotor mass (kg)
params.li = 0.6;  % Cable length (m)
params.g = 9.81;  % Gravity (m/s^2)
params.n = 3;     % Number of quadrotors
params.theta_d = deg2rad(5); % Desired cable angle (zenith) - 稳定的基础值

% Control Gains (基于调优总结的稳定组合)
params.beta = 0.5;
params.k1 = 7;        % 位置增益
params.k2 = 4;        % 速度增益/阻尼
params.kq = 7;        % 缆绳方向
params.komega = 4;    % 缆绳角速度
params.kr = 220;      % 姿态
params.kpsi = 1;      % Yaw gain

% Adaptive Parameters (与更小θd匹配的阻尼)
params.hq = 2;       % 缆绳自适应
params.homega = 0.3;  % 角速度自适应
params.hr = 3;       % 姿态自适应
params.hdL = 0.8;     % 扰动估计
params.hdi = 0.4;     % 扰动估计

% Disturbance Bounds (for projection)
params.dL_max = 0.2; % Assumed
params.di_max = 0.2; % Assumed

% Numerical safety / regularization
params.force_alloc_reg = 1e-2;   % Regularization for QQt inversion in force allocation
params.omega_cmd_limit = 30;     % Cap commanded angular rate to avoid solver step collapse (rad/s)

% True Disturbances (for simulation) - 全部移除
params.d_L_true = zeros(3, 1);
params.d_i_true = zeros(3, params.n);

% Initial Conditions (handled in init_state)
end
