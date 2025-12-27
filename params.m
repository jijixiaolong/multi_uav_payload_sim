function params = params()
% PARAMS Define simulation parameters
%   Returns a struct with all physical parameters and control gains.

% Physical Parameters
params.mL = 0.06; % Payload mass (kg)
params.mi = 0.21; % Quadrotor mass (kg)
params.li = 0.6;  % Cable length (m)
params.g = 9.81;  % Gravity (m/s^2)
params.n = 3;     % Number of quadrotors

% Control Gains
params.beta = 0.5;
params.k1 = 5;
params.k2 = 2;
params.kq = 5;
params.komega = 2; % k_omega
params.kr = 180;
params.kpsi = 1; % Yaw gain (assumed)

% Adaptive Parameters
params.hq = 10;
params.homega = 1;
params.hr = 30;
params.hdL = 0.5;
params.hdi = 0.2;

% Disturbance Bounds (for projection)
params.dL_max = 0.2; % Assumed
params.di_max = 0.2; % Assumed

% True Disturbances (for simulation)
% dL = 0.1 * mL * g * [2/3; 2/3; 1/3]
params.d_L_true = 0.1 * params.mL * params.g * [2/3; 2/3; 1/3];

% di = 0.1 * mi * g * [2/3; 2/3; 1/3]
d_i_vec = 0.1 * params.mi * params.g * [2/3; 2/3; 1/3];
params.d_i_true = repmat(d_i_vec, 1, params.n);

% Initial Conditions (handled in init_state)
end
