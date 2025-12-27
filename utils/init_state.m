function x0 = init_state(n)
% INIT_STATE Initialize the state vector
%   x0 = init_state(n) returns the initial state vector for n quadrotors.
%   Values are hardcoded based on the paper's simulation section or defaults.

% Payload
pL0 = [0.5; 0; -1]; % From paper
vL0 = [0; 0; 0];

% Quadrotors and Cables
R0 = eye(3);
Omega0 = [0; 0; 0];
omega0 = [0; 0; 0];

% Calculate initial q based on paper:
% qdi = [cos(psi_di)sin(theta_d), sin(psi_di)sin(theta_d), cos(theta_d)]
% theta_d = 40 deg, psi_di = (i-2)*60 deg
theta_d = deg2rad(40);

R_all = zeros(3,3,n);
Omega_all = zeros(3,n);
q_all = zeros(3,n);
omega_all = zeros(3,n);

for i = 1:n
    psi_di = deg2rad((i-2)*60);
    q_di = [cos(psi_di)*sin(theta_d); sin(psi_di)*sin(theta_d); cos(theta_d)];

    R_all(:,:,i) = R0;
    Omega_all(:,i) = Omega0;
    q_all(:,i) = q_di; % Start at desired configuration
    omega_all(:,i) = omega0;
end

% Disturbances (estimates start at 0)
dL_hat0 = [0; 0; 0];
d_hat0 = zeros(3,n);

x0 = pack_state(pL0, vL0, R_all, Omega_all, q_all, omega_all, dL_hat0, d_hat0);
end
