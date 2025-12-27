function dx = system_dynamics(t, x, params)
% SYSTEM_DYNAMICS Complete state differential dx = f(x,u)
%
%   Inputs:
%   t: Time
%   x: State vector
%   params: Parameter structure
%
%   Output:
%   dx: State derivative

% Unpack state
n = params.n;
[pL, vL, R, Omega, q, omega, dL_hat, d_hat] = unpack_state(x, n);

% Get control inputs
% Note: In a real simulation, control would be calculated here or passed in.
% For now, we assume a control function exists or we calculate it here.
% To keep it modular, we'll call the control module.
% However, since ode45 requires a function of (t,x), we need to compute
% control inside here.

% Desired trajectory
[pd, dpd, d2pd, d3pd, d4pd] = trajectory(t);

% Control Law Calculation
% 1. Payload Control -> f_dL
% 2. Force Allocation -> f_qdi
% 3. Cable Control -> f_di_perp
% 4. Attitude Control -> T_i, Omega_i (This is the input to dynamics)

% Ideally, we should have a separate function `controller(t, x, params)`
% that returns the actual inputs T and Omega_ref (or moments).
% But the paper's control law gives Omega_i directly as a kinematic input
% for attitude, or we assume we track Omega_i perfectly?
% The paper says: "controlling the vehicle's attitude through angular velocity actuation"
% Eq (1c) is R_dot = R * S(Omega).
% The control law (Eq 33) designs Omega_i.
% So Omega_i is the input.

% Call controller to get Omega and Thrust
[T, Omega_cmd, dL_hat_dot, d_hat_dot] = controller_wrapper(t, x, params, pd, dpd, d2pd, d3pd, d4pd);

% Dynamics Calculation

% 1. Payload Dynamics (Eq 8)
% Need to calculate f_iq = (q_i' * f_i) * q_i
% f_i = -T_i * R_i * e3

f = zeros(3, n);
f_q = zeros(3, n);
sum_terms = zeros(3, 1);
M = params.mL * eye(3);

for i = 1:n
    f(:,i) = -T(i) * R(:,:,i) * [0;0;1];
    f_q(:,i) = (q(:,i)' * f(:,i)) * q(:,i);

    % Disturbance on quadrotor (simulated)
    % For simulation, we need actual disturbance.
    % Let's assume constant disturbance defined in params or zero.
    d_i_true = params.d_i_true(:,i);
    d_iq = (q(:,i)' * d_i_true) * q(:,i);

    term_i = f_q(:,i) + d_iq - params.mi * params.li * norm(omega(:,i))^2 * q(:,i);
    sum_terms = sum_terms + term_i;

    M = M + params.mi * (q(:,i) * q(:,i)');
end

d_L_true = params.d_L_true;

dvL = M \ (sum_terms + d_L_true) + params.g * [0;0;1];
dpL = vL;

% 2. Cable Dynamics (Eq 9)
domega = zeros(3, n);
dq = zeros(3, n);

for i = 1:n
    d_i_true = params.d_i_true(:,i);

    term1 = (1/params.li) * hat(q(:,i)) * (dvL - params.g * [0;0;1]);
    term2 = (1/(params.mi * params.li)) * hat(q(:,i)) * (f(:,i) + d_i_true);

    domega(:,i) = term1 - term2;
    dq(:,i) = hat(omega(:,i)) * q(:,i);
end

% 3. Attitude Dynamics (Eq 1c)
dR = zeros(3,3,n);
for i = 1:n
    % We use the commanded Omega as the actual Omega (assuming perfect low-level control)
    % or we could add actuator dynamics. The paper seems to treat Omega as input.
    dR(:,:,i) = R(:,:,i) * hat(Omega_cmd(:,i));
end

% 4. Disturbance Estimation Dynamics (Eq 35, 36)
% These are calculated in the controller

% Pack derivatives
dx = pack_state(dpL, dvL, dR, zeros(3,n), dq, domega, dL_hat_dot, d_hat_dot);

% Note: pack_state expects Omega, but here we need dOmega.
% Since Omega is an input, dOmega is not defined in the state.
% However, our state vector includes Omega?
% Let's check unpack_state.
% x includes Omega.
% If Omega is an input, we shouldn't integrate it.
% But usually in these sims, we might have dynamics for Omega.
% The paper treats Omega as control input.
% So in the state vector, we might not need Omega if it's purely input.
% BUT, init_state included it.
% If we keep it in state, we need its derivative.
% Since it's an input, we can set dOmega = 0 or use a filter.
% For now, let's set dOmega = 0 (instantaneous tracking) or remove it from state.
% Given the structure, let's assume dOmega = 0 for now, effectively holding it constant between steps?
% No, ode45 integrates.
% If Omega is input, it shouldn't be in the state vector x that is integrated.
% It should be an algebraic variable.
% However, I already defined the state vector to include it.
% Let's modify pack_state/unpack_state to NOT include Omega if it's an input?
% Or just set dOmega = 0 and ignore the integrated value, always overwriting it with control?
% Better: The state should probably NOT include Omega if it's a control input.
% But let's stick to the current structure and set dOmega = 0.
% Actually, wait. If Omega is input, why is it in state?
% Maybe for visualization?
% Let's set dOmega = 0.

% Re-pack with dOmega = 0
dx = pack_state(dpL, dvL, dR, zeros(3,n), dq, domega, dL_hat_dot, d_hat_dot);

end

function [T, Omega_cmd, dL_hat_dot, d_hat_dot] = controller_wrapper(t, x, params, pd, dpd, d2pd, d3pd, d4pd)
% Wrapper to call the control functions
% We need to implement the control logic here or call separate files.
% To avoid circular dependencies or complex pathing, we can assume the control files are on path.

% Add control path
addpath(genpath(fullfile(pwd, '..', 'control')));
addpath(genpath(fullfile(pwd, '..', 'math')));

% Call main control function (to be implemented)
% [T, Omega_cmd, dL_hat_dot, d_hat_dot] = full_controller(t, x, params, ...);

% For now, return dummy values to allow testing dynamics
n = params.n;
T = ones(n, 1) * params.mi * 9.81; % Hover thrust approx
Omega_cmd = zeros(3, n);
dL_hat_dot = zeros(3, 1);
d_hat_dot = zeros(3, n);

% We will replace this with actual control calls later.
end
