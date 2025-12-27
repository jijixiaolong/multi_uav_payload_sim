function [dL_hat_dot, d_hat_dot] = disturbance_est(x, params, ev, e_omega_i)
% DISTURBANCE_EST Disturbance estimation (Eq 35, 36)
%   Calculates the derivatives of the disturbance estimates.
%
%   Inputs:
%   x: State vector
%   params: Parameters
%   ev: Velocity error
%   e_omega_i: Cable angular velocity error
%
%   Outputs:
%   dL_hat_dot: Derivative of payload disturbance estimate
%   d_hat_dot: Derivative of quadrotor disturbance estimates

n = params.n;
[~, ~, ~, ~, q, ~, dL_hat, d_hat] = unpack_state(x, n);

% dV4/dev = dV1/dev = beta*sigma(e) + ev (Approx)
% We need e. Let's assume we can calculate it or pass it.
% For now, let's use a simplified dV4/dev = ev (if e is small).
% Better: Recalculate e.
[pL, vL, ~, ~, ~, ~, ~, ~] = unpack_state(x, n);
% We need pd, dpd.
% This function needs more inputs to be accurate.
% Let's assume ev is passed, but we need e for dV4/dev.
% Let's assume dV4/dev is passed directly?
% Or calculate it.

% Let's assume we pass dV4_dev (which is dV3_dev approx?)
% Actually, dV4/dev = dV3/dev.
% And dV4/de_omega_i = dV3/de_omega_i.

% Let's recalculate dV4_dev here if we have pd.
% Since we don't have pd, we rely on inputs.
% Let's assume ev contains the necessary info or we pass dV4_dev.
% Let's change input to dV4_dev.

% But wait, the function signature in system_dynamics needs to be consistent.
% In system_dynamics, we call controller_wrapper.
% We can calculate dV4_dev in controller_wrapper and pass it.

% Let's implement the projection logic.

% Proj(y, d_hat)
% If ||d_hat|| < d_max or (||d_hat|| >= d_max and y'*d_hat <= 0)
%   return y
% else
%   return y - (y'*d_hat * d_hat) / ||d_hat||^2

% dL_hat_dot
% y_L = h_dL * M^-1 * (dV4/dev)^T
% But dV4/dev is row vector.

% Let's assume inputs are:
% dV4_dev: 3x1 vector (transposed)
% dV4_de_omega_i: 3xn matrix

% We need to update the function signature in the next step or assume inputs.
% Let's assume inputs are passed correctly.

dL_hat_dot = zeros(3, 1);
d_hat_dot = zeros(3, n);

% Placeholder return
end
