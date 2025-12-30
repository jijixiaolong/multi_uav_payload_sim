function [dL_hat_dot, d_hat_dot] = disturbance_est(params, e, ev, e_omega_i, M, q, omega, dL_hat, d_hat)
% DISTURBANCE_EST Disturbance estimation (Eq 35, 36)
%   Calculates the derivatives of the disturbance estimates.
%
%   Inputs:
%   params: Parameters
%   e, ev: Payload errors
%   e_omega_i: Angular velocity errors (3xn)
%   M: Precomputed mass matrix
%   q: Normalized cable directions (3xn)
%   omega: Cable angular velocities (3xn)
%   dL_hat: Payload disturbance estimate
%   d_hat: Quadrotor disturbance estimates (3xn)

n = params.n;

% dV4/dev = dV1/dev = beta*sigma(e) + ev
dV1_dev = params.beta * sat(e) + ev;

% dV3/dev
homega_li = params.homega / params.li;
sum_term = zeros(3, 1);
for i = 1:n
    sum_term = sum_term + homega_li * hat(omega(:,i))' * e_omega_i(:,i);
end
dV3_dev = dV1_dev - sum_term;

% Precompute M \ dV3_dev once
M_inv_dV3_dev = M \ dV3_dev;

% dL_hat_dot (Eq 35)
y_L = params.hdL * M_inv_dV3_dev;
dL_hat_dot = proj(y_L, dL_hat, params.dL_max);

% d_hat_dot (Eq 36)
d_hat_dot = zeros(3, n);
inv_mi_li = 1 / (params.mi * params.li);
inv_li = 1 / params.li;

for i = 1:n
    dV3_de_omega_i = params.homega * e_omega_i(:,i);

    y_i = params.hdi * (inv_mi_li * dV3_de_omega_i - inv_li * q(:,i) * (q(:,i)' * M_inv_dV3_dev));
    d_hat_dot(:,i) = proj(y_i, d_hat(:,i), params.di_max);
end

end

function out = proj(y, d_hat, d_max)
% Projection operator
norm_d_sq = sum(d_hat.^2);
norm_d = sqrt(norm_d_sq);
if norm_d < d_max || (y' * d_hat <= 0)
    out = y;
else
    out = y - (y' * d_hat / norm_d_sq) * d_hat;
end
end
