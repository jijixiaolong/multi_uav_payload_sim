function [f_dL, e, ev] = payload_ctrl(x, params, pd, dpd, d2pd)
% PAYLOAD_CTRL Payload outer loop control (Eq 14)
%   Calculates the desired force f_dL for the payload.
%
%   Inputs:
%   x: State vector
%   params: Control parameters (k1, k2, beta, etc.)
%   pd, dpd, d2pd: Desired trajectory and derivatives
%
%   Outputs:
%   f_dL: Desired payload force
%   e: Coupling error
%   ev: Velocity error

n = params.n;
[pL, vL, ~, ~, q, ~, dL_hat, d_hat] = unpack_state(x, n);

% Error definitions
ep = pL - pd;
ev = vL - dpd;
e = params.k1 * (ep + params.beta * ev);

% Disturbance compensation term
% sum(d_iq_hat)
sum_d_iq_hat = zeros(3, 1);
M = params.mL * eye(3);
for i = 1:n
    d_iq_hat = (q(:,i)' * d_hat(:,i)) * q(:,i);
    sum_d_iq_hat = sum_d_iq_hat + d_iq_hat;
    M = M + params.mi * (q(:,i) * q(:,i)');
end

% Control Law (Eq 14)
% f_dL = sigma(e) + k2*sigma(beta*sigma(e) + ev) + g*e3 - d2pd + M^-1 * (dL_hat + sum_d_iq_hat)
% Wait, Eq 14 in paper:
% f_dL = sigma(e) + k2*sigma(beta*sigma(e) + ev) + g*e3 - d2pd + M^-1 * (dL_hat + sum(d_iq_hat))
%
% Wait, looking at Eq 8: vL_dot = M^-1 * (sum(f_iq + ...) + dL) + g*e3
% If we want vL_dot to track desired dynamics, we need to invert M?
% Let's check Eq 14 carefully in formulas.md.
% f_dL := sigma(e) + ... + M^-1 * (...)
%
% But later in Problem 2: sum(...) = -M * f_dL
% So -M * f_dL is the desired force from quadrotors?
%
% Let's re-read Eq 16.
% V1_dot = ... + M^-1 * (sum(...) + f_dL)
% So f_dL is defined such that it cancels terms in V1_dot.
%
% Yes, Eq 14 defines f_dL.

term1 = sat(e);
term2 = params.k2 * sat(params.beta * sat(e) + ev);
term3 = params.g * [0;0;1];
term4 = -d2pd;
term5 = M \ (dL_hat + sum_d_iq_hat);

f_dL = term1 + term2 + term3 + term4 + term5;
end
