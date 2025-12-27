function [f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(x, params, f_dL, e, ev, dpd, d2pd, d3pd)
% CABLE_CTRL Cable control (Eq 28)
%   Calculates the desired perpendicular force component f_di_perp.
%
%   Inputs:
%   x: State vector
%   params: Parameters
%   f_dL: Desired payload force (for vLn calculation)
%   e, ev: Payload errors
%   dpd, d2pd, d3pd: Trajectory derivatives
%
%   Outputs:
%   f_di_perp: 3xn desired perpendicular forces
%   omega_di: Desired angular velocities
%   dot_omega_di: Derivative of desired angular velocities (approx)
%   e_qi, e_omega_i: Errors

n = params.n;
[~, ~, ~, ~, q, omega, ~, d_hat] = unpack_state(x, n);

% Desired configuration q_di
% In this paper, q_di is given (Problem 1).
% We assume q_di is constant or slowly varying as per simulation section.
% "The desired cable-configuration ... was set as qdi = ..."
% So q_di is a function of time or constant.
% Let's assume it's constant for now as in the simulation setup,
% or we need a function `configuration(t)`.
% For now, we use the initial configuration logic from init_state.

theta_d = deg2rad(40);
q_di_all = zeros(3, n);
dq_di_all = zeros(3, n); % Assuming constant for now
d2q_di_all = zeros(3, n);

for i = 1:n
    psi_di = deg2rad((i-2)*60);
    q_di_all(:,i) = [cos(psi_di)*sin(theta_d); sin(psi_di)*sin(theta_d); cos(theta_d)];
end

% vLn (Nominal/Desired Payload Acceleration term?)
% Eq 26: dot_vLn = -sigma(e) - k2*sigma(beta*sigma(e) + ev) + d2pd
% Wait, the paper defines dot_vLn in Eq 26.
% This looks like the "desired" acceleration for the payload dynamics inversion?
dot_vLn = -sat(e) - params.k2 * sat(params.beta * sat(e) + ev) + d2pd;

% Derivative of dot_vLn is needed for dot_omega_di?
% Eq 28 uses dot_vLn.
% It also uses dot_omega_di.

f_di_perp = zeros(3, n);
omega_di = zeros(3, n);
dot_omega_di = zeros(3, n); % Need to implement derivative
e_qi = zeros(3, n);
e_omega_i = zeros(3, n);

for i = 1:n
    q_i = q(:,i);
    q_di = q_di_all(:,i);
    dq_di = dq_di_all(:,i);

    % Error
    e_qi(:,i) = q_i - q_di;

    % Desired Omega (Eq 24)
    % omega_di = S(q_di)*dq_di + (kq/hq)*S(q_i)*q_di
    omega_di(:,i) = hat(q_di) * dq_di + (params.kq / params.hq) * hat(q_i) * q_di;

    % Error Omega
    % e_omega_i = S(q_i) * (omega_i - omega_di)
    e_omega_i(:,i) = hat(q_i) * (omega(:,i) - omega_di(:,i));

    % Derivative of omega_di (dot_omega_di)
    % Need analytical derivative of Eq 24.
    % dot_omega_di = S(dq_di)*dq_di + S(q_di)*d2q_di + (kq/hq)*(S(dq_i)*q_di + S(q_i)*dq_di)
    % dq_i = S(omega_i)*q_i
    dq_i = hat(omega(:,i)) * q_i;

    term_dot_1 = hat(dq_di) * dq_di; % 0 if constant
    term_dot_2 = hat(q_di) * d2q_di_all(:,i); % 0 if constant
    term_dot_3 = (params.kq / params.hq) * (hat(dq_i) * q_di + hat(q_i) * dq_di);

    dot_omega_di(:,i) = term_dot_1 + term_dot_2 + term_dot_3;

    % Control Law (Eq 28)
    % f_di_perp = mi*li * S^2(q_i) * [ ... ]

    S_qi = hat(q_i);
    S2_qi = S_qi * S_qi;

    term_dyn_1 = (1/params.li) * S2_qi * (dot_vLn - params.g * [0;0;1]);
    term_dyn_2 = -S_qi * dot_omega_di(:,i);
    term_dyn_3 = (1/(params.mi * params.li)) * d_hat(:,i);
    term_dyn_4 = hat(dq_i) * (omega(:,i) - omega_di(:,i));
    term_fb_1 = (params.hq / params.homega) * e_qi(:,i);
    term_fb_2 = (params.komega / params.homega) * e_omega_i(:,i);

    bracket_term = term_dyn_1 + term_dyn_2 + term_dyn_3 + term_dyn_4 + term_fb_1 + term_fb_2;

    f_di_perp(:,i) = params.mi * params.li * S2_qi * bracket_term;
end
end
