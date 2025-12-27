function [f_qdi, u] = force_allocation(x, params, f_dL)
% FORCE_ALLOCATION Force allocation (Eq 17, 18)
%   Calculates the desired parallel force component f_qdi for each quadrotor.
%
%   Inputs:
%   x: State vector
%   params: Parameters
%   f_dL: Desired payload force
%
%   Outputs:
%   f_qdi: 3xn desired parallel forces
%   u: Auxiliary force allocation

n = params.n;
[~, ~, ~, ~, q, omega, ~, ~] = unpack_state(x, n);

% Auxiliary force u (Eq 18)
% u = mL * Q' * (Q*Q')^-1 * f_dL
Q = q; % 3xn

% Check rank of Q
if rank(Q) < 3
    warning('force_allocation: Q is singular (rank < 3). Pseudo-inverse used.');
end

% u vector (nx1)
% Note: The formula in paper is u = mL * Q' * (Q*Q')^-1 * f_dL
% This distributes the force required for the payload mass mL.
% Wait, Eq 16 says sum(...) = -M * f_dL
% And Problem 2 says sum(...) = -M * f_dL
% And sum(...) = -mL * f_dL - sum(mi * qi * qi' * f_dL) + sum(mi * li * |w|^2 * qi) - sum(u_i * qi)
% So we need sum(u_i * qi) = mL * f_dL
% The solution u = mL * Q' * (Q*Q')^-1 * f_dL satisfies Q * u = mL * f_dL.

u = params.mL * Q' * ((Q * Q') \ f_dL);

f_qdi = zeros(3, n);
for i = 1:n
    term1 = params.mi * params.li * norm(omega(:,i))^2 * q(:,i);
    term2 = -params.mi * (q(:,i) * q(:,i)') * f_dL;
    term3 = -u(i) * q(:,i);

    f_qdi(:,i) = term1 + term2 + term3;
end
end
