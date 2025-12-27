function dvL = payload_dyn(x, params, f_q, d_q, d_L)
% PAYLOAD_DYN Payload dynamics (Eq 8)
%   dvL = M^-1 * (sum(f_iq + d_iq - m_i*l_i*|omega_i|^2*q_i) + d_L) + g*e3

n = params.n;
[pL, vL, R, Omega, q, omega, ~, ~] = unpack_state(x, n);

sum_terms = zeros(3, 1);
M = params.mL * eye(3);

for i = 1:n
    term_i = f_q(:,i) + d_q(:,i) - params.mi * params.li * norm(omega(:,i))^2 * q(:,i);
    sum_terms = sum_terms + term_i;

    M = M + params.mi * (q(:,i) * q(:,i)');
end

dvL = M \ (sum_terms + d_L) + params.g * [0;0;1];
end
