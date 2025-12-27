function domega = cable_dyn(x, params, dvL, f, d)
% CABLE_DYN Cable dynamics (Eq 9)
%   domega_i = (1/l_i) * S(q_i) * (dvL - g*e3) - (1/(m_i*l_i)) * S(q_i) * (f_i + d_i)

n = params.n;
[~, ~, ~, ~, q, ~, ~, ~] = unpack_state(x, n);

domega = zeros(3, n);

for i = 1:n
    term1 = (1/params.li) * hat(q(:,i)) * (dvL - params.g * [0;0;1]);
    term2 = (1/(params.mi * params.li)) * hat(q(:,i)) * (f(:,i) + d(:,i));

    domega(:,i) = term1 - term2;
end
end
