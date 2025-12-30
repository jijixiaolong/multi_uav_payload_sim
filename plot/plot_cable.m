function plot_cable(t, x, params)
% PLOT_CABLE Plot cable direction errors

n = params.n;
len = length(t);

% We need q_di to calculate error.
% Re-calculate q_di (same logic as init_state/cable_ctrl)
theta_d = deg2rad(40);
q_di_all = zeros(3, n);
for i = 1:n
    psi_di = deg2rad((i-2)*60);
    q_di_all(:,i) = [cos(psi_di)*sin(theta_d); sin(psi_di)*sin(theta_d); cos(theta_d)];
end

eq_norm = zeros(n, len);

for k = 1:len
    [~, ~, ~, q_k, ~, ~, ~] = unpack_state(x(k,:)', n);
    for i = 1:n
        eq_norm(i,k) = norm(q_k(:,i) - q_di_all(:,i));
    end
end

figure;
plot(t, eq_norm');
title('Cable Direction Error Norm ||q_i - q_{di}||');
legend('Cable 1', 'Cable 2', 'Cable 3'); % Assuming n=3
grid on;
end
