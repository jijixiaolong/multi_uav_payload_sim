function plot_input(t, x, params)
% PLOT_INPUT Plot inputs (Thrust and Omega)
%   Note: Inputs are not stored in state x directly (Omega is, but T is not).
%   To plot T, we would need to re-calculate it or store it.
%   For now, we just plot Omega from state.

n = params.n;
len = length(t);

Omega_norm = zeros(n, len);

for k = 1:len
    [~, ~, ~, Omega_k, ~, ~, ~, ~] = unpack_state(x(k,:)', n);
    for i = 1:n
        Omega_norm(i,k) = norm(Omega_k(:,i));
    end
end

figure;
plot(t, Omega_norm');
title('Quadrotor Angular Velocity Norm ||\Omega_i||');
legend('UAV 1', 'UAV 2', 'UAV 3');
grid on;
end
