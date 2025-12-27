function plot_disturb(t, x, params)
% PLOT_DISTURB Plot disturbance estimation errors

n = params.n;
len = length(t);

dL_err_norm = zeros(1, len);
di_err_norm = zeros(n, len);

dL_true = params.d_L_true;
di_true = params.d_i_true;

for k = 1:len
    [~, ~, ~, ~, ~, ~, dL_hat_k, d_hat_k] = unpack_state(x(k,:)', n);

    dL_err_norm(k) = norm(dL_hat_k - dL_true);

    for i = 1:n
        di_err_norm(i,k) = norm(d_hat_k(:,i) - di_true(:,i));
    end
end

figure;
subplot(2,1,1);
plot(t, dL_err_norm);
title('Payload Disturbance Estimation Error Norm');
grid on;

subplot(2,1,2);
plot(t, di_err_norm');
title('Quadrotor Disturbance Estimation Error Norm');
legend('UAV 1', 'UAV 2', 'UAV 3');
grid on;
end
