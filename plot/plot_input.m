function plot_input(t, x, params)
% PLOT_INPUT Plot inputs (Thrust and Omega)
%   Re-calculates control inputs from state trajectory.

n = params.n;
len = length(t);

Omega_norm = zeros(n, len);
T_cmd = zeros(n, len);
f_dL_all = zeros(3, len);
f_qdi_norm = zeros(n, len);
f_di_perp_norm = zeros(n, len);

for k = 1:len
    % Unpack state
    x_k = x(k,:)';
    [pL, vL, R, q, omega, dL_hat, d_hat] = unpack_state(x_k, n);

    % 归一化 q
    q_norms = sqrt(sum(q.^2, 1));
    q = q ./ q_norms;

    % 计算矩阵 M
    M = params.mL * eye(3);
    for i = 1:n
        M = M + params.mi * (q(:,i) * q(:,i)');
    end

    % Trajectory
    [pd, dpd, d2pd, d3pd, d4pd] = trajectory(t(k));

    % Re-calculate Control (使用新接口)
    % 1. Payload Position Control
    [f_dL, e, ev] = payload_ctrl(params, pd, dpd, d2pd, M, pL, vL, q, dL_hat, d_hat);
    f_dL_all(:,k) = f_dL;

    % 2. Force Allocation
    f_qdi = force_allocation(params, f_dL, q, omega);
    f_qdi_norm(:,k) = sqrt(sum(f_qdi.^2, 1));

    % 3. Cable Configuration Control
    [f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i] = cable_ctrl(params, f_dL, e, ev, dpd, d2pd, d3pd, q, omega, d_hat);
    f_di_perp_norm(:,k) = sqrt(sum(f_di_perp.^2, 1));

    % 4. Attitude Control
    [Omega_cmd, T] = attitude_ctrl(params, f_qdi, f_di_perp, omega_di, dot_omega_di, e_qi, e_omega_i, ev, e, M, R, q, omega);

    for i = 1:n
        Omega_norm(i,k) = norm(Omega_cmd(:,i));
        T_cmd(i,k) = T(i);
    end
end

figure;
subplot(2,1,1);
plot(t, T_cmd');
title('Commanded Thrust T_i');
legend('UAV 1', 'UAV 2', 'UAV 3');
grid on;

subplot(2,1,2);
plot(t, Omega_norm');
title('Commanded Angular Velocity Norm ||\Omega_i||');
legend('UAV 1', 'UAV 2', 'UAV 3');
grid on;

% 控制器中间量（力）曲线
figure;
subplot(3,1,1);
plot(t, f_dL_all');
title('Payload Desired Force f_{dL}');
legend('x', 'y', 'z');
xlabel('Time [s]');
ylabel('N');
grid on;

subplot(3,1,2);
plot(t, f_qdi_norm');
title('Parallel Cable Forces ||f_{qdi}||');
legend('UAV 1', 'UAV 2', 'UAV 3');
xlabel('Time [s]');
ylabel('N');
grid on;

subplot(3,1,3);
plot(t, f_di_perp_norm');
title('Perpendicular Cable Forces ||f_{d\perp i}||');
legend('UAV 1', 'UAV 2', 'UAV 3');
xlabel('Time [s]');
ylabel('N');
grid on;
end
