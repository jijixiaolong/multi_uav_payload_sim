function plot_payload(t, x, params)
% PLOT_PAYLOAD Plot payload position and velocity errors

n = params.n;
len = length(t);
pL = zeros(3, len);
vL = zeros(3, len);
pd = zeros(3, len);
dpd = zeros(3, len);

for k = 1:len
    [pL_k, vL_k, ~, ~, ~, ~, ~, ~] = unpack_state(x(k,:)', n);
    pL(:,k) = pL_k;
    vL(:,k) = vL_k;

    [pd_k, dpd_k, ~, ~, ~] = trajectory(t(k));
    pd(:,k) = pd_k;
    dpd(:,k) = dpd_k;
end

ep = pL - pd;
ev = vL - dpd;

figure;
subplot(2,1,1);
plot(t, ep');
title('Payload Position Error');
legend('x', 'y', 'z');
grid on;

subplot(2,1,2);
plot(t, ev');
title('Payload Velocity Error');
legend('vx', 'vy', 'vz');
grid on;
end
