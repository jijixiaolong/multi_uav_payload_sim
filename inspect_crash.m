
% Inspect crash state
addpath('utils');
addpath('dynamics');
addpath('control');
load('results/sim_fast_20251228_185020.mat');
last_x = x(end,:)';
n = p.n;
[pL, vL, R, q, omega, dL_hat, d_hat] = unpack_state(last_x, n);

disp('Last State Analysis:');
disp(['Time: ', num2str(t(end))]);
disp('Payload Position pL:'); disp(pL');
disp('Payload Velocity vL:'); disp(vL');
disp('Cable Directions q:'); disp(q);
disp('Cable Angular Velocities omega:'); disp(omega);
disp('Disturbance Estimate dL_hat:'); disp(dL_hat');

% Check for NaNs
if any(isnan(last_x))
    disp('NaNs detected in state vector!');
end

% Check for large values
if any(abs(last_x) > 1e3)
    disp('Large values detected (>1e3)!');
end
