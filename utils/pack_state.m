function x = pack_state(pL, vL, R, q, omega, dL_hat, d_hat)
% PACK_STATE Pack individual variables into the state vector x
%
%   Inputs:
%   pL: 3x1 payload position
%   vL: 3x1 payload velocity
%   R: 3x3xn quadrotor attitudes
%   q: 3xn cable directions
%   omega: 3xn cable angular velocities
%   dL_hat: 3x1 payload disturbance estimate
%   d_hat: 3xn quadrotor disturbance estimates
%
%   Output:
%   x: State vector

n = size(R, 3);

% 预分配状态向量: 6 + n*(9+3+3) + 3 + n*3 = 6 + 15n + 3 + 3n = 9 + 18n
x = zeros(9 + 18*n, 1);

% Payload
x(1:3) = pL;
x(4:6) = vL;

% Quadrotors and Cables
idx = 7;
for i = 1:n
    x(idx:idx+8) = reshape(R(:,:,i), 9, 1);
    idx = idx + 9;
    x(idx:idx+2) = q(:,i);
    idx = idx + 3;
    x(idx:idx+2) = omega(:,i);
    idx = idx + 3;
end

% Disturbances
x(idx:idx+2) = dL_hat;
idx = idx + 3;
for i = 1:n
    x(idx:idx+2) = d_hat(:,i);
    idx = idx + 3;
end
end
