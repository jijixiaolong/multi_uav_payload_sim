function [pL, vL, R, q, omega, dL_hat, d_hat] = unpack_state(x, n)
% UNPACK_STATE Unpack the state vector x into individual variables
%
%   x: State vector (column vector)
%   n: Number of quadrotors
%
%   State structure:
%   x = [pL; vL;
%        R1(:); q1; omega1;
%        ...
%        Rn(:); qn; omegan;
%        dL_hat; d1_hat; ...; dn_hat]
%
%   Outputs:
%   pL: 3x1 payload position
%   vL: 3x1 payload velocity
%   R: 3x3xn quadrotor attitudes
%   q: 3xn cable directions
%   omega: 3xn cable angular velocities
%   dL_hat: 3x1 payload disturbance estimate
%   d_hat: 3xn quadrotor disturbance estimates

% Payload
pL = x(1:3);
vL = x(4:6);

% 预分配
R = zeros(3,3,n);
q = zeros(3,n);
omega = zeros(3,n);

% Quadrotors and Cables
idx = 7;
for i = 1:n
    R(:,:,i) = reshape(x(idx:idx+8), 3, 3);
    idx = idx + 9;
    q(:,i) = x(idx:idx+2);
    idx = idx + 3;
    omega(:,i) = x(idx:idx+2);
    idx = idx + 3;
end

% Disturbances
dL_hat = x(idx:idx+2);
idx = idx + 3;
d_hat = zeros(3,n);
for i = 1:n
    d_hat(:,i) = x(idx:idx+2);
    idx = idx + 3;
end
end
