function x = pack_state(pL, vL, R, Omega, q, omega, dL_hat, d_hat)
% PACK_STATE Pack individual variables into the state vector x
%
%   Inputs:
%   pL: 3x1 payload position
%   vL: 3x1 payload velocity
%   R: 3x3xn quadrotor attitudes
%   Omega: 3xn quadrotor angular velocities
%   q: 3xn cable directions
%   omega: 3xn cable angular velocities
%   dL_hat: 3x1 payload disturbance estimate
%   d_hat: 3xn quadrotor disturbance estimates
%
%   Output:
%   x: State vector

n = size(R, 3);
x_cell = cell(1, 2 + 4*n + 1 + n);

% Payload
x_cell{1} = pL;
x_cell{2} = vL;

curr = 3;
% Quadrotors and Cables
for i = 1:n
    x_cell{curr} = reshape(R(:,:,i), 9, 1); curr = curr + 1;
    x_cell{curr} = Omega(:,i); curr = curr + 1;
    x_cell{curr} = q(:,i); curr = curr + 1;
    x_cell{curr} = omega(:,i); curr = curr + 1;
end

% Disturbances
x_cell{curr} = dL_hat; curr = curr + 1;
for i = 1:n
    x_cell{curr} = d_hat(:,i); curr = curr + 1;
end

x = vertcat(x_cell{:});
end
