function dR = attitude_dyn(R, Omega)
% ATTITUDE_DYN Attitude kinematics R_dot = R * S(Omega)
%   dR = R * hat(Omega)

n = size(R, 3);
dR = zeros(3,3,n);

for i = 1:n
    dR(:,:,i) = R(:,:,i) * hat(Omega(:,i));
end
end
