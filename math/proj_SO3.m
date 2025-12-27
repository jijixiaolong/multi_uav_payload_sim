function R_proj = proj_SO3(M)
% PROJ_SO3 Project a 3x3 matrix onto SO(3)
%   R_proj = proj_SO3(M) returns the closest rotation matrix to M
%   in the Frobenius norm sense.
%   This is typically solved using SVD. M = U * Sigma * V'.
%   R_proj = U * diag([1, 1, det(U*V')]) * V'.

[U, ~, V] = svd(M);
R_proj = U * diag([1, 1, det(U*V')]) * V';
end
