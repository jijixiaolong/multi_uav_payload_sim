function q_proj = proj_S2(q)
% PROJ_S2 Project a vector onto the unit sphere S2
%   q_proj = proj_S2(q) returns the normalized vector q / ||q||.
%   If norm(q) is 0, it returns [0;0;1] (or handles error).

norm_q = norm(q);
if norm_q < 1e-6
    warning('proj_S2: Zero vector input, returning [0;0;1]');
    q_proj = [0; 0; 1];
else
    q_proj = q / norm_q;
end
end
