function x = vee(S)
% VEE Map a skew-symmetric matrix to a vector
%   x = vee(S) returns the 3x1 vector corresponding to the 3x3
%   skew-symmetric matrix S.
%
%   S = [  0  -x3   x2;
%         x3    0  -x1;
%        -x2   x1    0 ];
%   x = [x1; x2; x3]

x = [S(3,2); S(1,3); S(2,1)];
end
