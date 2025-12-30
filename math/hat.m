function S = hat(x)
% HAT Map a vector to a skew-symmetric matrix
%   S = hat(x) returns the 3x3 skew-symmetric matrix corresponding to the
%   3x1 vector x.
%
%   x = [x1; x2; x3]
%   S = [  0  -x3   x2;
%         x3    0  -x1;
%        -x2   x1    0 ];

S = [  0   -x(3)  x(2);
    x(3)   0   -x(1);
    -x(2)  x(1)   0 ];
end
