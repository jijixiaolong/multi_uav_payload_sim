function y = sat(x)
% SAT Saturation function (sigmoid)
%   y = sat(x) returns the element-wise tanh of vector x.
%   As defined in the paper: sigma(x) := [tanh(x1), tanh(x2), tanh(x3)]^T

y = tanh(x);
end
