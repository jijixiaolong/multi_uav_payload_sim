function [f_dL, e, ev] = payload_ctrl(params, pd, dpd, d2pd, ~, pL, vL, ~, ~, ~)
% PAYLOAD_CTRL Payload outer loop control (Eq 14)
%   Calculates the desired force f_dL for the payload.
%
%   Inputs:
%   params: Control parameters (k1, k2, beta, etc.)
%   pd, dpd, d2pd: Desired trajectory and derivatives
%   M: Precomputed mass matrix
%   pL, vL: Payload position and velocity
%   q: Normalized cable directions (3xn)
%   dL_hat: Payload disturbance estimate
%   d_hat: Quadrotor disturbance estimates (3xn)
%
%   Outputs:
%   f_dL: Desired payload force
%   e: Coupling error
%   ev: Velocity error

% Error definitions
ep = pL - pd;
ev = vL - dpd;
e = params.k1 * (ep + params.beta * ev);

% Control Law (Eq 14) - 严格按论文公式
sat_e = sat(e);
f_dL = sat_e ...                                        % σ(e) 位置误差反馈
     + params.k2 * sat(params.beta * sat_e + ev) ...    % k2·σ(β·σ(e) + ev) 速度误差反馈
     + params.g * [0;0;1] ...                           % +g·c3 重力项（论文原文）
     - d2pd;                                            % -p̈_d 前馈
% 注：扰动补偿由自适应估计器处理，暂不加入
end
