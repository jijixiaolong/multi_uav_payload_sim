function [Omega_cmd, T, f_di] = attitude_ctrl(params, f_qdi, f_di_perp, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~)
% ATTITUDE_CTRL 简化版本 - 理想推力跟踪
%   假设四旋翼底层姿态控制器可以完美跟踪期望推力方向
%   直接返回期望推力向量 f_di，姿态 R 从推力方向计算
%
%   输入:
%   params:     参数结构体
%   f_qdi:      缆绳方向期望力 (3×n)
%   f_di_perp:  缆绳垂直期望力 (3×n)
%   [其他输入用 ~ 占位，保持接口兼容性]
%
%   输出:
%   Omega_cmd:  角速度命令 (3×n) - 简化版本中设为零
%   T:          推力大小 (1×n)
%   f_di:       期望推力向量 (3×n) - 新增输出，用于动力学计算

n = params.n;

% 期望总推力向量 f_di = f_qdi + f_di_perp
f_di = f_qdi + f_di_perp;

% 推力大小
T = sqrt(sum(f_di.^2, 1));  % 1×n: ||f_di||

% 应用推力限制
T_max = 60;  % N, 最大推力
T_min = params.mi * params.g + (params.mL * params.g) / n;  % 最小推力
T = max(min(T, T_max), T_min);

% 角速度命令设为零（简化版本不需要控制姿态）
Omega_cmd = zeros(3, n);

end
