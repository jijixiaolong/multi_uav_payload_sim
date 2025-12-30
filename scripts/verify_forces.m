% VERIFY_FORCES - 验证力的符号和方向
clear; clc;

% NED坐标系：
% X: 北 (North)
% Y: 东 (East)
% Z: 下 (Down) - 正方向向下

% 重力在NED中的表示
g = 9.81;
gravity_NED = [0; 0; g];  % 向下为正

fprintf('=== NED坐标系验证 ===\n');
fprintf('重力向量 (NED): [%.2f, %.2f, %.2f] m/s²\n', gravity_NED);
fprintf('说明: Z=%.2f > 0，表示向下加速\n\n', gravity_NED(3));

% 悬停所需的推力加速度
% 要抵消重力，推力加速度应该是 -gravity，即向上
thrust_accel_needed = -gravity_NED;
fprintf('悬停所需推力加速度: [%.2f, %.2f, %.2f] m/s²\n', thrust_accel_needed);
fprintf('说明: Z=%.2f < 0，表示向上加速\n\n', thrust_accel_needed(3));

% 根据payload_ctrl的公式：
% f_dL = σ(e) + k2·σ(...) + g·c3 - d2pd
% 在悬停时：e=0, ev=0, d2pd=0
% 所以 f_dL = g * [0;0;1]
f_dL_hover = g * [0; 0; 1];
fprintf('payload_ctrl在悬停时的输出:\n');
fprintf('f_dL = [%.2f, %.2f, %.2f] m/s²\n', f_dL_hover);

% 这个f_dL会被用在force_allocation中
% Eq. 18: u = mL * Q^T * (Q*Q^T)^(-1) * f_dL
% 如果f_dL = [0;0;g]，那么期望的是向下的加速度？

fprintf('\n问题：\n');
fprintf('如果f_dL = [0;0;%.2f]，这表示期望**向下**加速%.2f m/s²\n', g, g);
fprintf('但悬停时应该期望**向上**加速%.2f m/s²来抵消重力！\n', g);
fprintf('\n结论：payload_ctrl的符号可能有误！\n');
fprintf('正确的应该是: f_dL = -g * [0;0;1] = [0;0;%.2f]\n', -g);
