function f_qdi = force_allocation(params, f_dL, q, omega)
% FORCE_ALLOCATION 力分配 (论文公式 Eq 17, 18)
%   将负载期望力 f_dL 分配到各缆绳的平行分量 f_qdi
%
%   输入:
%   params: 参数结构体 (mL, mi, li)
%   f_dL:   负载期望力 (3×1)，来自 payload_ctrl
%   q:      缆绳方向单位向量 (3×n)
%   omega:  缆绳角速度 (3×n)
%
%   输出:
%   f_qdi:  各缆绳期望平行力 (3×n)

% 辅助力分配 u (Eq 18)
% u = mL * Q^T * (Q*Q^T)^(-1) * f_dL
% 用于确保所有缆绳力的合力等于期望力 f_dL
reg = params.force_alloc_reg;
QQt = q * q' + reg * eye(3);            % Q*Q^T + 正则化防止病态
u = params.mL * q' * (QQt \ f_dL);      % n×1: 各缆绳的辅助力大小

% 期望平行力 f_qdi (Eq 17) - 向量化计算
% 公式: f_qdi = mi*li*||ω_i||²*q_i - mi*(q_i^T*f_dL)*q_i - u_i*q_i
% 提取公因子: f_qdi = coeff_i * q_i
%   其中 coeff_i = mi*li*||ω_i||² - mi*(q_i^T*f_dL) - u_i

omega_norm_sq = sum(omega.^2, 1);       % 1×n: 各缆绳角速度模方 ||ω_i||²
f_dL_proj = sum(q .* f_dL, 1);          % 1×n: f_dL 在各缆绳方向的投影 q_i^T*f_dL

% 合并系数 (1×n)
coeff = params.mi * params.li * omega_norm_sq ...   % 离心力补偿项
      - params.mi * f_dL_proj ...                   % 负载力分配项
      - u';                                         % 辅助力修正项

% 各缆绳期望平行力 = 系数 × 缆绳方向
f_qdi = q .* coeff;                     % 3×n: 逐列相乘
end
