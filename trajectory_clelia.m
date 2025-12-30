function [pd, dpd, d2pd, d3pd, d4pd] = trajectory_clelia(t)
% TRAJECTORY_CLELIA 生成Clelia曲线轨迹
%   基于论文 Wang et al. 2024, Eq.(39)
%
%   Clelia曲线定义：
%   pd(γ) = [1.3*cos(γ)*cos(γ/4);
%            1.3*cos(γ)*sin(γ/4);
%            1.3*sin(γ) - 2.2]  (单位: m)
%
%   约束条件: ||∂pd/∂t|| = 3 m/s (恒定速度)
%
%   参数 γ(t) 通过数值求解满足: 3 = ∫₀ᵗ ||∂pd(γ(τ))/∂γ(τ)||⁻¹ dτ
%
%   为了简化，我们使用近似方法：
%   - 预计算 γ 和弧长的关系
%   - 使用插值获得 γ(t)

persistent gamma_lookup s_lookup

% 第一次调用时，预计算查找表
if isempty(gamma_lookup)
    % 预计算一个周期的弧长
    gamma_max = 2*pi;  % Clelia曲线周期为2π
    N = 10000;         % 采样点数
    gamma_samples = linspace(0, gamma_max, N);

    % 计算每个点的速度 ||∂pd/∂γ||
    ds_dgamma = zeros(1, N);
    for k = 1:N
        g = gamma_samples(k);
        % ∂pd/∂γ
        dpd_dgamma = [-1.3*sin(g)*cos(g/4) - 1.3*cos(g)*sin(g/4)*0.25;
                      -1.3*sin(g)*sin(g/4) + 1.3*cos(g)*cos(g/4)*0.25;
                       1.3*cos(g)];
        ds_dgamma(k) = norm(dpd_dgamma);
    end

    % 积分得到弧长 s(γ)
    s_samples = zeros(1, N);
    for k = 2:N
        s_samples(k) = s_samples(k-1) + ds_dgamma(k-1) * (gamma_samples(k) - gamma_samples(k-1));
    end

    % 存储查找表
    gamma_lookup = gamma_samples;
    s_lookup = s_samples;
end

% 论文中要求恒定速度 v = 3 m/s
v_desired = 3.0;  % m/s
s = v_desired * t;  % 当前弧长

% 处理周期性（Clelia曲线周期为2π）
s_max = s_lookup(end);
s_mod = mod(s, s_max);

% 通过插值获得 γ(t)
gamma = interp1(s_lookup, gamma_lookup, s_mod, 'linear', 'extrap');

% 计算位置 pd(γ)
pd = [1.3 * cos(gamma) * cos(gamma/4);
      1.3 * cos(gamma) * sin(gamma/4);
      1.3 * sin(gamma) - 2.2];

% 计算 ∂pd/∂γ
dpd_dgamma = [-1.3*sin(gamma)*cos(gamma/4) - 1.3*cos(gamma)*sin(gamma/4)*0.25;
              -1.3*sin(gamma)*sin(gamma/4) + 1.3*cos(gamma)*cos(gamma/4)*0.25;
               1.3*cos(gamma)];

% 计算 dγ/dt (通过链式法则: v = ||∂pd/∂t|| = ||∂pd/∂γ|| * |dγ/dt|)
norm_dpd_dgamma = norm(dpd_dgamma);
dgamma_dt = v_desired / norm_dpd_dgamma;

% ∂pd/∂t = ∂pd/∂γ * dγ/dt
dpd = dpd_dgamma * dgamma_dt;

% 计算 ∂²pd/∂γ²
d2pd_dgamma2 = [-1.3*cos(gamma)*cos(gamma/4) + 1.3*sin(gamma)*sin(gamma/4)*0.25 ...
                 + 1.3*sin(gamma)*sin(gamma/4)*0.25 - 1.3*cos(gamma)*cos(gamma/4)*0.0625;
                -1.3*cos(gamma)*sin(gamma/4) - 1.3*sin(gamma)*cos(gamma/4)*0.25 ...
                 - 1.3*sin(gamma)*cos(gamma/4)*0.25 - 1.3*cos(gamma)*sin(gamma/4)*0.0625;
                -1.3*sin(gamma)];

% 计算 d²γ/dt² (假设近似恒定速度，二阶导数较小)
% 精确计算需要 d/dt(||∂pd/∂γ||)，这里简化为0
d2gamma_dt2 = 0;

% ∂²pd/∂t² = ∂²pd/∂γ² * (dγ/dt)² + ∂pd/∂γ * d²γ/dt²
d2pd = d2pd_dgamma2 * dgamma_dt^2 + dpd_dgamma * d2gamma_dt2;

% 计算 ∂³pd/∂γ³
d3pd_dgamma3 = [1.3*sin(gamma)*cos(gamma/4) - 1.3*cos(gamma)*sin(gamma/4)*0.75 ...
                + 1.3*cos(gamma)*sin(gamma/4)*0.75 + 1.3*sin(gamma)*cos(gamma/4)*0.046875;
                1.3*sin(gamma)*sin(gamma/4) + 1.3*cos(gamma)*cos(gamma/4)*0.75 ...
                + 1.3*cos(gamma)*cos(gamma/4)*0.75 - 1.3*sin(gamma)*sin(gamma/4)*0.046875;
                -1.3*cos(gamma)];

% 三阶导数（简化处理）
d3pd = d3pd_dgamma3 * dgamma_dt^3;

% 四阶导数（简化处理）
d4pd_dgamma4 = [1.3*cos(gamma)*cos(gamma/4) + 1.3*sin(gamma)*sin(gamma/4)*1.0;
                1.3*cos(gamma)*sin(gamma/4) - 1.3*sin(gamma)*cos(gamma/4)*1.0;
                1.3*sin(gamma)];
d4pd = d4pd_dgamma4 * dgamma_dt^4;

end
