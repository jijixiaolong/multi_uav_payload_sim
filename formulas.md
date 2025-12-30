# 论文核心公式提取："Robust Cooperative Transportation of a Cable-Suspended Payload by Multiple Quadrotors Featuring Cable-Reconfiguration Capabilities"

## 1. 动力学 (Dynamics)

### 载荷动力学 (Payload Dynamics, Eq. 8)
$$
\dot{v}_L = M^{-1} \left( \sum_{i=1}^n (f_{iq} + d_{iq} - m_i \ell_i \|\omega_i\|^2 q_i) + d_L \right) + g c_3
$$
其中 $M = m_L I + \sum_{i=1}^n m_i q_i q_i^T$。

### 绳索动力学 (Cable Dynamics, Eq. 9)
$$
\dot{\omega}_i = \frac{1}{\ell_i} S(q_i) (\dot{v}_L - g c_3) - \frac{1}{m_i \ell_i} S(q_i) (f_i + d_i)
$$

## 2. 控制设计 (Control Design)

### 载荷位置控制 (Payload Position Control)
**误差定义:**
$$
e_p = p_L - p_d, \quad e_v = v_L - \dot{p}_d
$$
**耦合误差:**
$$
e = k_1 (e_p + \beta e_v)
$$
**载荷期望力 (Desired Force for Payload, Eq. 14):**
$$
f_{dL} = \sigma(e) + k_2 \sigma(\beta \sigma(e) + e_v) - g c_3 - \ddot{p}_d + M^{-1} (\hat{d}_L + \sum_{i=1}^n \hat{d}_{iq})
$$
**注意**：此处应为 **-g c₃**，因为 f_dL 代表期望的**推力加速度**，而动力学方程(Eq. 8)中重力单独添加为 +gc₃。要抵消重力实现悬停，推力加速度需为 -gc₃。

### 力分配 (Force Distribution, Eq. 17 & 18)
**期望平行分量:**
$$
f_{qdi} = m_i \ell_i \|\omega_i\|^2 q_i - m_i q_i q_i^T f_{dL} - u_i q_i
$$
**辅助力分配:**
$$
u = m_L Q^T (Q Q^T)^{-1} f_{dL}
$$
其中 $Q = [q_1, q_2, \dots, q_n]$。

### 绳索构型控制 (Cable Configuration Control)
**误差定义:**
$$
e_{qi} = q_i - q_{di}, \quad e_{\omega i} = S(q_i)(\omega_i - \omega_{di})
$$
**期望角速度 (Desired Angular Velocity, Eq. 24):**
$$
\omega_{di} = S(q_{di})\dot{q}_{di} + \frac{k_q}{h_q} S(q_i) q_{di}
$$
**期望垂直力 (Desired Perpendicular Force, Eq. 28):**
$$
f_{d\perp i} = m_i \ell_i S^2(q_i) \left[ \frac{1}{\ell_i} S^2(q_i)(\dot{v}_{Ln} - g c_3) - S(q_i)\dot{\omega}_{di} + \frac{1}{m_i \ell_i} \hat{d}_i + S(\dot{q}_i)(\omega_i - \omega_{di}) + \frac{h_q}{h_\omega} e_{qi} + \frac{k_\omega}{h_\omega} e_{\omega i} \right]
$$
其中 $\dot{v}_{Ln} = -\sigma(e) - k_2 \sigma(\beta \sigma(e) + e_v) + \ddot{p}_d$。

### 四旋翼姿态控制 (Quadrotor Attitude Control)
**误差定义:**
$$
e_{ri} = r_i - r_{di}, \quad r_{di} = f_{di} / \|f_{di}\|
$$
**控制律 (Control Law, Eq. 33):**
$$
\Omega_i = z_i c_3 - \frac{k_r}{h_r} S(c_3) R_i^T S(r_{di}) r_{di} + S(c_3) R_i^T \dot{r}_{di} + \frac{T_{di}}{h_r} S(c_3) R_i^T \left( \frac{\partial V_3}{\partial e_{\omega i}} \right)^T + \frac{T_{di}}{h_r} S(c_3) R_i^T q_i q_i^T M^{-1} \left( \frac{\partial V_3}{\partial e_v} \right)^T
$$
其中 $z_i = -k_\psi \psi_i - \psi_{di}$。

### 扰动估计 (Disturbance Estimation, Eq. 35 & 36)
$$
\dot{\hat{d}}_L = h_{dL} \text{Proj}\left( M^{-1} \left( \frac{\partial V_4}{\partial e_v} \right)^T, \hat{d}_L \right)
$$
$$
\dot{\hat{d}}_i = h_{di} \text{Proj}\left( q_i q_i^T M^{-1} \left( \frac{\partial V_4}{\partial e_v} \right)^T + \left( \frac{\partial V_4}{\partial e_{\omega i}} \right)^T, \hat{d}_i \right)
$$
