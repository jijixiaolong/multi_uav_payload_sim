# Multi-UAV Payload Simulation

基于论文 "Robust Cooperative Transportation of a Cable-Suspended Payload by Multiple Quadrotors Featuring Cable-Reconfiguration Capabilities" 的多无人机吊挂载荷协同运输仿真系统。

## 项目结构

```
multi_uav_payload_sim/
├── main.m              # 主入口脚本
├── params.m            # 系统参数定义
├── trajectory.m        # 期望轨迹生成
├── formulas.md         # 论文公式摘要（中文）
├── dynamics/           # 动力学模块
│   ├── system_dynamics.m   # 完整系统动力学
│   ├── payload_dyn.m       # 载荷动力学 (Eq 8)
│   ├── cable_dyn.m         # 绳索动力学 (Eq 9)
│   └── attitude_dyn.m      # 姿态动力学 (Eq 1c)
├── control/            # 控制模块
│   ├── payload_ctrl.m      # 载荷位置控制 (Eq 14)
│   ├── force_allocation.m  # 力分配 (Eq 17, 18)
│   ├── cable_ctrl.m        # 绳索构型控制 (Eq 28)
│   ├── attitude_ctrl.m     # 姿态控制 (Eq 33)
│   └── disturbance_est.m   # 扰动估计 (Eq 35, 36)
├── math/               # 数学工具函数
│   ├── hat.m               # 向量 -> 反对称矩阵
│   ├── vee.m               # 反对称矩阵 -> 向量
│   ├── sat.m               # 饱和函数 (tanh)
│   ├── proj_SO3.m          # 投影到 SO(3)
│   └── proj_S2.m           # 投影到单位球
├── utils/              # 工具函数
│   ├── pack_state.m        # 打包状态向量
│   ├── unpack_state.m      # 解包状态向量
│   └── init_state.m        # 初始化状态
└── plot/               # 绘图函数
    ├── plot_payload.m      # 载荷轨迹与误差
    ├── plot_cable.m        # 绳索误差
    ├── plot_disturb.m      # 扰动估计误差
    └── plot_input.m        # 控制输入
```

## 快速开始

1. 打开 MATLAB，切换到项目目录
2. 运行 `main.m`

```matlab
main
```

## 坐标系

本仿真使用 **北东地 (NED)** 坐标系：
- X 轴：指向北
- Y 轴：指向东  
- Z 轴：指向地心（向下为正）

## 状态向量定义

状态向量 `x` 包含以下变量（总长度 72，当 n=3 时）：

| 变量 | 维度 | 说明 |
|------|------|------|
| $p_L$ | 3×1 | 载荷位置 |
| $v_L$ | 3×1 | 载荷速度 |
| $R_i$ | 9×1 | 无人机姿态（展平） |
| $\Omega_i$ | 3×1 | 无人机角速度 |
| $q_i$ | 3×1 | 绳索方向（UAV→Payload） |
| $\omega_i$ | 3×1 | 绳索角速度 |
| $\hat{d}_L$ | 3×1 | 载荷扰动估计 |
| $\hat{d}_i$ | 3×1 | 无人机扰动估计 |

## 关键公式

无人机位置由载荷位置和绳索方向确定：
$$p_i = p_L - \ell_i q_i$$

其中 $q_i$ 定义为从无人机指向载荷的单位向量。

## 参数说明

主要参数在 `params.m` 中定义：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mL` | 0.06 kg | 载荷质量 |
| `mi` | 0.21 kg | 无人机质量 |
| `li` | 0.6 m | 绳长 |
| `n` | 3 | 无人机数量 |

## 当前状态

- ✅ 系统动力学已实现
- ✅ 简化 PD 控制器可正常运行
- ⚠️ 论文完整控制器需要进一步调试

## 参考文献

> Guanrui Li, Giuseppe Loianno. "Robust Cooperative Transportation of a Cable-Suspended Payload by Multiple Quadrotors Featuring Cable-Reconfiguration Capabilities"
