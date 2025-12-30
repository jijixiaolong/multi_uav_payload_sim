# 多无人机吊挂载荷协同运输仿真 - 简化版

## 项目概述

本项目实现了基于论文 "Robust Cooperative Transportation of a Cable-Suspended Payload by Multiple Quadrotors" (Wang et al., IEEE TITS 2024) 的**简化控制模型**。

### 简化策略
- **假设理想推力跟踪**: `f = f_di`（无人机底层控制器完美跟踪期望推力）
- **移除姿态动力学**: 姿态不参与ODE求解（`dR = 0`）
- **移除扰动估计**: 无自适应补偿项

### 性能指标
- **跟踪精度**: 平均 0.09 cm（亚毫米级）
- **初始高度一致性**: 三架UAV高度差 0 cm
- **仿真速度**: 20秒仿真耗时 0.4 秒

---

## 项目结构

```
multi_uav_payload_sim/
├── params.m                    # 参数配置（物理参数、控制增益）
├── trajectory.m                # 期望轨迹生成（圆形/直线/矩形）
│
├── control/                    # 控制模块
│   ├── payload_ctrl.m         # 载荷位置控制（Eq 14）
│   ├── force_allocation.m     # 力分配到各缆绳（Eq 17-18）
│   ├── cable_ctrl.m           # 缆绳配置控制（Eq 24, 28）
│   └── attitude_ctrl.m        # 姿态控制（简化为理想跟踪）
│
├── dynamics/                   # 动力学模块
│   └── system_dynamics.m      # 系统主动力学（载荷+缆绳）
│
├── utils/                      # 工具函数
│   ├── init_state.m           # 初始化状态（对称配置）
│   ├── pack_state.m           # 状态向量打包
│   └── unpack_state.m         # 状态向量解包
│
├── math/                       # 数学工具
│   ├── hat.m                  # 向量转反对称矩阵
│   ├── vee.m                  # 反对称矩阵转向量
│   ├── sat.m                  # 饱和函数 tanh
│   ├── proj_SO3.m             # 投影到旋转群
│   └── proj_S2.m              # 投影到单位球面
│
├── plot/                       # 绘图模块
│   └── plot_all_in_one.m      # 综合绘图（含UAV高度曲线）
│
├── scripts/                    # 运行脚本
│   └── main.m                 # 主仿真脚本
│
├── results/                    # 仿真结果（自动生成）
│   └── sim_accurate_*.mat     # 保存的仿真数据
│
├── formulas.md                 # 论文公式参考
├── CLAUDE.md                   # Claude Code 指南
└── _backup_unused/             # 已删除文件的备份
    ├── dynamics/               # 独立动力学模块（已集成）
    ├── control/                # 扰动估计（已移除）
    ├── plot/                   # 详细绘图函数（已简化）
    └── test/                   # 测试脚本（已清理）
```

---

## 核心文件说明

### 1. 配置与轨迹

#### `params.m`
定义所有仿真参数：
- **物理参数**: 质量 (mL, mi)、缆绳长度 (li)、期望天顶角 (theta_d = 40°)
- **控制增益**: k1=15, k2=8, kq=15, komega=8（针对简化模型优化）
- **扰动**: 当前设为零，可取消注释启用论文扰动

#### `trajectory.m`
生成期望轨迹及其导数（最高到4阶）。**新增多种轨迹类型**：

**平面轨迹（2D）**：
- `'circle'`: 圆形轨迹（R=1m, T=20s, 默认）
- `'line'`: 直线轨迹（0.05 m/s）
- `'rectangle'`: 圆角矩形轨迹（保证速度连续）
- `'figure8'`: 8字形轨迹（利萨如曲线，平滑且对称）
- `'sine_wave'`: 正弦波轨迹（沿X轴前进，Y方向振荡）
- `'polygon'`: 正六边形轨迹（圆角处理）
- `'hover'`: 悬停（固定点，测试稳定性）

**3D轨迹**：
- `'helix'`: 螺旋轨迹（XY平面圆周运动 + Z方向缓慢上升）
- `'clelia'`: Clelia曲线（论文原始轨迹，过于激进不推荐）

**设计原则**：所有轨迹均设计为慢速、低加速度，确保系统稳定性

---

### 2. 控制模块

控制流程（分层backstepping）：
```
trajectory → payload_ctrl → force_allocation → cable_ctrl → attitude_ctrl
```

#### `payload_ctrl.m`
- **输入**: 期望轨迹、载荷状态、缆绳方向
- **输出**: 载荷期望力 `f_dL`
- **公式**: Eq 14（简化：无扰动估计）
- **关键**: 饱和函数 σ(e) = tanh(e) 保证 Lyapunov 稳定性

#### `force_allocation.m`
- **输入**: `f_dL`, 缆绳方向 `q`, 角速度 `omega`
- **输出**: 各缆绳平行力 `f_qdi`
- **公式**: Eq 17-18
- **作用**: 将总期望力分配到3根缆绳，保证合力平衡

#### `cable_ctrl.m`
- **输入**: `f_dL`, 缆绳状态 `q`, `omega`
- **输出**: 垂直力 `f_di_perp`, 期望角速度 `omega_di`
- **公式**: Eq 24, 28
- **作用**: 调整缆绳方向到期望配置（40°天顶角，120°间隔）

#### `attitude_ctrl.m`（简化版）
- **输入**: `f_qdi`, `f_di_perp`
- **输出**: 推力大小 `T`, 期望推力向量 `f_di`
- **简化**: 直接返回 `f_di = f_qdi + f_di_perp`，假设理想跟踪

---

### 3. 动力学模块

#### `system_dynamics.m`
ODE右端函数，计算状态导数 `dx = f(t, x)`：

**状态向量** (63维，n=3时):
```
x = [pL(3), vL(3), R1(9), R2(9), R3(9), q1(3), q2(3), q3(3), 
     omega1(3), omega2(3), omega3(3), dL_hat(3), d_hat1(3), d_hat2(3), d_hat3(3)]
```

**计算流程**:
1. 解包状态: `unpack_state(x)` → pL, vL, R, q, omega, ...
2. 调用控制器: `controller_wrapper()` → f_di
3. 载荷动力学: `dvL = M\(sum_forces) + g*e3` (Eq 8)
4. 缆绳动力学: `domega = ...` (Eq 9)
5. 姿态动力学: `dR = 0` (简化)
6. 打包导数: `pack_state(dpL, dvL, dR, dq, domega, ...)`

**关键简化**:
- 实际推力 `f = f_di`（理想跟踪）
- 姿态不更新 `dR = 0`
- 扰动估计为零 `dL_hat_dot = 0`, `d_hat_dot = 0`

---

### 4. 初始化

#### `init_state.m`
生成初始状态 `x0`:

**策略**（确保对称性）:
1. 载荷位置: `pL0 = trajectory(0)`（轨迹起点）
2. 缆绳方向: **直接使用期望配置**
   - 天顶角: `theta = 40°`（与期望一致，避免大幅调整）
   - 方位角: `[0°, 120°, 240°]`（水平均匀分布）
3. 姿态矩阵: `R = eye(3)`（简化模型不使用）

**效果**: 三架UAV初始高度完全一致（高度差 0 cm）

---

### 5. 绘图

#### `plot_all_in_one.m`
6子图综合显示:
1. **3D轨迹带缆绳**（左上）
2. **俯视图 XY平面**（中上）
3. **侧视图 XZ平面**（右上）
4. **X方向位置**（左下）
5. **Y方向位置**（中下）
6. **UAV高度曲线**（右下，新增）

**统计输出**:
- 跟踪误差（最大/平均/最终）
- 各UAV平均高度、标准差
- 初始/最终高度差

---

## 运行说明

### 快速开始

```matlab
cd multi_uav_payload_sim/scripts
main
```

### 修改配置

#### 1. 切换轨迹类型
编辑 [scripts/trajectory.m](scripts/trajectory.m) 第17行:
```matlab
traj_type = 'circle';  % 可选: 'line', 'rectangle', 'circle', 'figure8',
                       %       'helix', 'sine_wave', 'polygon', 'hover', 'clelia'
```

**推荐轨迹**（稳定且美观）：
- `'circle'`: 标准测试，完美对称
- `'figure8'`: 8字形，展示平面机动性
- `'sine_wave'`: 正弦波，测试沿轴振荡
- `'helix'`: 3D螺旋，测试高度变化

**高级轨迹**（需谨慎使用）：
- `'rectangle'`, `'polygon'`: 包含尖角（已圆角处理）
- `'clelia'`: 激进轨迹，可能导致不稳定

#### 2. 调整仿真模式
编辑 `main.m` 第22行:
```matlab
sim_mode = 'accurate';  % 'debug', 'fast', 'accurate'
```
- `debug`: 1秒快速验证
- `fast`: 2秒中等精度
- `accurate`: 20秒高精度

#### 3. 修改控制增益
编辑 `params.m` 第19-22行:
```matlab
params.k1 = 15;   % 位置增益
params.k2 = 8;    % 速度增益
params.kq = 15;   % 缆绳方向增益
params.komega = 8; % 缆绳角速度增益
```

#### 4. 启用扰动
编辑 `params.m` 取消第51-52行注释:
```matlab
params.d_L_true = 0.1 * params.mL * params.g * [2/3; 2/3; 1/3];
params.d_i_true = 0.1 * params.mi * params.g * repmat([2/3; 2/3; 1/3], 1, params.n);
```

---

## 坐标系说明

### NED坐标系（北东地）
- **X轴**: 北 (North)
- **Y轴**: 东 (East)
- **Z轴**: 地 (Down, 向下为正)

### 关键约定
- 高度 = -Z（负Z值表示高度）
- `pL = [0, 0, -2]` → 载荷在2米高度
- 重力 `g*e3 = g*[0,0,1]` 指向下方

### 缆绳方向
- `q_i` 指向: **载荷 → 无人机**
- `p_i = pL - li * q_i`（无人机位置）
- 天顶角 = `acos(q_z)`（与+Z轴夹角）

---

## 性能优化记录

| 版本 | 平均误差 | 初始高度差 | 说明 |
|------|---------|-----------|------|
| 原论文参数 | 崩溃 | N/A | theta_d=40°初始化不稳定 |
| 增大增益3-4倍 | 0.82 cm | 57 cm | UAV3高度异常 |
| 120°均匀分布 | 0.09 cm | 0 cm | 期望配置，完美对称 |

**最终配置**:
- 初始天顶角 = 期望天顶角 = 40°
- 方位角: 0°, 120°, 240°（均匀）
- 控制增益: k1=15, k2=8, kq=15, komega=8

---

## 关键简化点

### 与论文的区别

| 项目 | 论文完整模型 | 本简化模型 |
|------|-------------|-----------|
| 姿态动力学 | Ṙ = R·S(Ω) | dR = 0 |
| 推力计算 | f = -T·R·e3 | f = f_di |
| 扰动估计 | Eq 35-36 | 关闭 |
| 姿态控制 | Eq 33（完整） | 理想跟踪 |
| 初始化 | 迭代求解平衡 | 直接期望配置 |

### 保留的核心

✅ 载荷位置控制（Eq 14）  
✅ 力分配（Eq 17-18）  
✅ 缆绳配置控制（Eq 24, 28）  
✅ 载荷动力学（Eq 8）  
✅ 缆绳动力学（Eq 9）  
✅ Backstepping控制结构  
✅ Lyapunov稳定性（饱和函数）

---

## 扩展建议

### 可添加功能
1. **扰动抑制**: 取消 `params.m` 中扰动注释
2. **更多轨迹**: 在 `trajectory.m` 添加新轨迹类型
3. **动态避障**: 在控制器中添加避障约束
4. **完整姿态**: 恢复 Eq 33 实现（参考备份文件）

### 调试工具
备份目录 `_backup_unused/test/` 包含:
- `debug_forces.m`: 力平衡验证
- `verify_results.m`: 结果分析
- `monitor_sim.m`: 实时监控

恢复方法:
```bash
cp _backup_unused/test/*.m scripts/
```

---

## 依赖与兼容性

- **MATLAB版本**: R2020a 及以上
- **工具箱**: 无需额外工具箱（仅基础MATLAB）
- **求解器**: `ode15s`（刚性求解器）

---

## 参考文献

Wang, G., Li, G., & Loianno, G. (2024). "Robust Cooperative Transportation of a Cable-Suspended Payload by Multiple Quadrotors Featuring Cable-Reconfiguration Capabilities." *IEEE Transactions on Intelligent Transportation Systems*.

---

## 更新日志

**2024-12-30**: 简化模型完成
- ✅ 移除姿态动力学、扰动估计
- ✅ 优化初始化（确保对称配置）
- ✅ 添加UAV高度曲线绘图
- ✅ 清理未使用模块
- ✅ 达到亚毫米级跟踪精度

