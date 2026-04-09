# Python Leg Odometry 技术原理详解文档

## 目录

1. [引言](#1-引言)
2. [卡尔曼滤波器理论基础](#2-卡尔曼滤波器理论基础)
3. [系统建模](#3-系统建模)
4. [状态向量与观测模型](#4-状态向量与观测模型)
5. [滤波器实现细节](#5-滤波器实现细节)
6. [足部接触与滑动检测](#6-足部接触与滑动检测)
7. [噪声参数分析](#7-噪声参数分析)
8. [滤波器类型对比](#8-滤波器类型对比)
9. [参数调优指南](#9-参数调优指南)
10. [代码实现分析](#10-代码实现分析)
11. [典型应用场景](#11-典型应用场景)
12. [数学公式汇总](#12-数学公式汇总)

---

## 1. 引言

### 1.1 项目背景

`python_leg_odom` 是基于 MIT 腿部机器人研究团队提出的 IMU 融合方法，使用卡尔曼滤波技术实现腿足机器人本体位置和速度的高精度估计。该实现是对原始 C++ 版本的 Python 移植。

### 1.2 为什么需要腿部里程计

传统的轮式里程计在腿足机器人上无法直接使用，因为：
- 足部与地面的接触是间歇性的（支撑相与摆动相交替）
- 支撑足相对于地面可能发生滑动
- IMU 存在积分漂移问题

因此需要融合 IMU 和腿部运动学数据，互补长短。

### 1.3 核心挑战

1. **数据融合**：如何融合 IMU 和腿部里程计
2. **支撑/摆动检测**：如何处理不同相位的足部数据
3. **滑动检测**：如何检测和处理足部滑动
4. **漂移抑制**：如何抑制 IMU 积分漂移

---

## 2. 卡尔曼滤波器理论基础

### 2.1 什么是卡尔曼滤波器

卡尔曼滤波器（Kalman Filter, KF）是一种**线性最优状态估计器**，它在系统模型和观测数据之间寻找最优平衡。其核心思想是：
- 利用**先验知识**（系统模型）进行预测
- 利用**测量数据**进行修正
- 根据噪声水平动态调整预测和修正的权重

### 2.2 卡尔曼滤波的两个步骤

```
┌─────────────────────────────────────────────────────────┐
│                    卡尔曼滤波循环                        │
│                                                         │
│  ┌──────────────┐     ┌──────────────┐                  │
│  │   预测步骤    │────▶│   更新步骤    │                  │
│  │   (Predict)   │     │   (Update)   │                  │
│  └──────────────┘     └──────────────┘                  │
│        │                       │                         │
│        ▼                       ▼                         │
│   x̂ₖ⁻ = A·x̂ₖ₋₁ + B·uₖ    x̂ₖ = x̂ₖ⁻ + Kₖ(zₖ - H·x̂ₖ⁻)       │
│   Pₖ⁻ = A·Pₖ₋₁·A' + Q        Pₖ = (I - Kₖ·H)·Pₖ⁻        │
└─────────────────────────────────────────────────────────┘
```

### 2.3 预测步骤（Predict）

已知上一时刻状态估计 $\hat{x}_{k-1}$ 和协方差 $P_{k-1}$：

**状态预测**：
$$\hat{x}_k^- = A \cdot \hat{x}_{k-1} + B \cdot u_k$$

**协方差预测**：
$$P_k^- = A \cdot P_{k-1} \cdot A^T + Q$$

其中：
- $A$：状态转移矩阵
- $B$：输入矩阵
- $u_k$：系统输入（如 IMU 加速度）
- $Q$：过程噪声协方差矩阵

### 2.4 更新步骤（Update）

**计算卡尔曼增益**：
$$S_k = H \cdot P_k^- \cdot H^T + R$$
$$K_k = P_k^- \cdot H^T \cdot S_k^{-1}$$

**状态更新**：
$$\hat{x}_k = \hat{x}_k^- + K_k \cdot (z_k - H \cdot \hat{x}_k^-)$$

**协方差更新**：
$$P_k = (I - K_k \cdot H) \cdot P_k^-$$

其中：
- $H$：观测矩阵
- $z_k$：观测向量
- $R$：观测噪声协方差矩阵
- $K_k$：卡尔曼增益

### 2.5 卡尔曼增益的物理意义

卡尔曼增益 $K_k$ 决定了预测值和观测值的融合权重：

- **$K_k$ 接近 0**：更信任预测值，观测噪声大
- **$K_k$ 接近 1**：更信任观测值，预测不确定度大

---

## 3. 系统建模

### 3.1 状态转移模型

在离散时间系统中，假设采样时间为 $dt$，使用恒定加速度模型：

**位置更新**：
$$p_{k+1} = p_k + v_k \cdot dt + \frac{1}{2} a \cdot dt^2$$

**速度更新**：
$$v_{k+1} = v_k + a \cdot dt$$

**足部位置**（支撑相保持不变）：
$$f_{k+1} = f_k$$

### 3.2 状态转移矩阵

根据上述模型，状态转移矩阵 $A$ 为：

$$
A = \begin{bmatrix}
I_3 & dt \cdot I_3 & 0 \\
0 & I_3 & 0 \\
0 & 0 & I_{3n}
\end{bmatrix}
$$

其中：
- $I_3$：3×3 单位矩阵
- $I_{3n}$：3n×3n 单位矩阵（n为腿的数量）
- $dt$：采样时间

### 3.3 输入矩阵

IMU 加速度作为系统输入，输入矩阵 $B$ 为：

$$
B = \begin{bmatrix}
\frac{1}{2}dt^2 \cdot I_3 \\
dt \cdot I_3 \\
0
\end{bmatrix}
$$

因此状态预测方程为：
$$
\hat{x}_k^- = \begin{bmatrix}
p + v \cdot dt + \frac{1}{2}a \cdot dt^2 \\
v + a \cdot dt \\
f
\end{bmatrix}
$$

### 3.4 过程噪声

过程噪声 $Q$ 反映了系统模型的不确定性：

$$
Q = \begin{bmatrix}
Q_p & 0 & 0 \\
0 & Q_v & 0 \\
0 & Q_f
\end{bmatrix}
$$

- $Q_p$：位置过程噪声（与 $k_{pb}$ 相关）
- $Q_v$：速度过程噪声（与 $k_{vb}$ 相关）
- $Q_f$：足部过程噪声（与 $k_{pf}$ 相关）

代码中的实现（第329-331行）：
```python
self._Q[0:3, 0:3] = (self.T_predict / 20.0) * Kpb * np.eye(3)
self._Q[3:6, 3:6] = (self.T_predict * 9.8 / 20.0) * Kvb * np.eye(3)
self._Q[6:6+3*self.branchn, 6:6+3*self.branchn] = self.T_predict * Kpf * np.eye(3 * self.branchn)
```

---

## 4. 状态向量与观测模型

### 4.1 状态向量定义

状态向量 $X$ 包含机器人本体和所有足部的位置与速度信息：

$$
X = \begin{bmatrix}
p_x \\ p_y \\ p_z \\
v_x \\ v_y \\ v_z \\
f_{1x} \\ f_{1y} \\ f_{1z} \\
f_{2x} \\ f_{2y} \\ f_{2z} \\
\vdots \\
f_{nx} \\ f_{ny} \\ f_{nz}
\end{bmatrix}
= \begin{bmatrix}
\text{body}_p \\ \text{body}_v \\ \text{foot}_p
\end{bmatrix}
$$

| 组成部分 | 维度 | 索引 | 说明 |
|---------|------|------|------|
| body_p | 3 | 0:3 | 本体位置 [x, y, z] |
| body_v | 3 | 3:6 | 本体速度 [vx, vy, vz] |
| foot_p | 3×n | 6:end | 足部位置 |

### 4.2 观测向量

观测向量 $Z$ 包含腿部里程计的测量数据：

对于 EKF_IL 模式（无外部观测）：
$$
Z = \begin{bmatrix}
\Delta p_1 \\ \Delta p_2 \\ \cdots \\ \Delta p_n \\
\Delta v_1 \\ \Delta v_2 \\ \cdots \\ \Delta v_n \\
h_1 \\ h_2 \\ \cdots \\ h_n
\end{bmatrix}
$$

对于 EKF_EIL 模式（有外部观测）：
$$
Z = \begin{bmatrix}
\Delta p_1 \\ \Delta p_2 \\ \cdots \\ \Delta p_n \\
\Delta v_1 \\ \Delta v_2 \\ \cdots \\ \Delta v_n \\
h_1 \\ h_2 \\ \cdots \\ h_n \\
p_{ext} \\ v_{ext}
\end{bmatrix}
$$

### 4.3 观测矩阵

观测矩阵 $H$ 将状态空间映射到观测空间：

**足部位置观测**（第302-303行）：
```python
self._C[3*i:3*(i+1), 0:3] = np.eye(3)  # 身体位置
self._C[3*i:3*(i+1), self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1)] = -np.eye(3)  # 足部位置
```
物理意义：$z_{dp} = p_{body} - f_{foot}$

**足部速度观测**（第304行）：
```python
self._C[3*(self.branchn + i):3*(self.branchn + i + 1), 3:6] = np.eye(3)
```
物理意义：$z_{dv} = v_{body} - v_{foot}$

**高度观测**（第305行）：
```python
self._C[6*self.branchn + i, 8 + 3*i] = 1
```
物理意义：$z_h = p_{body,z} + \Delta p_{foot,z}$

### 4.4 观测方程的物理意义

代码中的观测方程构建（第367-373行）：

```python
# 位置观测
self._z[rindex1:rindex1+3, 0] = -self.foot_dp[:, i]
# 物理含义: body_p - foot_p - foot_dp = 0

# 速度观测
v_term = (1.0 - trust) * v0 + trust * (-self.foot_dv[:, i])
self._z[rindex2:rindex2+3, 0] = v_term
# 物理含义: 根据接触状态选择速度来源

# 高度观测
z_height = (1.0 - trust) * (p0[2] + self.foot_dp[2, i]) + FOOT_PLANE * trust * self.foot_p0[2, i]
self._z[rindex3, 0] = z_height
```

---

## 5. 滤波器实现细节

### 5.1 矩阵初始化

**状态转移矩阵 A**（第292-297行）：
```python
self._A = np.eye(self.XN)
self._A[0:3, 3:6] = dt * np.eye(3)  # 位置受速度影响
self._B[0:3, 0:3] = 0.5 * dt * dt * np.eye(3)  # 位置受加速度影响
self._B[3:6, 0:3] = dt * np.eye(3)  # 速度受加速度影响
```

### 5.2 协方差矩阵初始化

**状态协方差 P**（第311行）：
```python
self._P = 100 * np.eye(self.XN)
```
初始值较大（100），表示初始状态的不确定度较高。

### 5.3 卡尔曼滤波计算流程

**kf_once() 方法**（第87-106行）：

1. **状态预测**：
   ```python
   self._x = self._A @ self._x + self._B @ self._u
   ```

2. **预测协方差**：
   ```python
   At = self._A.T
   Pm = self._A @ self._P @ At + self._Q
   ```

3. **计算新息**：
   ```python
   Ct = self._C.T
   zModel = self._C @ self._x
   ey = self._z - zModel  # 新息（测量残差）
   ```

4. **计算卡尔曼增益**：
   ```python
   S = self._C @ Pm @ Ct + self._R
   S_ey = np.linalg.solve(S, ey)
   self._x += Pm @ Ct @ S_ey
   ```

5. **更新协方差**：
   ```python
   S_C = np.linalg.solve(S, self._C)
   self._P = (self._I - Pm @ Ct @ S_C) @ Pm
   ```

6. **对称化**（数值稳定性）：
   ```python
   self._P = (self._P + self._P.T) / 2.0
   ```

### 5.4 数值稳定性处理

代码中使用了两个关键技巧：

1. **协方差对称化**（第106行）：
   ```python
   self._P = (self._P + self._P.T) / 2.0
   ```
   由于舍入误差，$P$ 可能变得非对称，这会导致数值不稳定。

2. **XY平面约束**（第377-380行）：
   ```python
   if np.linalg.det(self._P[0:2, 0:2]) > 0.000001:
       self._P[0:2, 2:2+4+3*self.branchn] = 0
       self._P[2:2+4+3*self.branchn, 0:2] = 0
       self._P[0:2, 0:2] = self._P[0:2, 0:2] / 10
   ```
   这个处理假设机器人在 XY 平面上不会发生剧烈变化，将 XY 位置与其他状态解耦。

---

## 6. 足部接触与滑动检测

### 6.1 支撑相与摆动相

腿部机器人在行走过程中交替出现两种相位：

| 相位 | 接触标志 | 特点 |
|-----|---------|------|
| 支撑相(Stance) | contact = 1.0 | 足部与地面接触，可靠的参考点 |
| 摆动相(Swing) | contact = 0.0 | 足部在空中移动，测量不可靠 |

### 6.2 摆动腿噪声处理

代码中使用 trust 因子处理两种相位（第352-365行）：

```python
trust = min(self.foot_contact[i], 1.0)  # 接触置信度
thek0 = 1 + SWING_NOISE * (1 - trust)   # 噪声放大因子

# 摆动相时
if trust == 0:
    thek0 = 1 + 1000  # 噪声放大1000倍
```

物理意义：
- 支撑腿（trust=1）：正常噪声权重
- 摆动腿（trust=0）：噪声放大1000倍，忽略其测量

### 6.3 滑动检测机制

代码中的滑动检测（第226-237行）：

```python
flag = 0
if contact[i] >= 1:  # 支撑腿
    the_dp = self.foot_p0[:, i] - self._x[self.foot_p_id + 3*i:...].flatten()
    
    if self.foot_contact[i] < 0:
        flag = 1  # 刚着地，初始化参考点
    elif np.linalg.norm(the_dp) > FOOT_SLIP_MIN:
        flag = 1  # 检测到滑动

if flag:
    self.foot_p0[:, i] = self._x[self.foot_p_id + 3*i:...].flatten()
```

滑动检测逻辑：
1. **首次着地**：重置参考位置
2. **位置突变**：当支撑足位置变化超过阈值（1.0米）时，判定为滑动

---

## 7. 噪声参数分析

### 7.1 噪声参数总览

| 参数 | 位置 | 作用 | 增大效果 | 减小效果 |
|-----|------|------|---------|---------|
| kpb | Q[0:3,0:3] | 预测位置噪声 | 信任IMU积分少 | 信任IMU积分多 |
| kvb | Q[3:6,3:6] | 预测速度噪声 | 速度变化更平滑 | 速度响应更快 |
| kpf | Q[6:,6:] | 预测足位噪声 | 足位变化更随机 | 足位更稳定 |
| kpbf | R[0:3n,0:3n] | 观测足位噪声 | 信任腿部里程计少 | 信任腿部里程计多 |
| kpfv | R[3n:6n,3n:6n] | 观测足速噪声 | 信任足速少 | 信任足速多 |
| kfz | R[6n:7n,6n:7n] | 观测高度噪声 | 忽略高度观测 | 重视高度观测 |
| kpex | R[7n:7n+3,7n:7n+3] | 外部位置噪声 | 忽略外部位置 | 重视外部位置 |
| kvex | R[7n+3:,7n+3:] | 外部速度噪声 | 忽略外部速度 | 重视外部速度 |

### 7.2 噪声参数与协方差矩阵

代码中噪声参数的映射（第328-340行）：

**过程噪声 Q**：
```python
self._Q[0:3, 0:3] = (dt / 20.0) * Kpb * I_3
self._Q[3:6, 3:6] = (dt * 9.8 / 20.0) * Kvb * I_3
self._Q[6:, 6:] = dt * Kpf * I_(3n)
```

**观测噪声 R**：
```python
self._R[0:3n, 0:3n] = Kpbf * I_(3n)
self._R[3n:6n, 3n:6n] = Kpfv * I_(3n)
self._R[6n:7n, 6n:7n] = Kfz * I_n
```

注意：速度过程噪声中包含重力因子 9.8，这是为了模拟重力的不确定性影响。

### 7.3 默认参数

`create_default_odometry()` 中的默认参数（第513-522行）：
```python
odom.set_noise_parameters(
    kpb=0.1, kvb=0.1, kpf=0.1,
    kpbf=0.1, kpfv=0.1, kfz=0.1,
    kpex=0.1, kvex=0.1
)
```

所有参数均为 0.1，表示预测和观测的信任度基本均衡。

---

## 8. 滤波器类型对比

### 8.1 EKF_IL 模式

```python
self.ZN = 7 * self.branchn
```

观测向量维度：7n（n为腿数）

特点：
- 仅使用 IMU 和腿部里程计
- 适用于没有外部定位的纯惯性导航场景
- 计算量较小

### 8.2 EKF_EIL 模式

```python
self.ZN = 7 * self.branchn + 6
self._C[7*self.branchn:7*self.branchn+3, 0:3] = np.eye(3)      # 外部位置
self._C[7*self.branchn+3:7*self.branchn+6, 3:6] = np.eye(3)    # 外部速度
```

观测向量维度：7n + 6

特点：
- 额外融合外部位置传感器（如视觉里程计 VIO、GPS）
- 可以修正 IMU 积分漂移
- 计算量略大

### 8.3 模式选择建议

| 场景 | 推荐模式 | 原因 |
|-----|---------|------|
| 室内短距离行走 | EKF_IL | 无需外部定位 |
| 长距离户外行走 | EKF_EIL | 需要外部定位修正漂移 |
| 使用视觉里程计 | EKF_EIL | 融合 VIO 数据 |
| GPS 可用 | EKF_EIL | 融合 GPS 数据 |

---

## 9. 参数调优指南

### 9.1 调参基本原则

1. **理解各参数作用**：先理解每个参数的物理意义
2. **逐步调整**：一次只调整一个参数
3. **记录效果**：记录调整前后的效果变化
4. **根据场景调整**：不同场景需要不同参数

### 9.2 典型场景参数配置

#### 场景1：平地行走

```python
odom.set_noise_parameters(
    kpb=0.05,   # 减小：信任 IMU 积分
    kvb=0.05,   # 减小：信任速度估计
    kpf=0.1,
    kpbf=0.2,   # 增大：信任腿部里程计
    kpfv=0.2,
    kfz=0.05,   # 减小：信任高度观测
    kpex=0.1,
    kvex=0.1
)
```

#### 场景2：复杂地形（楼梯、坡道）

```python
odom.set_noise_parameters(
    kpb=0.2,    # 增大：地形变化大，不信任积分
    kvb=0.3,    # 增大：速度变化快
    kpf=0.5,    # 增大：足部滑动频繁
    kpbf=0.5,   # 增大：不信任足位
    kpfv=0.5,
    kfz=0.3,    # 增大：高度变化大
    kpex=0.1,
    kvex=0.1
)
```

#### 场景3：使用高精度 VIO

```python
odom.set_noise_parameters(
    kpb=0.1,
    kvb=0.1,
    kpf=0.1,
    kpbf=0.1,
    kpfv=0.1,
    kfz=0.1,
    kpex=0.01,  # 大幅减小：信任外部定位
    kvex=0.01
)
```

### 9.3 调参辅助工具

可以在代码中添加调试信息来帮助调参：

```python
# 查看状态协方差矩阵
print("位置方差:", np.diag(kf._P[0:3, 0:3]))
print("速度方差:", np.diag(kf._P[3:6, 3:6]))

# 查看卡尔曼增益（各传感器权重）
# 在 estimate_once() 中添加
K = Pm @ Ct @ np.linalg.inv(S)
print("位置卡尔曼增益:", K[0:3, 0:3])
```

---

## 10. 代码实现分析

### 10.1 文件结构

```
python_leg_odom/
├── leg_odom.py    # 约 500 行
│   ├── 常量定义
│   ├── CJW_Leg_Body_KF 类 (底层实现)
│   ├── LegOdometry 类 (高级封装)
│   └── create_default_odometry() 函数
└── example.py     # 示例代码
```

### 10.2 关键代码位置

| 功能 | 代码位置 | 行号 |
|-----|---------|------|
| 状态转移矩阵构建 | initial() | 292-297 |
| 观测矩阵构建 | initial() | 299-309 |
| 卡尔曼滤波计算 | kf_once() | 87-106 |
| 观测向量构建 | estimate_once() | 351-373 |
| 噪声参数应用 | estimate_once() | 328-340 |
| XY平面约束 | estimate_once() | 377-380 |

### 10.3 类设计分析

**CJW_Leg_Body_KF 类**：
- 提供底层卡尔曼滤波器实现
- 直接操作矩阵，灵活但使用复杂
- 适合需要深度定制的场景

**LegOdometry 类**：
- 对 CJW_Leg_Body_KF 的高级封装
- 提供简洁的 API 接口
- 适合大多数应用场景

### 10.4 数据流图

```
传感器数据输入
       │
       ▼
┌─────────────────┐
│   LegOdometry   │
│     .update()   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ set_imu_acc()   │ ──▶ imu_acc
│ set_leg_obs()   │ ──▶ foot_dp, foot_dv, contact
│ set_ext_obs()   │ ──▶ ex_position, ex_velocity
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ estimate_once() │
│   1. 更新 Q,R   │
│   2. 构建 z     │
│   3. kf_once() │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ get_estimate()  │
│   返回状态结果   │
└─────────────────┘
```

---

## 11. 典型应用场景

### 11.1 双足机器人静态站立

```python
# 场景：机器人静止站立在地面上
# 预期：位置和速度保持稳定

contact = np.array([1.0, 1.0])  # 双脚支撑
imu_acc = np.array([0.0, 0.0, -9.8])  # 重力

# 足部增量应该接近0
foot_dp = np.zeros((3, 2))
foot_dv = np.zeros((3, 2))

for _ in range(100):
    odom.update(imu_acc, foot_dp, foot_dv, contact)
    body_p, body_v, _ = odom.get_estimate()

# 预期结果：
# body_p ≈ [0, 0, 0.5]
# body_v ≈ [0, 0, 0]
```

### 11.2 双足机器人行走

```python
# 场景：机器人向前行走
# 预期：位置逐渐增加，速度保持稳定

while True:
    # 摆动相检测
    if step % 2 == 0:
        contact = np.array([1.0, 0.0])  # 右脚支撑
    else:
        contact = np.array([0.0, 1.0])  # 左脚支撑
    
    # 足部运动
    foot_dp[0, contact.argmax()] = 0.02  # 前进方向
    
    odom.update(imu_acc, foot_dp, foot_dv, contact)
    body_p, body_v, _ = odom.get_estimate()
```

### 11.3 融合视觉里程计

```python
# 场景：结合 VIO 使用
# 预期：利用 VIO 数据修正漂移

odom = LegOdometry(filter_type=EKF_EIL)

while True:
    # 获取 VIO 数据
    vio_p, vio_v = get_vio_data()
    
    odom.update(
        imu_acc, foot_dp, foot_dv, contact,
        external_position vio_p,
        external_velocity=vio_v
    )
```

---

## 12. 数学公式汇总

### 12.1 状态向量

$$
X_k = \begin{bmatrix} p_{body} \\ v_{body} \\ f_1 \\ f_2 \\ \vdots \\ f_n \end{bmatrix}_{3+3+3n \times 1}
$$

### 12.2 状态转移方程

$$
\begin{aligned}
p_{k+1} &= p_k + v_k \cdot dt + \frac{1}{2} a_k \cdot dt^2 \\
v_{k+1} &= v_k + a_k \cdot dt \\
f_{k+1} &= f_k
\end{aligned}
$$

### 12.3 观测方程

$$
\begin{aligned}
z_{p,i} &= p_k - f_i - \Delta p_i \\
z_{v,i} &= v_k - v_{foot,i} \\
z_{h,i} &= p_{k,z} + \Delta p_{i,z}
\end{aligned}
$$

### 12.4 卡尔曼增益

$$
K = P^- H^T (H P^- H^T + R)^{-1}
$$

### 12.5 状态更新

$$
\hat{x} = \hat{x}^- + K (z - H \hat{x}^-)
$$

