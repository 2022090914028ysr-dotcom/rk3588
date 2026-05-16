rosrun cartographer_ros cartographer_pbstream_to_ros_map   -pbstream_filename /home/ubuntu/Robit_SlamNav/carto_ws/map/meeting_room.pbstream   -map_filestem /home/ubuntu/Robit_SlamNav/carto_ws/map/Office_map   -resolution 0.05

rosservice call /write_state "{filename: '/home/ubuntu/Robit_SlamNav/carto_ws/map/meeting_room.pbstream', include_unfinished_submaps: true}"

####mapping ###
roslaunch cartographer_ros livox_2d.launch


######Localization#####

roslaunch cartographer_ros livox_2d_localization.launch


## reinitialization ###
roslaunch cartographer_ros cartographer_guard.launch

export DISPLAY=:0
source devel_isolated/setup.bash
conda activate control


# 机器人异常状态检测与自动重定位测试报告

## 1. 概述

本报告记录了机器人导航系统在异常状态下的自动检测与重定位功能测试结果。基于前期评估报告（《机器人导航系统评估报告》）中提出的优化建议，本阶段针对三类典型异常场景——**机器人摔倒**、**机器人被搬离**、**运行中定位丢失**——实现了自动检测与快速重定位机制，并完成了功能验证测试。

测试核心目标：检测到异常后，系统自动触发重定位，重定位时间（从触发重定位到定位收敛）在 **5～12 秒** 内，满足实际使用需求。

---

## 2. 系统架构

### 2.1 软件模块

系统由三个模块协同工作：

- **`livox_2d_localization.lua`**：Cartographer 纯定位模式配置，负责实时扫描匹配与全局位姿优化；
- **`cartographer_guard.py`**：异常状态守护节点，实现四状态机监控逻辑；
- **`cartographer_guard.launch`**：统一参数化启动入口。

### 2.2 状态机设计

守护节点实现四状态流转：

```
STATE_MONITOR
    │  连续 10 帧匹配分数 < score_threshold
    ▼
STATE_WAITING
    │  连续 10 帧雷达波动 < scan_change_threshold
    ▼
STATE_RECOVERING  ──→  结束旧轨迹 + 0.5 s 等待 + 启动新轨迹
    ▼
STATE_STABILIZING ──→  等待 stabilizing_duration 秒后返回 MONITOR
```

重定位时间定义为进入 `STATE_RECOVERING` 起至 Cartographer 完成全局约束搜索、定位收敛为止的时长。

---

## 3. 关键参数说明

以下三个核心参数均在 `cartographer_guard.launch` 中配置：

### 参数一：匹配质量分数阈值（`score_threshold`）

| 项目 | 值 |
|------|----|
| 参数名 | `score_threshold` |
| 当前设定值 | **0.6** |
| 作用 | 判定定位是否丢失的质量分数下界。当 `/matching_score` 话题发布的 Cartographer 扫描匹配分数连续 10 帧低于此值时，系统判定定位异常，进入等待状态。 |
| 取值说明 | 分数范围 0～1，值越高要求越严格。设定为 0.6 兼顾灵敏度与误触发率，在摔倒、搬离、定位漂移三类场景下均能稳定触发。 |

### 参数二：雷达数据波动阈值（`scan_change_threshold`）

| 项目 | 值 |
|------|----|
| 参数名 | `scan_change_threshold` |
| 当前设定值 | **0.5 m** |
| 作用 | 判断激光雷达信号是否已稳定的波动上限。进入等待状态后，系统对相邻两帧雷达扫描进行逐点差值计算（每 5 点采样一次），若连续 10 帧的平均差值均小于此阈值，则认为雷达信号已稳定，触发重定位执行。 |
| 取值说明 | 此参数用于过滤机器人仍在移动（如被搬运途中）的干扰状态，确保仅在机器人静止落地后再触发重定位，避免在运动过程中重定位失败。 |

### 参数三：重定位后不检测时长（`stabilizing_duration`）

| 项目 | 值 |
|------|----|
| 参数名 | `stabilizing_duration` |
| 当前设定值 | **10 秒** |
| 作用 | 重定位完成后进入稳定期的持续时长。在此期间，系统屏蔽匹配分数监控，避免 Cartographer 在轨迹初始化阶段分数尚未收敛时误触发新一轮重定位。稳定期结束后自动返回正常监控状态。 |
| 取值说明 | 设定为 10 秒，为 Cartographer 完成全局约束搜索、位姿图优化并输出稳定位姿留出充足余量，防止重复触发重定位。 |

---

## 4. 测试场景与结果

地图已预先建立并以 `.pbstream` 格式加载，以纯定位模式运行。

### 4.1 场景一：机器人摔倒检测与恢复

**测试描述**：机器人正常导航过程中，人为使机器人侧翻倒地，观察系统检测与恢复行为。

**触发过程**：
- 机器人摔倒后雷达扫描平面发生畸变，扫描匹配分数骤降至 0.6 以下；
- 连续 10 帧低分后系统进入 `STATE_WAITING`；
- 机器人静止倒地后雷达帧间差值降至 0.5 m 以下，连续 10 帧稳定后触发重定位。

**测试结果**：

| 测试次序 | 重定位时间 | 是否满足目标（5～12 s）|
|----------|-----------|----------------------|
| 第 1 次  | 6.5 s     | ✓ 满足 |
| 第 2 次  | 5.8 s     | ✓ 满足 |
| 第 3 次  | 7.2 s     | ✓ 满足 |
| 平均     | 6.5 s     | ✓ 满足 |

---

### 4.2 场景二：机器人被搬离检测与恢复

**测试描述**：机器人被人为抬起并搬至房间内另一位置放下，观察系统检测与恢复行为。

**触发过程**：
- 被搬运期间机器人位置快速偏移，匹配分数持续低于 0.6；
- 搬运过程中雷达帧间差值持续超过 0.5 m，系统在 `STATE_WAITING` 等待雷达稳定；
- 机器人落地静止后雷达数据恢复稳定，触发重定位。

**测试结果**：

| 测试次序 | 重定位时间 | 是否满足目标（5～12 s）|
|----------|-----------|----------------------|
| 第 1 次  | 8.3 s     | ✓ 满足 |
| 第 2 次  | 7.0 s     | ✓ 满足 |
| 第 3 次  | 9.1 s     | ✓ 满足 |
| 平均     | 8.1 s     | ✓ 满足 |

> **说明**：重定位时间从机器人落地、系统触发 `do_recovery()` 起计，不包含搬运过程中的等待时间。

---

### 4.3 场景三：运行中定位丢失检测与恢复

**测试描述**：机器人正常导航过程中，通过环境变化使匹配分数持续下降至阈值以下，观察系统自动恢复行为。

**触发过程**：
- 定位漂移导致匹配分数持续低于 0.6；
- 机器人静止后雷达数据稳定，系统触发重定位；
- 此场景下环境变化较小，重定位收敛较快。

**测试结果**：

| 测试次序 | 重定位时间 | 是否满足目标（5～12 s）|
|----------|-----------|----------------------|
| 第 1 次  | 5.5 s     | ✓ 满足 |
| 第 2 次  | 6.8 s     | ✓ 满足 |
| 第 3 次  | 8.0 s     | ✓ 满足 |
| 平均     | 6.8 s     | ✓ 满足 |

---

## 5. Cartographer 定位配置参数（`livox_2d_localization.lua`）

| 参数 | 数值 | 说明 |
|------|------|------|
| `linear_search_window` | 0.3 m | 前端实时扫描线性搜索窗口（扩大以应对大位移）|
| `angular_search_window` | ±15° | 前端角度搜索窗口 |
| `fast_correlative_scan_matcher.linear_search_window` | 8.0 m | 后端全局重定位线性搜索范围 |
| `fast_correlative_scan_matcher.angular_search_window` | ±45° | 后端全局重定位角度搜索范围 |
| `max_submaps_to_keep` | 2 | 纯定位模式保留子图数量 |
| `optimize_every_n_nodes` | 15 | 位姿图优化频率（节点数）|
| `global_localization_min_score` | 0.40 | 全局重定位最低匹配分数 |
| `voxel_filter_size` | 0.08 m | 体素滤波尺寸（降低点云密度提升速度）|
| `num_background_threads` | 4 | 后台处理线程数 |

---

## 6. 重定位时间汇总

| 测试场景 | 最短重定位时间 | 最长重定位时间 | 平均重定位时间 | 是否满足 5～12 s 目标 |
|----------|--------------|--------------|--------------|----------------------|
| 机器人摔倒       | 5.8 s | 7.2 s | 6.5 s | ✓ 满足 |
| 机器人被搬离     | 7.0 s | 9.1 s | 8.1 s | ✓ 满足 |
| 运行中定位丢失   | 5.5 s | 8.0 s | 6.8 s | ✓ 满足 |
| **综合**         | **5.5 s** | **9.1 s** | **7.1 s** | **✓ 满足** |

---

## 7. 结论

本次测试验证了基于 Cartographer 守护节点（`cartographer_guard.py`）的异常检测与自动重定位方案在三类典型场景下的有效性：

1. **功能完整性**：摔倒、被搬离、定位丢失三种异常均可被可靠检测，并自动触发重定位，无需人工干预；
2. **重定位速度**：所有测试场景的重定位时间均在 **5～12 秒** 范围内（实测 5.5～9.1 秒），达成目标；
3. **参数有效性**：
   - `score_threshold = 0.6`：在三类场景下均能稳定触发，未出现漏检或误触发；
   - `scan_change_threshold = 0.5 m`：有效区分运动中与静止状态，保证仅在机器人静止后执行重定位；
   - `stabilizing_duration = 10 s`：充分覆盖 Cartographer 定位收敛所需时间，未出现重复触发问题。

**后续优化建议**：
- 可尝试引入 IMU 数据辅助摔倒检测，进一步缩短异常检测阶段时间；
- 在大场景环境下验证后端全局搜索窗口（8 m / ±45°）是否仍满足需求；
- 考虑将 `score_consecutive_frames` 与 `scan_stable_frames` 参数化暴露，以便针对不同场景快速调优。