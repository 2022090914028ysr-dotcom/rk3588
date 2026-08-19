# SCAN-Planner 仅使用窄视角深度相机总结

## 1. 是否可以只使用深度相机

可以。

SCAN-Planner 支持两种环境输入：

```text
sensor_type=lidar：输入 sensor_msgs/PointCloud2
sensor_type=depth：输入 sensor_msgs/Image 深度图
```

因此没有三维雷达时，可以使用深度相机运行 SCAN。

## 2. 窄视角带来的影响



窄视角深度相机会限制 SCAN 的有效感知范围：

- 只能观察机器人前方较小区域；
- 转弯前看不到新行驶方向的侧面障碍；
- 无法可靠观察机器人两侧和后方；
- 相机近距离盲区可能漏掉贴近机身的障碍；
- 快速转向时，规划可能暂时使用不完整的环境信息；
- 桌腿、细杆、玻璃、黑色物体可能产生深度缺失；
- 阳光、反光和强光环境可能降低深度图质量；
- 没有后向传感器时，自动倒车风险较高。

窄视角不会导致 SCAN 无法启动，但会降低侧向、转弯和后退时的避障可靠性。


SCAN 确实具有“局部地图记忆”，所以相机转过去后，之前看到的障碍不会立刻消失。
它采用概率占据栅格：
看到障碍时，通过 p_hit=0.85 增强对应体素的占据概率。
射线确认空间为空时，通过 p_miss=0.30 逐渐降低占据概率。
障碍离开相机视野后，只要没有被新的空闲观测清除，仍会保留在局部地图中。
默认滑动地图大小约为 10 × 10 × 5 m。
相关实现位于 [grid_map.cpp (line 306)](/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_env/src/grid_map.cpp:306)，参数位于 [advanced_param.xml (line 41)](/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml:41)。
但它不是长期全局记忆：
从未被相机看到的侧面障碍无法记忆。
障碍移出滑动地图范围后，对应体素会被清除。
定位漂移会导致已记忆障碍位置偏移。
新的空闲射线会逐渐清除旧障碍。
深度值为零时，当前代码会按远距离空闲射线处理，因此连续深度空洞可能清除已有障碍记忆。

## 3. 推荐传感器组合

推荐组合：

```text
深度相机  -> SCAN 三维局部路径规划
二维雷达  -> NavOA 360°局部避障
超声波    -> 近距离急停保护
```

在该组合中：

- SCAN 根据深度图生成前方局部路径；
- NavOA 使用 `/scan` 检测前方和侧方动态障碍；
- `vel_filter` 根据二维雷达和超声波进行最后一级速度保护；
- 即使深度相机暂时没有看到侧面障碍，NavOA 仍可在执行阶段减速或避让。

如果只有深度相机，没有二维雷达和超声波，应采用更低速度，并禁止自动倒车。

## 4. SCAN 深度相机话题配置

修改文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

配置：

```xml
<arg name="sensor_type" default="depth"/>
<arg name="depth_topic" default="/实机深度图话题"/>
<arg name="sensor_pose_topic" default="/实机深度相机位姿话题"/>
```

消息类型：

```text
depth_topic：sensor_msgs/Image
sensor_pose_topic：nav_msgs/Odometry
```

启动示例：

```bash
roslaunch scan_planner run.launch \
  is_real_world:=true \
  sensor_type:=depth \
  controller_mode:=none \
  body_pose_topic:=/实机车体里程计话题 \
  sensor_pose_topic:=/实机深度相机位姿话题 \
  depth_topic:=/实机深度图话题 \
  world_frame:=实机定位父坐标系
```

## 5. 相机内参必须修改

修改文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/run.launch
```

以下参数必须替换为实机深度相机内参：

```xml
<arg name="cx" default="实机相机 cx"/>
<arg name="cy" default="实机相机 cy"/>
<arg name="fx" default="实机相机 fx"/>
<arg name="fy" default="实机相机 fy"/>
```

内参通常可从以下话题获得：

```bash
rostopic echo -n1 /实机相机信息话题
```

相机信息话题类型应为：

```text
sensor_msgs/CameraInfo
```

如果深度图经过裁剪、缩放或对齐，必须使用处理后深度图对应的内参，不能继续使用原始
彩色图或其他分辨率的内参。

## 6. 推荐 SCAN 参数

修改文件：

```text
/home/ubuntu/Robit_SlamNav/NavOA_ws/src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml
```

首次实机测试建议：

```xml
<arg name="max_vel" default="0.20"/>
<arg name="max_acc" default="0.20"/>
<arg name="planning_horizon" default="2.0"/>

<param name="grid_map/depth_filter_maxdist" value="2.5"/>
<param name="grid_map/depth_filter_mindist" value="0.25"/>
<param name="grid_map/depth_filter_margin" value="2"/>
<param name="grid_map/k_depth_scaling_factor" value="1000.0"/>
<param name="grid_map/skip_pixel" value="2"/>
```

参数原则：

```text
planning_horizon 必须小于深度相机的实际可靠测距范围
```

例如：

```text
深度相机标称范围：5.0 m
现场可靠范围：2.5 m
推荐 planning_horizon：1.5～2.0 m
```

不能因为相机标称测距为 5 米，就直接使用 5 米规划范围。应以实际环境中的稳定深度范围
为准。

## 7. 深度编码与缩放系数

需要检查深度图编码：

```bash
rostopic echo -n1 /实机深度图话题/encoding
```

常见情况：

```text
16UC1：通常以毫米表示深度，k_depth_scaling_factor 使用 1000.0
32FC1：通常以米表示深度，SCAN 内部会转换，仍需实机验证
```

检查方法：在已知距离放置平面障碍物，确认 SCAN 中显示的障碍距离与真实距离一致。

## 8. 深度相机安装要求

- 相机朝机器人主要行驶方向安装；
- 相机应适当向下倾斜，使画面同时覆盖近处地面和前方障碍；
- 不能被机器人外壳、机械臂或线缆遮挡；
- 相机视野应覆盖机器人宽度两侧；
- 相机安装必须牢固，不能在运行中改变外参；
- `world_frame -> camera_frame` TF 必须正确；
- 深度图和相机位姿时间戳应尽量同步。

如果机器人需要频繁横移或急转，建议增加左右侧相机或改用广角深度相机。

## 9. NavOA 和速度保护建议

NavOA 首次实机测试建议：

```text
ref_speed：0.15～0.20 m/s
segment_length：0.6～0.8 m
reach_tolerance：0.30～0.35 m
```

`vel_filter` 建议：

```text
max_forward_speed：0.20 m/s
max_angular_speed：0.40～0.50 rad/s
enable_forward_safety：true
enable_auto_reverse：false
```

没有后向传感器时必须关闭：

```xml
<param name="enable_auto_reverse" value="false"/>
```

## 10. 实机测试顺序

第一阶段：机器人不使能，只检查输入。

```bash
rostopic hz /实机深度图话题
rostopic hz /实机深度相机位姿话题
rostopic echo -n1 /实机深度图话题/encoding
```

第二阶段：检查 SCAN 输出。

```bash
rostopic hz /planning/bspline
rostopic hz /planning/path
```

第三阶段：低速直线测试。

- 速度限制为 `0.15 m/s`；
- 只测试前方静态大障碍；
- 确认深度方向、距离和坐标系正确；
- 确认障碍出现后路径会改变。

第四阶段：低速转弯测试。

- 在左右侧放置软质障碍；
- 检查相机转向后地图更新速度；
- 检查 NavOA 的二维雷达避障是否生效；
- 确认机器人不会向相机盲区持续运动。

## 11. 结论

仅使用窄视角深度相机可以运行 SCAN-Planner，但应满足：

```text
降低速度
缩短规划范围
正确配置相机内参
正确配置深度缩放
保证相机位姿和 TF 正确
禁止无后向感知情况下自动倒车
```

更可靠的实机方案仍然是：

```text
深度相机负责 SCAN 前方三维规划
二维雷达负责 NavOA 周向局部避障
超声波负责近距离急停
```
