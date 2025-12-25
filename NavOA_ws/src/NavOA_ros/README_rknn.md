# RKNN 迁移与代码改动说明

## 目标
- 将 DUNE 子模型从 PyTorch CPU 推理迁移到 RK3588 NPU（rknn-toolkit2）。
- 保持回退能力：如未启用 RKNN 或运行失败，可继续使用原始 .pth 模型在 CPU 上推理。

## 关键改动一览

- 启动参数（新增 rknn 选择与核心控制）
  - 文件：`src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch`
  - 新增 arg/param：`use_rknn`（默认 true）、`dune_rknn`、`rknn_target`（默认 rk3588）、`rknn_core`（默认 auto），传递给节点。

- 节点层（读取 rknn 配置并透传）
  - 文件：`src/NavOA_ros/src/NavOA_core.py`
  - 新增读取 `~use_rknn`/`~dune_rknn`/`~rknn_target`/`~rknn_core`，并在启动时日志打印；将这些参数透传给 PAN/DUNE。

- PAN 层（参数下发）
  - 文件：`src/NavOA_ros/NavOA/NavOA/blocks/pan.py`
  - PAN 构造函数新增 `rknn_target`、`rknn_core`，向下传给 DUNE。

- DUNE 层（RKNN 推理与分块）
  - 文件：`src/NavOA_ros/NavOA/NavOA/blocks/dune.py`
  - 新增 RKNN 运行时初始化：`init_runtime(target=rk3588, core_mask=...)`，支持 core 选择（auto/0/1/2/0_1/0_1_2/all）。
  - 绝对路径支持：`dune_rknn` 为绝对路径时不再错误拼接。
  - 推理分块与填充：RKNN 模型固定每次 100 点，DUNE 会将点云按 100 分块、零填充推理，再按实际长度拼接回 `mu`，避免维度不匹配。
  - 调试日志：加载/初始化/每 50 次前向打印 use_rknn、ready 状态、输入输出尺寸，便于确认是否走到 NPU。

## CPU 版本 vs NPU 版本差异

- 模型加载
  - CPU：加载 `.pth` 到 PyTorch `ObsPointNet`，在 CPU/torch 运行。
  - NPU：加载 `.rknn`，通过 rknn-toolkit2 在 NPU 推理；若 `use_rknn=false` 或初始化失败，仍可使用 CPU 路径。

- 输入尺寸
  - CPU：原生支持任意点数的张量。
  - NPU：RKNN 模型固定每批 100 点；DUNE 现在自动分块、填充、裁剪，确保输出与输入点数对齐。

- 运行时选择
  - 通过 launch 参数 `use_rknn` 控制；`rknn_target` 强制目标为 rk3588，`rknn_core` 可指定核掩码（auto/0/1/2/0_1/0_1_2/all）。

- 日志与验证
  - CPU：正常 ROS 日志。
  - NPU：额外打印 `[NavOA] use_rknn=... target=... core=...`，`[DUNE][RKNN] loading model ...`，`[DUNE][RKNN] init_runtime target=... core=...`，以及每 50 次 `[DUNE][dbg] ...`。
  - NPU 负载可用 `watch -n 0.2 sudo cat /sys/kernel/debug/rknpu/load` 观察（需驱动与 debugfs）。

## 使用方法（RKNN 模式）

```bash
# 确保 rknn-toolkit2 安装在当前环境，并存在 /dev/rknpu*
roslaunch NavOA_ros NavOA_gazebo_limo.launch \
  use_rknn:=true \
  dune_rknn:=/home/ubuntu/NavOA_ws2/src/NavOA_ros/example/gazebo_limo/pretrain_limo/model_5000_quant.rknn \
  rknn_target:=rk3588 \
  rknn_core:=auto
```

- 若需强制单核，例如核1：`rknn_core:=1`。
- 若想回退 CPU：`use_rknn:=false`，并设置 `dune_checkpoint` 指向 `.pth`。

## 变更文件列表
- `src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch`
- `src/NavOA_ros/src/NavOA_core.py`
- `src/NavOA_ros/NavOA/NavOA/blocks/pan.py`
- `src/NavOA_ros/NavOA/NavOA/blocks/dune.py`

## 已知注意事项
- 确保扫描有有效障碍点；若点被过滤，DUNE/RKNN 不会被调用，NPU 负载可能为 0。
- 驱动与权限：需存在 `/dev/rknpu*`，`/sys/kernel/debug/rknpu/load` 需挂载 debugfs。
- rknn-core 掩码兼容性：旧版 toolkit 若不支持 `core_mask`，代码自动回退只传 target。
