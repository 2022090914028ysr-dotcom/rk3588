-- =============================================================================
-- livox_2d_localization_warmup.lua
-- 预热模式：使用不同的 TF frame 名，避免与正在运行的旧节点冲突。
-- 发布  map_warmup → odom_warmup，由 tf_frame_switch 节点在切换后转发。
-- =============================================================================
include "livox_2d_localization.lua"

-- 覆盖 frame 名为预热专用名
options.map_frame = "map_warmup"
options.published_frame = "odom_warmup"
options.odom_frame = "odom_warmup"
