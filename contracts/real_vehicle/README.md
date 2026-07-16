# 真机准入验收

## 当前结论

当前状态是 `real_vehicle_gate_blocked`，不是软件失败，而是硬件尚未存在时应有的
fail-closed 结果。六个门禁全部明确阻塞，真实车辆控制保持禁止。

执行当前审计：

```bash
python3 contracts/real_vehicle/audit_real_vehicle_readiness.py \
  --output artifacts/real_vehicle_acceptance/pre_hardware_20260716_182135
python3 contracts/real_vehicle/validate_pre_hardware.py
```

原始结果位于
`artifacts/real_vehicle_acceptance/pre_hardware_20260716_182135/summary.json`。

## 为什么仿真通过仍不能上真车

1. **拓扑**：`dora_dataflow.yaml` 的相机、深度和里程计生产者仍是
   `isaac_sim_env`。它是仿真拓扑，不是车载拓扑。
2. **全局规划**：Phase 5-H/Phase 6 使用从 USD 构建的 `OracleGrid`，A* 在这张
   真值地图上规划。真车必须改用现场 SLAM 生成并冻结哈希的占据地图。
3. **碰撞监督**：仿真的独立裁判可以直接查询 USD 几何是否重叠；现实世界没有这个
   接口。真车必须增加不依赖学习语义输出的原始公制深度 guard 或独立测距传感器。
4. **定位**：当前 XFeat 能做匹配、RANSAC 和单应性估计，但估计结果没有尺度，也没有
   写回米制 `(x,y,yaw)`。它只能说明“画面像某个地方”，还不是可供 A* 使用的定位器。
5. **执行器**：现有闭环证据来自 Isaac articulation，没有真实编码器、驱动器、轮径、
   轮距、死区、反向符号、watchdog 或物理急停数据。
6. **相机**：当前主机没有 `/dev/video*`，也没有真实相机内参、畸变、外参和 depth
   scale 证据。

## 六项准入门禁

### 1. 车载拓扑

- 运行时不得出现 `isaac_sim_env`、`simulation-env`、`usd` 或 `oracle` 控制依赖。
- 全局地图来源必须声明为 `real_slam_occupancy_map`。
- 独立碰撞源必须是 `independent_metric_depth_guard`，不能复用 warehouse_nav14
  的最终语义 BEV。

这相当于考试时不能让控制器和监考员拿同一份答案：学习感知出错时，独立深度 guard
仍需根据原始距离让车辆停车。

### 2. 真机相机

沿用冻结的 Phase 5-E 标定门禁：至少 20 张棋盘格、覆盖 3x3 图像分区中的至少 6 格、
重投影 RMS 不超过 0.5 px、单视图 p95 不超过 0.8 px；相机到车体平移不确定度不超过
2 mm、旋转不确定度不超过 0.5 度；RGB/深度配对率至少 0.99，深度尺度相对误差不超过
2%。

相机到货后执行：

```bash
python3 contracts/phase5/phase5e_real_calibration_audit.py \
  --calibration artifacts/real_camera/calibration.json
```

### 3. 定位

- 连续运行至少 600 秒。
- ATE RMSE 不超过 0.10 m。
- 平移 RPE p95 不超过 0.08 m，航向误差 p95 不超过 3 度。
- 定位丢失比例不超过 1%。
- 人为遮挡后重定位成功率至少 95%，重定位 p95 不超过 2 秒。

评估必须使用独立真值，例如测量过的 AprilTag/ArUco 地标或运动捕捉；不能拿轮式里程计
同时做预测和真值。XFeat 应作为观测，和轮速/IMU 在带时间戳的滤波器或因子图中融合。

### 4. 真实地图与 A*

- 使用现场 SLAM 占据地图并保存地图哈希、分辨率和坐标原点。
- 至少覆盖 8 条不同起终点路线，规划成功率必须为 100%。
- 路径中落入未知或占据栅格的路点必须为 0。
- 地图变更后必须重新冻结哈希并重跑路线集，不能沿用 USD 坐标。

### 5. 独立碰撞监督

- 至少 40 个真实正/负场景，包括正前方、偏置、窄通道、低反射和突然横穿。
- 停车召回率至少 99%，放行特异度至少 95%。
- 最差情况下停车后与障碍物仍需保留至少 0.10 m。
- 证据中 `oracle_used` 必须为 `false`。

第一轮必须在低速、封闭区域、系绳和人工急停条件下完成。独立 guard 输出只允许
“放行/减速/停车”，不得生成导航方向。

### 6. 执行器

先架空车轮测试，再落地低速测试：

- 左右轮和正反方向符号全部正确。
- 线速度跟踪误差 p95 不超过 0.08 m/s。
- 角速度跟踪误差 p95 不超过 0.12 rad/s。
- watchdog 与物理急停 p95 均不超过 150 ms。
- 零命令残余爬行不超过 0.01 m/s。

电机 MCU 必须在 Jetson/Dora 失联时独立归零。软件进程不能成为唯一急停边界。

## 硬件证据接入

硬件到货并生成统一 JSON 后，执行：

```bash
python3 contracts/real_vehicle/audit_real_vehicle_readiness.py \
  --topology dora_dataflow_real_vehicle.yaml \
  --evidence artifacts/real_vehicle_acceptance/hardware_evidence.json \
  --output artifacts/real_vehicle_acceptance/final
```

只有 summary 中六个 gate 全为 `true`、`blocked_gates` 为空时，准入逻辑才会把
`real_vehicle_control_allowed` 设为 `true`。验收器只做判定，不会自动启动电机。
