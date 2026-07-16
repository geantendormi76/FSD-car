# FSD-car

FSD-car 是面向低速室内仓库 AMR 的自动驾驶研究工程。当前经过白盒验证的主链为：

```text
RGB + 公制深度
  -> warehouse_nav14 语义感知
  -> depth-lift 192x192 BEV
  -> acados NMPC
  -> Dora safety supervisor / watchdog
  -> Isaac articulation（仿真）
```

项目已经形成仿真 release candidate，但尚未完成真机验收。全局 A* 和仿真独立碰撞
监督仍使用 USD Oracle；真实相机、米制定位、真实地图、独立深度 guard 和执行器尚未
接入。因此 `real_vehicle_control_allowed=false`。

## 已验证结果

- Phase 2 证明 Cityscapes PIDNet 在原仿真域发生退化，ROI 假占用率接近 100%。
- warehouse_nav14 候选模型完成 1,000 帧影子门禁和受控接管。
- Phase 5-K 完成 72,000 帧、约一小时耐久与进程强杀、GPU OOM、磁盘写满等故障注入。
- Phase 6 完成 3 个种子 x 4 个场景：`12/12` 到达、零碰撞、零求解失败，最差
  sensor-to-wheel p95 为 `33.06 ms`。
- Phase 7 无 Isaac 部署画像：控制主链 p95 `17.07 ms`，推荐 Jetson Orin NX 16GB。

聚合数值见 [`docs/evidence/final_metrics.json`](docs/evidence/final_metrics.json)，完整接手说明
见 [`HANDOFF.md`](HANDOFF.md)。

## 当前组件状态

| 组件 | 状态 | 说明 |
|---|---|---|
| warehouse_nav14 ONNX | active | 320x240，20 Hz 仓库语义感知 |
| metric depth-to-BEV | active | 192x192，20m x 20m，未知区域 fail-closed |
| acados NMPC | active | 20 Hz，低速差速车局部控制 |
| XFeat ONNX | partial | 640x640，2 Hz；仅有特征证据，尚无米制位姿融合 |
| Dora safety/watchdog | active in simulation | 急停锁存、启动预热、超时归零、故障恢复 |
| Spiced BC/PPO | inactive | 未超过 BC 基线，不进入主控制链 |
| PIDNet Cityscapes | inactive | 与仓库部署域不匹配 |
| 真机控制 | blocked | 六项物理门禁均未完成 |

## 仓库结构

```text
core-perception/       Rust 感知与 XFeat 基础组件
core-control/          Rust 快控制、安全检查与 NMPC 接口
core-decision/         拓扑图和慢速决策实验代码
simulation-env/        Isaac、数据采集和 acados 求解器生成入口
contracts/v1/          数据、坐标系和 Dora 流契约
contracts/phase3-7/    仿真感知、控制、鲁棒性和部署验收
contracts/real_vehicle 真机准入门禁（当前 blocked）
model/                 仅分发两个 active ONNX 及哈希清单
scripts/               仓库卫生检查
```

`brain/spiced_rl_trainer`、`dora_dataflow_spice_bc*.yaml` 和相关 probe 保留为历史实验与
回归依据，不属于当前主链。

## 环境

冻结验证环境：Ubuntu 26.04、Rust 1.96.1、Dora 0.3.13、uv 0.11.26、Isaac Sim
Python 3.12。Isaac Sim 和 acados 不进入 Git，需要按 [`HANDOFF.md`](HANDOFF.md) 配置。

复制本地环境模板：

```bash
cp .env.example .env
```

## 快速验证

不启动 Isaac 的检查：

```bash
python3 scripts/check_repo_hygiene.py
python3 -m unittest discover -s contracts/real_vehicle -p 'test_*.py' -v
PYTHONPATH=contracts/phase5 /home/zhz/isaacsim/python.sh \
  -m unittest discover -s contracts/phase5 -p 'test_*.py'
python3 contracts/phase6/validate_phase6.py
python3 contracts/phase7/validate_phase7.py
```

最后两个状态验证器需要本地原始 `artifacts/`。GitHub 只保存聚合数值、状态和哈希，不保存
原始帧、CSV、私有数据集或故障注入大日志。

## 模型

Git 中只发布：

- `model/warehouse_nav14_candidate.onnx`
- `model/xfeat_640x640.onnx`

运行前检查 [`model/DELIVERY.json`](model/DELIVERY.json) 中的 SHA-256。PIDNet、Spiced
Brain、BC/PPO checkpoint 均为 inactive，不作为交付依赖。

## 安全边界与下一步

真机开发必须依次完成：真实 RGB-D 标定、轮速/IMU/XFeat 米制定位、真实 SLAM 地图、
不依赖学习语义的独立深度停车 guard、编码器闭环和物理急停。详细阈值与操作见
[`contracts/real_vehicle/README.md`](contracts/real_vehicle/README.md)。在六项门禁全部通过前，
不得向落地车轮发送自主驾驶命令。
