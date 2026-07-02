# 🚀 FSD-car: 极简纯视觉具身智能与自动驾驶控制栈 

`FSD-car` 是一个专为百元级超低成本 AMR（自主移动机器人）量身定制的、**全栈白盒纯视觉自动驾驶控制栈**。

我们彻底抛弃了高功耗的重型 3D 占据网络、高成本激光雷达与高精度定位仪，转而运用 **仿生学（青蛙眼时空感受野）**、**控制理论（acados NMPC）** 以及 **轻量深度局部特征（XFeat）**，在极低算力（如百元级瑞芯微/香橙派）和强干扰下，实现硬实时避障与时空位置自愈。

---

## 🗺️ 全生命周期开发路线图 (AMR Roadmap)

本项目严格按照以下五个阶段进行闭环验证与物理演进：

```text
【 阶段一：虚拟沙盘验证 】 ──────► 【 阶段二：神经通路分布式组网 】 ──────► 【 阶段三：空间积木感知 】
 MuJoCo + acados 极速控制闭环       ESP32-C6 + Zenoh 200Hz 串口并网       仿生青蛙眼2D势场 + XFeat稀疏快照
                                                                                   │
【 阶段五：自主探索与实车落地 】 ◄──── 【 阶段四：大小脑行为树决策 】 ◄─────────────┘
  真车装配、Mahony姿态、纠偏自愈        Rust Forester 行为树 + 语义拓扑图
```

---

## 📂 项目工作空间物理地图 (Workspace Map)

本主干仓库采用高内聚、低耦合的多包工作空间进行管理，每个阶段的代码完全独立：

```text
FSD-car/
├── .gitignore                      # 统一大资产与编译缓存拦截契约
├── test_frog.py                    # 仿生感受野 Python 验证探针
├── mujoco-nmpc-car/                # ──【阶段一】仿真沙盘 ──
│   ├── mujoco_nmpc_run.py          # MuJoCo NMPC 闭环仿真主程序
│   └── generate_solver.py          # acados C 语言 RTI-SQP 求解代码生成器
├── zenoh-rs/                       # ──【阶段二】神经通路 ──
│   ├── Cargo.toml                  # 依赖配置
│   └── src/bin/
│       ├── brain.rs                # 慢系统大脑发送端 (Rust)
│       └── spinal_cord_mock.rs     # 脊髓接收端 (ESP32-C6 模拟器)
├── amr_bionic_eye/                 # ──【阶段三】本能避障 ──
│   └── src/                        # 30Hz 乒乓缓冲、信箱模式极速势场感知
└── xfeatc/                         # ──【阶段三.五】稀疏纠偏 ──
    ├── model/                      # 存放 xfeat_640x640.onnx 神经网络权重
    ├── lib_dylib/                  # 本地物理链接 libonnxruntime.so.1.18.0
    └── src/                        # XFeat 提取器、双向余弦匹配器、RANSAC 纠偏算子
```

---

## 🛠️ 快速编译运行说明 (Quick Start)

本项目完全在 **Windows + WSL2 (Ubuntu 22.04) + Zsh** 黄金沙盘环境下跑通。

### 1. 运行阶段一 (MuJoCo 动力学与控制)
在 WSL2 编译安装好 `acados` 核心库后，进入 `mujoco-nmpc-car` 目录：
```bash
cd mujoco-nmpc-car
python3 generate_solver.py  # 自动生成 C 语言 MPC 控制器
python3 mujoco_nmpc_run.py  # 启动物理仿真
```

### 2. 运行阶段三 (仿生青蛙眼避障)
确保您的手机通过 `IP Webcam` App 在同一局域网并网：
```bash
cd amr_bionic_eye
cargo run --release          # 自动启动单元素覆盖信箱（Mailbox），0延迟输出
```

### 3. 运行阶段三.五 (XFeat 稀疏快照位置自愈)
由于国内网络阻断，本项目采用**本地物理库链接机制**。请在编译前确保环境变量就绪：
```bash
cd xfeatc
# linux系统版本落后，可以注入本地动态库环境变量
export ORT_LIB_PATH=$(pwd)/lib_dylib
export ORT_PREFER_DYNAMIC_LINK=1
export LD_LIBRARY_PATH=$(pwd)/lib_dylib

cargo run --release          # 自动录制站牌，移动摄像头体验 RANSAC 自愈画线！
```
