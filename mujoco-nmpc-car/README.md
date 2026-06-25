# 🚀 FSD-car: 极简纯视觉具身智能与自动驾驶控制栈 (仿真验证版)

本仓库是一个基于 **acados 非线性模型预测控制 (NMPC)** 与飞控级 **Mahony 6轴姿态估计滤波器** 实现的微型智驾底盘闭环控制仿真系统。致力于探索“极轻量、高精度、低硬件成本”的通用具身智能运动控制路径。

基于 **MuJoCo** 物理引擎与 **WSLg 3D 渲染**，本项目在强传感器噪声与车轮打滑的对抗干扰下，实现了厘米级的平顺控车与无漂移姿态解算。

---

## ✨ 核心亮点

- **小脑对齐 (Mahony AHRS)**：从飞控降维移植的 6 轴互补姿态解算算法，在注入高斯噪声的恶劣环境下，将航向估计误差死死控制在 1° 以内。
- **大脑决策 (acados NMPC)**：基于 RTI-SQP 极速实时迭代的非线性模型预测控制，单步求解耗时控制在超低的 0.6ms 内。
- **物理平顺控制**：通过 PT1 低通控制阀与电机力矩物理幅值限制，彻底消除了物理引擎中常见的碰撞共振与高频颤抖。

---

## 🛠️ 快速开始：三步在你的 PC 上 100% 复刻

本项目完全基于 **Windows + WSL2 (Ubuntu 22.04)** 黄金开发环境运行，无需任何实体硬件。

### 第一步：在 WSL2 中部署 acados C 语言核心库

打开你的 WSL2 终端，一键编译并安装 `acados`：

```bash
cd ~
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OPENMP=OFF ..
make install -j4
```

将以下环境变量写入你的 `~/.zshrc` 或 `~/.bashrc` 中：

```bash
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/acados/lib"
```

### 第二步：安装 Python 依赖

```bash
pip install -e ~/acados/interfaces/acados_template
pip install casadi setuptools mujoco numpy matplotlib
```

### 第三步：运行闭环仿真

1. 克隆本仓库：
```bash
git clone https://github.com/geantendormi76/mujoco-nmpc-car
cd mujoco-nmpc-car
```

2. 运行代码生成器，在本地编译生成 NMPC C 语言求解器：
```bash
python3 generate_solver.py
```

3. 启动 3D 闭环物理仿真：
```bash
python3 mujoco_nmpc_run.py
```

---

## 📊 数字化遥测日志说明

仿真启动后，系统会自动在本地建立 `nmpc_telemetry.log` 数据日志。每 50 步 (即 0.5 秒) 会记录一次核心数据，包含以下关键白盒审计指标：

1. `L2_Error_m`：小车重心与目标参考线之间的绝对几何距离误差。
2. `Yaw_Est_Error_deg`：Mahony 估计姿态与真实物理姿态的偏差度数。
3. `SolveTime_ms`：NMPC 单步 C 语言 RTI-SQP 求解耗时。
