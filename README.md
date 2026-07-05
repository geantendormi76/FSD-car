# 🚀 FSD-car: 极致纯视觉具身智能与自动驾驶控制栈 (Resolute v2026)

`FSD-car` 是一个专为百元级超低成本 AMR（自主移动机器人）量身定制的、**全栈白盒纯视觉自动驾驶控制栈**。

我们彻底抛弃了高功耗的重型 3D 占据网络、高成本激光雷达与高精度定位仪，转而运用 **仿生学（青蛙眼时空感受野）**、**控制理论（acados NMPC）** 以及 **轻量深度局部特征（XFeat 亚像素级自愈显微镜）**，在极低算力（如百元级瑞芯微/香橙派）和强干扰下，基于 **DORA 零拷贝架构**，实现硬实时避障、确定性时钟同步与时空位置自愈。

---

## 🗺️ 全生命周期开发路线图 (AMR Roadmap)

本项目严格按照以下五个阶段进行闭环验证与物理演进，当前已实现 **阶段三与阶段四的 100% 闭环并网与物理绿通**：

```text
【 阶段一：离线控制验证 】 ──────► 【 阶段二：神经通路分布式组网 】 ──────► 【 阶段三：空间机能感知 】
   acados NMPC 数学台架闭环          DORA 零拷贝共享内存物理并网           100Hz 仿生青蛙眼 + XFeat 亚像素自愈
                                                                                   │
【 阶段五：自主探索与实车落地 】 ◄──── 【 阶段四：物理时钟确定性锁定 】 ◄───────────┘
  真车装配、Mahony姿态、纠偏自愈        RenderingManager 100Hz 锁定步进
```

---

## 📂 项目工作空间物理地图 (Workspace Map)

本主干仓库基于 Rust 2021/2024 Edition 强类型契约，采用高内聚、低耦合的多包工作空间进行管理，去噪后的核心组件分布如下：

```text
FSD-car/
├── .gitignore                      # 统一大资产与编译缓存拦截契约
├── repomix.config.json             # AI 上下文扫描极致优化配置文件
├── dora_dataflow.yaml              # DORA 分布式零拷贝数据流拓扑蓝图
├── core-perception/                # ──【视觉感知包】──
│   └── src/perception/
│       ├── frog_eye.rs             # 30Hz 兴奋/抑制感受野本能避障势场
│       ├── matcher.rs              # 仿生匹配器（双通道亚像素纠偏显微镜 + 二次型梯度自愈算子）
│       └── xfeat_engine.rs         # XFeat 特征提取器（IEEE 754 浮点指数黑客 + 1D 扁平化 NMS 压制）
├── core-control/                   # ──【控制规控包】──
│   └── src/
│       ├── ffi.rs / solver.rs      # acados C-FFI 内存胶囊安全封装与 NMPC 求解器
│       ├── sensor_fusion.rs        # 层级多传感器融合标定（100Hz 物理小脑 + 1Hz 视觉互补纠偏烧录）
│       └── bin/fast_brain_node.rs  # 快系统 NMPC 控制环路节点 (100Hz)
├── core-decision/                  # ──【语义决策包】──
│   └── src/
│       ├── topo_graph/             # 空间拓扑地标节点记忆与 A* 寻路
│       └── bin/slow_brain_node.rs  # 慢系统拓扑建图与决策节点 (1Hz / Arrow 零拷贝接收)
├── showcase/                       # ──【可视化验证展示包】──
│   └── src/bin/
│       ├── demo_frog.rs            # 仿生感受野 2D 动态势场热力图大屏
│       ├── demo_xfeat.rs           # XFeat 视觉纠偏与 MNN 双通道实时对极画线沙盘
│       └── perception_sandbox.rs   # 感知引擎离线时序台架验证沙盘
└── simulation-env/                 # ──【NVIDIA 2026 物理代理环境】──
    ├── python.sh                   # -> 物理隔离的 GPU RTX 渲染执行器入口
    ├── isaac_dora_node.py          # RenderingManager 100Hz 确定性硬锁时钟物理代理
    ├── nmpc_model.py               # 车辆连续时间动力学常微分方程 (ODE) 建模
    └── generate_solver.py          # acados RTI-SQP 求解器 C 代码自动生成器
```

---

## 🛠️ 快速编译运行说明 (Quick Start)

本项目完全在 **Ubuntu 26.04 LTS (Resolute Raccoon)** 黄金沙盘环境下编译与运行，核心执行器路径绑定于 `/home/zhz/isaacsim/python.sh`。

### 1. 运行规控/感知底层单元验证 (Mathematics & Memory Check)
在 workspace 根目录下，执行高难度物理与内存越界验证：
```bash
# 验证阶段三：高维空间 L2 归一化泰勒展开极值自愈算子精度
cargo test -p core-perception test_parabolic_sub_pixel_interpolation_math -- --nocapture

# 验证阶段二：慢系统 Arrow 内存布局零拷贝向上/下转型与列存储映射
cargo test -p core-decision test_arrow_struct_array_zero_copy_deserialization -- --nocapture

# 验证阶段一：NMPC 求解器 FFI 内存胶囊分配、状态注入及 1000 次求解稳定性
cargo test -p core-control test_nmpc_solver_lifecycle_and_compute -- --nocapture
```

### 2. 生成并编译 NMPC 求解器 (Acados Code-Gen)
利用 Isaac Sim 内置的高性能 Python 环境生成最新的 C 语言 NLP 求解器代码：
```bash
cd simulation-env
/home/zhz/isaacsim/python.sh generate_solver.py
```

### 3. 一键启动数字孪生并网 (Launch DORA Flow)
在项目根目录下，启动 DORA 大总管协调器，拉起 100Hz 的确定性仿真控制闭环：
```bash
dora up
dora start dora_dataflow.yaml
```

---

## 💎 系统设计规范 (System Coding Standards)

1. **第零法则**：全系统严格执行 **Apache Arrow 列式内存对齐与零拷贝指针分发**，拒绝任何手动的 `struct.pack` 字节流编解码开销。
2. **时钟主权**：所有仿真物理节拍必须通过 `RenderingManager.set_dt(0.01)` 绑定，让 RTX 渲染、PhysX 线程在 DORA 事件流下处于**傀儡式手动步进模式**，消除一切时空滑移。
3. **内存守卫**：所有的 C-FFI 接口生命周期必须由 Rust 强类型结构体的 `Drop` 契约代管，严禁泄露 C 堆内存。

---

## 📚 学术文献引用 (Academic References)

本项目的设计哲学与底层算子深度对齐并参考了以下 **4 篇** 机器人、计算机视觉及分布式神经系统领域的顶级学术/顶会文献：

1. **DORA: Dataflow Oriented Robotic Architecture** (arXiv:2602.13252)
   * **作者**: Xiaodong Zhang, Baorui Lv, Xavier Tao, Xiong Wang, Jie Bao, Yong He, Yue Chen, Zijiang Yang
   * **系统价值**: 本项目底层“去中心化零拷贝共享内存”与“声明式数据流拓扑图”的设计源头。
   * **文献链接**: [arXiv:2602.13252](https://arxiv.org/abs/2602.13252)

2. **XFeat: Accelerated Features for Lightweight Image Matching** (CVPR 2024)
   * **作者**: Guilherme Potje, Felipe Cadar, Andre Araujo, Renato Martins, Erickson R. Nascimento
   * **系统价值**: 慢系统“亚像素纠偏显微镜”及快系统“轻量化特征提取”双三次权重插值算子设计的算法源头。
   * **文献链接**: [arXiv:2404.19174](https://arxiv.org/abs/2404.19174)

3. **Minimalist Visual Inertial Odometry** (arXiv:2605.19990)
   * **作者**: Francesco Pasti, Jeremy Klotz, Nicola Bellotto, Shree K. Nayar
   * **系统价值**: 支撑我们层级传感器融合中心（`sensor_fusion.rs`）在无图环境下执行超低功耗非霍洛诺姆车辆死步累积的运动学基石。
   * **文献链接**: [arXiv:2605.19990](https://arxiv.org/abs/2605.19990)

4. **Fully Autonomous Neuromorphic Navigation and Dynamic Obstacle Avoidance** (NeurIPS 2025)
   * **作者**: Xiaochen Shang, Pengwei Luo, Xinning Wang, Jiayue Zhao, Huilin Ge, Bo Dong, Xin Yang
   * **系统价值**: 启发我们仿生避障（`frog_eye.rs` 兴奋/抑制感受野）端到端动态势场解算及微秒级反射弧控制的仿生学灵感源头。
   * **文献链接**: [NeurIPS 2025 PDF](https://papers.neurips.cc/paper_files/paper/2025/file/50ee6db59fca8643dc625829d4a0eab9-Paper-Conference.pdf)
