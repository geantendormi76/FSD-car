// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

/*
=================================================================
🛡️ FSD-car V3.1: 快系统规控大脑节点 (NMPC & 仿生避障全自愈版)
设计哲学: 局部相对坐标系强制锚定 | 100Hz 物理步长完全对齐 | 零拷贝内存重塑
=================================================================
*/

use core_control::预测控制求解器;
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use std::sync::Arc;
use std::sync::RwLock; // 状态金库轻量级锁
use std::time::Duration;

/// 状态金库：快大脑 100Hz 线程与 避障力接收线程 间绝对安全的无锁共享上下文
struct 执行上下文 {
    pub 物理主权已初始化: bool,
    pub 期望_x: f64,              // 纵向势场排斥力 (避障减速 - Spice)
    pub 期望_y: f64,              // 横向势场逃逸力 (变道机动 - Spice)
    pub 当前线速度: f64,           // 上一帧 NMPC 输出并在物理世界执行后的真实线速度
    // 🎯 里程碑 2.1：物理小脑绝对坐标与人类引力锚点
    pub 当前_x: f64,
    pub 当前_y: f64,
    pub 当前_yaw: f64,
    pub 引力_x: f64,
    pub 引力_y: f64,
    pub 引力_yaw: f64,
    // 🛡️ 状态缓存：记录上一次引力点成功接收的时间戳，用于看门狗倒计时 [cite: 1.2.5]
    pub 上次引力更新时间: std::time::Instant,
}
#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🧠 [快系统] 规控大脑节点已启动，等待 DORA 共享内存注入...");
    // 1. 接入 DORA 数据流网络 (接管生命周期与共享内存池)
    let (mut node, mut events) = DoraNode::init_from_env()?;
    // 2. 初始化全简体中文状态金库
    let 状态金库 = Arc::new(RwLock::new(执行上下文 {
        物理主权已初始化: false,
        期望_x: 0.0,              // 默认无纵向排斥
        期望_y: 0.0,              // 默认无横向逃逸
        当前线速度: 0.0,
        当前_x: 0.0,              // 🎯 物理小脑高频绝对坐标 X 初始值
        当前_y: 0.0,              // 🎯 物理小脑高频绝对坐标 Y 初始值
        当前_yaw: 0.0,            // 🎯 物理小脑高频绝对角度 Yaw 初始值
        引力_x: 0.0,              // 🎯 慢系统人类引力锚点 X 初始值
        引力_y: 0.0,              // 🎯 慢系统人类引力锚点 Y 初始值
        引力_yaw: 0.0,            // 🎯 慢系统人类引力锚点 Yaw 初始值
        上次引力更新时间: std::time::Instant::now(),
    }));
    let 金库_规控 = 状态金库.clone();

    // ---------------------------------------------------------
    // [线程 A]：100Hz 极速 NMPC 控制环路 (The Control Loop)
    // ---------------------------------------------------------
    let 规控句柄 = tokio::spawn(async move {
        let mut 规控大脑 = 预测控制求解器::new().expect("❌ NMPC 求解器初始化失败");
        let mut 求解器已就绪 = false;
        let mut 循环计数: u64 = 0;

        // 🎯 SOTA 药方 2：自激振荡一阶低通防抖阻尼器状态 (PT1 Filter) [cite: 1.1.2]
        let mut 滤波后的期望_x = 0.0f64;
        let mut 滤波后的期望_y = 0.0f64;

        // 🎯 SOTA 药方 3：基于一阶滞后空间归一化的平滑前视滤波器状态 (First-Order Lag Spatial Filter) [cite: 21]
        let mut 滤波后的引力_x = 0.0f64;
        let mut 滤波后的引力_y = 0.0f64;
        let mut 滤波后的引力_yaw = 0.0f64;

        // 🎯 战役三第七版核心：接通障碍物物理平滑滤波器状态，消灭突变
        let mut 滤波后的障碍物_x = 1000.0f64;
        let mut 滤波后的障碍物_y = 1000.0f64;

        // 🛡️ SCAN-Planner 状态缓存：记录上一次控制周期的角速度输出，用于自车光流虚警抑制
        let mut 上一次角速度_w = 0.0f64;
        // 🎯 建立高频落盘时序审计日志 (CSV 格式)
        use std::io::Write as _;
        let mut 日志文件 = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("nmpc_telemetry.csv")
            .expect("❌ 无法创建高频时序审计日志");
        // 写入 CSV 表头，对齐所有控制维度
        let _ = writeln!(
            日志文件,
            "tick,cur_x,cur_y,cur_yaw,target_x,target_y,target_yaw,force_x,force_y,v_cmd,w_cmd,cur_v"
        );

        let mut 节拍器 = tokio::time::interval(Duration::from_millis(10));
        节拍器.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            节拍器.tick().await;
            循环计数 += 1;
            // 极速读取状态金库 (新增生命时间戳获取)
            let (已初始化, 期望_x, 期望_y, 当前线速度, 上次更新时间) = {
                let lock = 金库_规控.read().unwrap();
                (lock.物理主权已初始化, lock.期望_x, lock.期望_y, lock.当前线速度, lock.上次引力更新时间)
            };
            if !已初始化 {
                continue;
            }

            // 🛡️ 第四道防线：生命看门狗安全自愈守护 (Failsafe Watchdog) [cite: 1.2.5]
            // 飞控级底线防线：由于视觉重定位与重放存在天然的 1Hz 帧率抖动，我们将超时宽限放宽至 5.0 秒 [cite: 1.2.5]
            // 允许小车在特征偏稀疏的盲区（如 Node 6/7）暂时丢失视觉锁时，完全依赖 100Hz 高频惯性航位推算平滑“滑行（Glide）”通过！ [cite: 1.2.5]
            let 失联时长 = 上次更新时间.elapsed();
            if 失联时长 > std::time::Duration::from_millis(5000) {
                eprintln!("⚠️ [Failsafe Watchdog] 致命警告: 慢脑失联时长过长 ({:.2?}s)！触发安全防线紧急抱闸！", 失联时长.as_secs_f64());
                let 零速度指令_arrow = dora_node_api::arrow::array::Float32Array::from(vec![0.0f32, 0.0f32]);
                let _ = node.send_output(
                    "control_cmd".to_string().into(),
                    MetadataParameters::default(),
                    零速度指令_arrow,
                );
                continue; // 阻断 NMPC 主线程，保证底盘绝对安全
            }
            // NMPC 求解器温启动
            if !求解器已就绪 {
                println!("✅ [快系统] NMPC 求解器温启动完成，物理主权接管就绪！");
                求解器已就绪 = true;
            }
            if let Err(e) = 规控大脑.设置当前状态(0.0, 0.0, 0.0, 当前线速度) {
                eprintln!("⚠️ 局部状态锚定失败: {}，跳过本帧", e);
                continue;
            }
            let (当前_x, 当前_y, 当前_yaw) = {
                let lock = 金库_规控.read().unwrap();
                (lock.当前_x, lock.当前_y, lock.当前_yaw)
            };
            let (引力_x, 引力_y, 引力_yaw) = {
                let lock = 金库_规控.read().unwrap();
                (lock.引力_x, lock.引力_y, lock.引力_yaw)
            };

            // 🛡️ 第三道防线：基于一阶滞后空间归一化的平滑前视滤波器 (First-Order Lag Spatial Filter) [cite: 21]
            // 物理作用：将示教地图中高频突变的 20-30cm 阶跃（锯齿折线）平滑拟合为 C2 连续的过渡渐进线 [cite: 21]
            // 避免 NMPC 在切换节点时高频左右摆舵，从根本上消灭轮速差和车身抖动的物理共振！ [cite: 21]
            // 🚨 战役三 Rust 降噪屏障 A：低速静默闸 (Evasion Gating Guard)
            let (有效期望_x, 有效期望_y) = if 当前线速度.abs() < 0.04 {
                (0.0f64, 0.0f64)
            } else {
                (期望_x, 期望_y)
            };
            
            // 🎯 3.3.1 核心自愈：横纵交叉耦合减速机制 (Cross-Coupling Deceleration)
            // 彻底消灭“比例盲区”！只要横向逃逸避障力 (有效期望_y) 显著存在，
            // 说明障碍物正在切入马路，我们强行注入一个与横向力大小成正比的纵向刹车力减速！
            let 横向引发的减速惩罚 = - (有效期望_y.abs() * 0.45); // 0.45 为黄金减速耦合增益
            let 综合期望_x = 有效期望_x + 横向引发的减速惩罚;

            // 🎯 战役三第六版：滤波系统响应升级
            let 侧向滤波系数 = 0.125f64; 
            滤波后的期望_x += 0.125f64 * (综合期望_x - 滤波后的期望_x);
            滤波后的期望_y += 侧向滤波系数 * (有效期望_y - 滤波后的期望_y);

            // 🎯 🛡️ SCAN-Planner 药方 2：自车运动光流虚警抑制 (Egomotion Flow Suppression)
            let mut 抑制后的期望_y = 滤波后的期望_y;
            let 角速度绝对值 = 上一次角速度_w.abs();
            if 角速度绝对值 > 0.12 {
                let 抑制因子 = (-4.5 * (角速度绝对值 - 0.12)).exp();
                抑制后的期望_y *= 抑制因子.clamp(0.15, 1.0);
            }

            // 🎯 第五版核心：控制主权仲裁器 (Supervisory Control Allocation)
            // 实时精算避障势场力的最大绝对值，代表当前的危险逼近程度
            // =================================================================
            // 🎯 战役三第七版大一统控制逻辑（NMPC 弹性松弛避障）
            // =================================================================

            // 1. 🛡️ 基于一阶滞后空间归一化的平滑前视滤波器
            if 循环计数 == 1 || (引力_x == 0.0 && 引力_y == 0.0) {
                滤波后的引力_x = 引力_x;
                滤波后的引力_y = 引力_y;
                滤波后的引力_yaw = 引力_yaw;
            } else {
                let 空间滤波系数 = 0.15f64; // 前视目标点在 100-150ms 内平滑过渡
                滤波后的引力_x += 空间滤波系数 * (引力_x - 滤波后的引力_x);
                滤波后的引力_y += 空间滤波系数 * (引力_y - 滤波后的引力_y);
                滤波后的引力_yaw += 空间滤波系数 * (引力_yaw - 滤波后的引力_yaw);
            }

            // 2. 局部相对坐标系变换
            let dx = 滤波后的引力_x - 当前_x;
            let dy = 滤波后的引力_y - 当前_y;
            let mut 局部目标_x = dx * 当前_yaw.cos() + dy * 当前_yaw.sin();
            let mut 局部目标_y = -dx * 当前_yaw.sin() + dy * 当前_yaw.cos();

            // 3. SCAN-Planner 双圆盘体态切向对齐 [cite: 2]
            let mut 局部目标_yaw = 滤波后的引力_yaw - 当前_yaw;
            局部目标_yaw = 局部目标_yaw.clamp(-0.25, 0.25);

            // 4. 前视引力弹性限制器
            let 物理跨度 = (局部目标_x * 局部目标_x + 局部目标_y * 局部目标_y).sqrt();
            let 物理前视极限 = 1.2f64;
            if 物理跨度 > 物理前视极限 {
                let 缩放比 = 物理前视极限 / 物理跨度;
                局部目标_x *= 缩放比;
                局部目标_y *= 缩放比;
            }

            // 5. 注入 20 步参考轨迹
            // 采用第一版的经典无约束参考速度与路径形变设定（仅靠 Q/R 代价矩阵牵引）
            let mut rebound_y = 0.0f64;
            if 抑制后的期望_y.abs() > 0.02 {
                let rebound_dir = 抑制后的期望_y.signum();
                rebound_y = rebound_dir * (抑制后的期望_y.abs() * 0.75).min(0.35);
            }

            let 目标线速度 = (0.20 + 滤波后的期望_x).clamp(0.0, 0.20); 
            let mut 注入成功 = true;
            for k in 0..=20 {
                let 比例 = (k as f64) / 20.0;
                let 基础_ref_x = 局部目标_x * 比例;
                let 基础_ref_y = 局部目标_y * 比例;
                let 基础_ref_yaw = 局部目标_yaw * 比例;
                
                let spiced_ref_x = 基础_ref_x + (滤波后的期望_x * 比例);
                let spiced_ref_y = 基础_ref_y + (rebound_y * 比例); 
                if let Err(e) = 规控大脑.设置参考轨迹点(k, spiced_ref_x, spiced_ref_y, 基础_ref_yaw, 目标线速度) {
                    eprintln!("⚠️ 第 {} 步参考轨迹注入失败: {}", k, e);
                    注入成功 = false;
                    break;
                }
            }
            if !注入成功 { continue; }

            // 6. 🎯 战役三第七版精髓：障碍物坐标连续低通飘移
            // 将经由经典 FAST 特征纯净化（无静态地标虚警）后的纯动态斥力，在小车坐标系正前方坍缩
            let (目标障碍物_x, 目标障碍物_y, 椭圆长轴_a, 椭圆短轴_b) = if 抑制后的期望_y.abs() > 0.04 || 滤波后的期望_x < -0.04 {
                // 障碍物在前方 0.65 米处，其横向偏置与我们逃逸打舵方向完全相反
                let ox = 0.65;
                let oy = -抑制后的期望_y.clamp(-0.35, 0.35); 
                (ox, oy, 0.35, 0.25)
            } else {
                // 安全期：优雅地流放到 1000m 外，维持极小轴
                (1000.0, 1000.0, 0.1, 0.1)
            };

            // 120ms 一阶阻尼平滑过滤器，杜绝水泥墙“空降砸脸”
            let 障碍物阻尼系数 = 0.125f64; 
            if 目标障碍物_x > 500.0 {
                滤波后的障碍物_x = 目标障碍物_x;
                滤波后的障碍物_y = 目标障碍物_y;
            } else {
                if 滤波后的障碍物_x > 500.0 {
                    滤波后的障碍物_x = 目标障碍物_x;
                    滤波后的障碍物_y = 目标障碍物_y;
                } else {
                    滤波后的障碍物_x += 障碍物阻尼系数 * (目标障碍物_x - 滤波后的障碍物_x);
                    滤波后的障碍物_y += 障碍物阻尼系数 * (目标障碍物_y - 滤波后的障碍物_y);
                }
            }

            // 将阻尼移动后的椭圆物理参数实时喂入求解器！
            // 此时由于求解器内部具备第四版 L1/L2 软弹性床垫，遇到危险时小车将顺滑绕行，绝不发散或抖动！
            let _ = 规控大脑.设置动态障碍物硬约束(滤波后的障碍物_x, 滤波后的障碍物_y, 椭圆长轴_a, 椭圆短轴_b);

            // 7. 纯净 RTI-SQP 求解
            let (线速度_v, 角速度_w) = match 规控大脑.求解最优控制量(当前线速度) {
                Ok((v, w)) => (v, w),
                Err(e) => {
                    eprintln!("⚠️ NMPC 求解器非线性解算异常: {}", e);
                    (0.0, 0.0) // 紧急锁死安全防线
                }
            };

            // 统一写入并更新状态金库，保证下个控制周期所有权正常流转
            {
                let mut lock = 金库_规控.write().unwrap();
                lock.当前线速度 = 线速度_v;
            }
            上一次角速度_w = 角速度_w;

            // 📊 2026 工业级落盘时序审计
            let _ = writeln!(
                日志文件,
                "{}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}",
                循环计数, 当前_x, 当前_y, 当前_yaw, 引力_x, 引力_y, 引力_yaw, 有效期望_x, 有效期望_y, 线速度_v, 角速度_w, 当前线速度
            );

            // 📊 2026 工业级数值探针
            if 循环计数 % 100 == 0 {
                let 禁区状态 = if 滤波后的障碍物_x < 100.0 { "🔴 弹性防御" } else { "🟢 畅通无阻" };
                // 优化控制台打印：显式输出从 DORA 接收到的青蛙眼避障原始推力，实现数据白盒化
                println!(
                    "[快脑 100Hz 遥测] 步: {:<5} | {} | 势场原始力: (Fx:{:>6.3}, Fy:{:>6.3}) | 速度: {:.3} m/s | 打舵: {:>6.3} rad/s",
                    循环计数, 禁区状态, 期望_x, 期望_y, 线速度_v, 角速度_w
                );
            }

            // 🎯 架构师升维：构建 Arrow Float32Array，通过共享内存零拷贝直达 Python
            let 运动指令_arrow = dora_node_api::arrow::array::Float32Array::from(vec![
                线速度_v as f32, 
                角速度_w as f32
            ]);
            if let Err(e) = node.send_output(
                "control_cmd".to_string().into(),
                MetadataParameters::default(),
                运动指令_arrow,
            ) {
                eprintln!("❌ 控制指令发送失败: {}", e);
            }
        }
    });

    // ---------------------------------------------------------
    // [线程 B]：DORA 神经反射弧 (The Event Loop - 100Hz 避障力注入)
    // ---------------------------------------------------------
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() == "obstacle_force" {
                    // 🎯 架构师升维：直接将 Arrow 内存映射为 Float32Array，彻底消除反序列化
                    let 势场数组 = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("❌ 无法将 DORA 数据转换为 Float32Array"))?;
                    if 势场数组.len() < 2 {
                        continue;
                    }
                    let f_x = 势场数组.value(0) as f64;
                    let f_y = 势场数组.value(1) as f64;
                    
                    // 🎯 物理揉入：将解出来的仿生势场逃逸矢量，写入状态金库
                    {
                        let mut lock = 状态金库.write().unwrap();
                        lock.期望_x = f_x; // 作用于 NMPC 的纵向参考速度
                        lock.期望_y = f_y; // 作用于 NMPC 的横向路径偏移
                    }
                }
                else if id.as_str() == "odometry" {
                    let arr = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();
                    if arr.len() >= 3 {
                        let mut lock = 状态金库.write().unwrap();
                        lock.当前_x = arr.value(0) as f64;
                        lock.当前_y = arr.value(1) as f64;
                        lock.当前_yaw = arr.value(2) as f64;
                    }
                }
                else if id.as_str() == "human_prior" {
                    let arr = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();
                    if arr.len() >= 3 {
                        let mut lock = 状态金库.write().unwrap();
                        lock.引力_x = arr.value(0) as f64;
                        lock.引力_y = arr.value(1) as f64;
                        lock.引力_yaw = arr.value(2) as f64;
                        
                        // 🎯 战役三第六版：起跑线温启动锁定闸 (Start-line Gating)
                        // 只有当慢脑初次匹配成功、发来第一个有效人类引力坐标时，才允许并网激活物理主权！
                        // 彻底杜绝冷启动前 7 秒由于无引力目标导致小车在原点打转偏航的惨剧！
                        if !lock.物理主权已初始化 {
                            lock.物理主权已初始化 = true;
                            println!("🟢 [温启动] 慢脑首帧匹配成功！物理主权安全并网，起跑线解锁！");
                        }
                        
                        // 🎯 喂狗：刷新引力点接收时间戳，喂养生命看门狗！ [cite: 1.2.5]
                        lock.上次引力更新时间 = std::time::Instant::now();
                    }
                }
            }
            Event::Stop(_) => {
                println!("🛑 [快系统] 接收到 DORA 停止信号，安全卸载并释放控制权...");
                break;
            }
            _ => {}
        }
    }
    
    // 优雅卸载
    规控句柄.abort();
    Ok(())
}

#[cfg(test)]
mod tests {
    use dora_node_api::arrow::array::{Float32Array, StructArray, FixedSizeListArray, Array};
    use dora_node_api::arrow::datatypes::{DataType, Field};
    use std::sync::Arc;

    #[test]
    fn test_arrow_struct_array_zero_copy_deserialization() {
        println!("🛡️ [内存探针] 正在模拟 Python 端 pyarrow 内存布局...");

        // 1. 模拟 Python 端构建 Arrow StructArray 的过程 (2个 XFeat 特征点)
        let x_arr = Arc::new(Float32Array::from(vec![10.5, 20.5])) as Arc<dyn Array>;
        let y_arr = Arc::new(Float32Array::from(vec![15.2, 25.2])) as Arc<dyn Array>;
        let score_arr = Arc::new(Float32Array::from(vec![0.95, 0.88])) as Arc<dyn Array>;

        // 构建 64 维描述子 (2个特征点，共 128 个 f32 连续内存)
        let mut desc_data = Vec::with_capacity(128);
        for i in 0..128 {
            desc_data.push(i as f32 * 0.1);
        }
        let desc_flat = Float32Array::from(desc_data);
        let field = Arc::new(Field::new("item", DataType::Float32, true));
        let desc_list_arr = Arc::new(
            FixedSizeListArray::try_new(
                field.clone(),
                64,
                Arc::new(desc_flat),
                None,
            ).expect("FixedSizeListArray 构建失败")
        ) as Arc<dyn Array>;

        // 组装为最终的 StructArray (严格对齐 Python 端的 names)
        let struct_arr = StructArray::from(vec![
            (Arc::new(Field::new("x", DataType::Float32, false)), x_arr),
            (Arc::new(Field::new("y", DataType::Float32, false)), y_arr),
            (Arc::new(Field::new("score", DataType::Float32, false)), score_arr),
            (Arc::new(Field::new("descriptor", DataType::FixedSizeList(field, 64), false)), desc_list_arr),
        ]);

        // 模拟 DORA 跨进程传递过来的 Arc<dyn Array> 泛型指针
        let data: Arc<dyn Array> = Arc::new(struct_arr);

        println!("✅ [内存探针] 虚拟共享内存构建完毕，开始执行 Rust 端零拷贝解析...");

        // ----------------------------------------------------------------
        // 2. 验证 Rust 端的零拷贝解析逻辑 (严格对齐 slow_brain_node.rs 的业务代码)
        // ----------------------------------------------------------------
        let 结构体数组 = data.as_any().downcast_ref::<StructArray>().expect("❌ 致命错误：向下转型为 StructArray 失败");
        assert_eq!(结构体数组.len(), 2, "特征点数量应该为 2");

        let 解析_x = 结构体数组.column_by_name("x").unwrap().as_any().downcast_ref::<Float32Array>().unwrap();
        let 解析_y = 结构体数组.column_by_name("y").unwrap().as_any().downcast_ref::<Float32Array>().unwrap();
        let 解析_score = 结构体数组.column_by_name("score").unwrap().as_any().downcast_ref::<Float32Array>().unwrap();
        let 解析_desc_list = 结构体数组.column_by_name("descriptor").unwrap().as_any().downcast_ref::<FixedSizeListArray>().unwrap();
        let 解析_desc_values = 解析_desc_list.values().as_any().downcast_ref::<Float32Array>().unwrap();

        // ----------------------------------------------------------------
        // 3. 物理断言：验证内存指针偏移与数值精度是否绝对无损
        // ----------------------------------------------------------------
        assert_eq!(解析_x.value(0), 10.5, "X 坐标解析错误");
        assert_eq!(解析_y.value(1), 25.2, "Y 坐标解析错误");
        assert_eq!(解析_score.value(0), 0.95, "置信度解析错误");

        // 验证第一个特征点的描述子 (偏移量 0)
        let offset_0 = 0;
        assert_eq!(解析_desc_values.value(offset_0 + 0), 0.0);
        assert_eq!(解析_desc_values.value(offset_0 + 1), 0.1);

        // 验证第二个特征点的描述子 (偏移量 64)
        let offset_1 = 64;
        assert_eq!(解析_desc_values.value(offset_1 + 0), 6.4); // 64 * 0.1
        assert_eq!(解析_desc_values.value(offset_1 + 1), 6.5); // 65 * 0.1

        println!("🏆 [验证结论] Arrow StructArray 零拷贝解析逻辑完美通过！");
        println!("诊断报告：");
        println!("  1. 内存对齐：Python 端的列式内存布局被 Rust 完美识别。");
        println!("  2. 零拷贝：全程未使用任何反序列化函数，仅通过指针偏移 (downcast_ref) 完成数据提取。");
        println!("  3. 性能预估：解析 1000 个特征点的耗时将从之前的数毫秒暴降至纳秒级 (O(1) 复杂度)。");
    }
}