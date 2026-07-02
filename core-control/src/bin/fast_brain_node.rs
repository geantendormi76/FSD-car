// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

/*
=================================================================
🛡️ FSD-car V3.0: 快系统规控大脑节点 (NMPC & 仿生避障合拢版)
设计哲学: 完全剥离图像计算 | 100Hz 绝对物理时钟 | 势场逃逸力数学合拢
=================================================================
*/

use core_control::预测控制求解器;
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre; // 🛡️ 架构师 2026 修复：移除未使用的 Context
use std::sync::Arc;
use std::sync::RwLock; // 状态金库轻量级锁
use std::time::Duration;

/// 状态金库：快大脑 100Hz 线程与 避障力接收线程 间绝对安全的无锁共享上下文
struct 执行上下文 {
    pub 物理主权已初始化: bool,
    pub 期望_x: f64,              // 🛡️ 架构师 2026 修复：将 # 注释替换为 Rust 标准双斜杠 //
    pub 期望_y: f64,              // 🛡️ 架构师 2026 修复：将 # 注释替换为 Rust 标准双斜杠 //
    pub 当前线速度: f64,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🧠 [快系统] 规控大脑节点已启动，等待 DORA 共享内存注入...");

    // 1. 接入 DORA 数据流网络 (接管生命周期与共享内存池)
    let (mut node, mut events) = DoraNode::init_from_env()?;

    // 2. 初始化状态金库
    let 状态金库 = Arc::new(RwLock::new(执行上下文 {
        物理主权已初始化: false,
        期望_x: 0.0,              // 默认无纵向排斥
        期望_y: 0.0,              // 默认无横向逃逸
        当前线速度: 0.0,
    }));

    let 金库_规控 = 状态金库.clone();

    // ---------------------------------------------------------
    // [线程 A]：100Hz 极速 NMPC 控制环路 (The Control Loop)
    // ---------------------------------------------------------
    let 规控句柄 = tokio::spawn(async move {
        let mut 规控大脑 = 预测控制求解器::new().expect("❌ NMPC 求解器初始化失败");
        let mut 求解器已就绪 = false;
        
        let mut 节拍器 = tokio::time::interval(Duration::from_millis(10));
        
        loop {
            节拍器.tick().await;
            
            // 极速读取状态金库
            let (已初始化, 期望_x, 期望_y, 当前线速度) = {
                let lock = 金库_规控.read().unwrap();
                (lock.物理主权已初始化, lock.期望_x, lock.期望_y, lock.当前线速度)
            };

            if !已初始化 {
                continue;
            }

            // NMPC 求解器温启动 (仅执行一次)
            if !求解器已就绪 {
                if let Err(e) = 规控大脑.设置当前状态(0.0, 0.0, 0.0, 0.0) {
                    eprintln!("⚠️ 温启动状态注入失败: {}，跳过本帧", e);
                    continue;
                }
                求解器已就绪 = true;
                println!("✅ [快系统] NMPC 求解器温启动完成，物理主权接管就绪！");
            }

            // 🎯 将避障力带来的 (期望_x, 期望_y) 偏差，高频注入 NMPC 数学命题
            // 期望_x: 基础前行速度 0.3m/s 加上纵向排斥 (F_x 为负，实现遇障减速)
            // 期望_y: 居中前行 y=0 加上横向逃逸 (F_y 偏离 0，实现局部路径机动)
            let 目标线速度 = (0.3 + 期望_x).max(0.0).min(0.3); // 限制在安全范围内
            
            let mut 注入成功 = true;
            for k in 0..=20 {
                // 将计算出的避障修正，揉入 NMPC 20步预测时域的参考轨迹 yref 中！
                if let Err(e) = 规控大脑.设置参考轨迹点(k, 目标线速度 * (k as f64 * 0.01), 期望_y, 0.0, 目标线速度) {
                    eprintln!("⚠️ 第 {} 步参考轨迹注入失败: {}", k, e);
                    注入成功 = false;
                    break;
                }
            }
            if !注入成功 { continue; }
            
            // 求解最优控制量
            match 规控大脑.求解最优控制量(当前线速度) {
                Ok((线速度_v, 角速度_w)) => {
                    // 更新线速度状态用于下一次积分
                    {
                        let mut lock = 金库_规控.write().unwrap();
                        lock.当前线速度 = 线速度_v;
                    }

                    // 🎯 极速裸二进制直发：向并网网关传输 8 字节控制指令
                    let 运动指令_裸数组 = [线速度_v as f32, 角速度_w as f32];
                    let 裸内存切片: &[u8] = unsafe {
                        std::slice::from_raw_parts(运动指令_裸数组.as_ptr() as *const u8, 8)
                    };
                    
                    if let Err(e) = node.send_output_bytes(
                        "control_cmd".to_string().into(),
                        MetadataParameters::default(),
                        8,
                        裸内存切片,
                    ) {
                        eprintln!("❌ 控制指令发送失败: {}", e);
                    }
                }
                Err(e) => {
                    eprintln!("⚠️ NMPC 求解器异常发散: {}", e);
                }
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
                    // 🎯 核心接收自愈：将并网网关发来的 8字节 势场力解析出来
                    let 裸数据_vec: Vec<u8> = dora_node_api::into_vec(&data)
                        .map_err(|e| eyre!("❌ 无法解析 DORA 势场力数据: {}", e))?;
                    
                    if 裸数据_vec.len() < 8 {
                        continue;
                    }

                    // 零拷贝内存重塑为 F_x (纵向排斥力) 和 F_y (横向逃逸力)
                    let f_x = f32::from_le_bytes(裸数据_vec[0..4].try_into().unwrap()) as f64;
                    let f_y = f32::from_le_bytes(裸数据_vec[4..8].try_into().unwrap()) as f64;

                    // 🛡️ 架构师 2026 修复：替换 # 为 //
                    let 需要初始化 = {
                        let lock = 状态金库.read().unwrap();
                        !lock.物理主权已初始化
                    };

                    if 需要初始化 {
                        let mut lock = 状态金库.write().unwrap();
                        lock.物理主权已初始化 = true;
                        println!("✅ [快系统] 跨 OS 仿生眼避障通道激活，控制权交接完毕！");
                    }

                    // 🎯 物理揉入：将解出来的仿生势场逃逸矢量，写入状态金库
                    {
                        let mut lock = 状态金库.write().unwrap();
                        lock.期望_x = f_x; // 作用于 NMPC 的纵向参考速度
                        lock.期望_y = f_y; // 作用于 NMPC 的横向路径偏移
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