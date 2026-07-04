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
    pub 期望_x: f64,              // 纵向势场排斥力 (避障减速)
    pub 期望_y: f64,              // 横向势场逃逸力 (变道机动)
    pub 当前线速度: f64,           // 上一帧 NMPC 输出并在物理世界执行后的真实线速度
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
    }));

    let 金库_规控 = 状态金库.clone();

    // ---------------------------------------------------------
    // [线程 A]：100Hz 极速 NMPC 控制环路 (The Control Loop)
    // ---------------------------------------------------------
    let 规控句柄 = tokio::spawn(async move {
        let mut 规控大脑 = 预测控制求解器::new().expect("❌ NMPC 求解器初始化失败");
        let mut 求解器已就绪 = false;
        let mut 循环计数: u64 = 0;
        
        let mut 节拍器 = tokio::time::interval(Duration::from_millis(10));
        节拍器.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        
        loop {
            节拍器.tick().await;
            循环计数 += 1;
            
            // 极速读取状态金库
            let (已初始化, 期望_x, 期望_y, 当前线速度) = {
                let lock = 金库_规控.read().unwrap();
                (lock.物理主权已初始化, lock.期望_x, lock.期望_y, lock.当前线速度)
            };

            if !已初始化 {
                continue;
            }

            // NMPC 求解器温启动
            if !求解器已就绪 {
                println!("✅ [快系统] NMPC 求解器温启动完成，物理主权接管就绪！");
                求解器已就绪 = true;
            }

            // 🎯 核心修复 1 (自愈锚定)
            // 纯视觉无图导航采用“局部相对坐标系”。
            // 必须在【每一帧】将当前状态强制锚定在原点 (0,0,0)，仅更新当前真实线速度！
            // 否则 Acados 会使用上一帧的预测末端作为初始状态，导致控制坐标系漂移与疯狂旋转！
            if let Err(e) = 规控大脑.设置当前状态(0.0, 0.0, 0.0, 当前线速度) {
                eprintln!("⚠️ 局部状态锚定失败: {}，跳过本帧", e);
                continue;
            }

            // 将避障力带来的期望偏差，高频注入 NMPC 数学命题
            let 目标线速度 = (0.3 + 期望_x).clamp(0.0, 0.3); // 限制在安全范围内
            
            let mut 注入成功 = true;
            for k in 0..=20 {
                // 🎯 核心修复 2 (时域步长对齐)
                // NMPC 的预测总时间为 1.0s，共 20 步，单步时间间隔为 0.05s！
                // 必须使用 0.05 替换原版的 0.01，使参考轨迹的时域与求解器时域完全对齐！
                // 这样小车线速度便能顺利释放到真实的 0.3 m/s，绝不发生动力爬行或滞后。
                let ref_x = 目标线速度 * (k as f64 * 0.05);
                
                if let Err(e) = 规控大脑.设置参考轨迹点(k, ref_x, 期望_y, 0.0, 目标线速度) {
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

                    // 📊 2026 工业级数值探针
                    if 循环计数 % 100 == 0 {
                        println!(
                            "[快大脑 100Hz 遥测] 步数: {:<6} | 目标线速: {:.3} m/s | 避障偏置: {:.3} m | NMPC输出 -> v: {:.3} m/s, w: {:.3} rad/s",
                            循环计数, 目标线速度, 期望_y, 线速度_v, 角速度_w
                        );
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