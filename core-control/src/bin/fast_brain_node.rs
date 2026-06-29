use dora_node_api::arrow::array::Float32Array;
use core_control::预测控制求解器;
use core_perception::perception::frog_eye::{仿生青蛙眼, 伪青蛙眼感知器};
use dora_node_api::{DoraNode, Event, MetadataParameters, Parameter};
use eyre::{eyre, Context};
use opencv::core::{Mat, CV_8UC3};
use std::ffi::c_void;
use std::sync::Arc;
use std::sync::RwLock; // 🛡️ 架构师修正：降级为标准库锁，拒绝 Tokio 调度让出
use std::time::Duration;

/// 🛡️ 状态金库：跨线程无锁/轻量锁共享的执行上下文
struct 执行上下文 {
    pub 物理主权已初始化: bool,
    pub 期望_x: f64,
    pub 期望_y: f64,
    pub 当前线速度: f64,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🧠 [快系统] 规控大脑节点已启动，等待 DORA 共享内存注入...");

    // 1. 接入 DORA 数据流网络 (接管生命周期与共享内存池)
    let (mut node, mut events) = DoraNode::init_from_env()?;

    // 2. 初始化状态金库 (Arc + RwLock 保证多读单写并发安全)
    let 状态金库 = Arc::new(RwLock::new(执行上下文 {
        物理主权已初始化: false,
        期望_x: 0.3,
        期望_y: 0.0,
        当前线速度: 0.0,
    }));

    let 金库_规控 = 状态金库.clone();

    // ---------------------------------------------------------
    // [阶段 A]：规控节拍器 (The Control Loop - 100Hz 绝对物理时钟)
    // ---------------------------------------------------------
    let 规控句柄 = tokio::spawn(async move {
        let mut 规控大脑 = 预测控制求解器::new().expect("❌ NMPC 求解器初始化失败");
        let mut 求解器已就绪 = false;
        
        // 🛡️ 架构师指令：强制 100Hz (10ms) 物理时钟，彻底斩断与摄像头帧率的耦合！
        let mut 节拍器 = tokio::time::interval(Duration::from_millis(10));
        
        loop {
            节拍器.tick().await;
            
            // 1. 极速读取状态金库 (读写锁，读操作极快)
            let (已初始化, 期望_x, 期望_y, 当前线速度) = {
                // 🛡️ 架构师修正：使用标准库锁，不跨 await 边界，耗时仅需几纳秒
                let lock = 金库_规控.read().unwrap();
                (lock.物理主权已初始化, lock.期望_x, lock.期望_y, lock.当前线速度)
            };

            if !已初始化 {
                continue;
            }

            // 2. 求解器温启动 (仅执行一次)
            if !求解器已就绪 {
                if let Err(e) = 规控大脑.设置当前状态(0.0, 0.0, 0.0, 0.0) {
                    eprintln!("⚠️ 温启动状态注入失败: {}，跳过本帧", e);
                    continue;
                }
                求解器已就绪 = true;
                println!("✅ [快系统] NMPC 求解器温启动完成，进入 100Hz 极速控制环");
            }

            // 3. 注入最新的参考轨迹偏移
            let mut 注入成功 = true;
            for k in 0..=20 {
                if let Err(e) = 规控大脑.设置参考轨迹点(k, 期望_x, 期望_y, 0.0, 0.3) {
                    eprintln!("⚠️ 第 {} 步参考轨迹注入失败: {}", k, e);
                    注入成功 = false;
                    break;
                }
            }
            if !注入成功 { continue; }
            
            // 4. 求解最优控制量 (基于严格的 10ms dt)
            match 规控大脑.求解最优控制量(当前线速度) {
                Ok((线速度_v, 角速度_w)) => {
                    // 更新金库中的线速度，供下一次积分使用
                    {
                        let mut lock = 金库_规控.write().unwrap();
                        lock.当前线速度 = 线速度_v;
                    }

                    // 5. 零拷贝输出指令
                    let 运动指令 = Float32Array::from(vec![线速度_v as f32, 角速度_w as f32]);
                    if let Err(e) = node.send_output(
                        "control_cmd".to_string().into(),
                        MetadataParameters::default(),
                        运动指令,
                    ) {
                        eprintln!("❌ 控制指令发送失败: {}", e);
                    }
                }
                Err(e) => {
                    eprintln!("⚠️ NMPC 求解发散或异常: {}", e);
                }
            }
        }
    });

    // ---------------------------------------------------------
    // [阶段 B]：神经反射弧 (The Event Loop - 30Hz 视觉感知)
    // ---------------------------------------------------------
    let mut 青蛙眼 = 仿生青蛙眼::new();

    // 🛡️ 架构师修正：使用 recv_async().await 彻底释放 Tokio 线程池，避免阻塞
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, metadata, data } => {
                if id.as_str() == "image" {
                    let 宽度 = match metadata.parameters.get("width") {
                        Some(Parameter::Integer(val)) => *val as i32,
                        _ => 640,
                    };
                    let 高度 = match metadata.parameters.get("height") {
                        Some(Parameter::Integer(val)) => *val as i32,
                        _ => 480,
                    };

                    // 检查并初始化物理主权
                    let 需要初始化 = {
                        let lock = 状态金库.read().unwrap();
                        !lock.物理主权已初始化
                    };

                    if 需要初始化 {
                        青蛙眼.初始化(宽度, 高度).map_err(|e| eyre!(e))?;
                        let mut lock = 状态金库.write().unwrap();
                        lock.物理主权已初始化 = true;
                        println!("✅ [快系统] 感受野初始化完成 ({}x{})", 宽度, 高度);
                    }

                    // 物理级零拷贝 (Arrow -> OpenCV Mat)
                    let 裸内存切片: &[u8] = (&data).try_into().context("❌ Arrow 内存强转 &[u8] 失败")?;
                    let 视网膜帧 = unsafe {
                        Mat::new_rows_cols_with_data_unsafe(
                            高度,
                            宽度,
                            CV_8UC3,
                            裸内存切片.as_ptr() as *mut c_void,
                            opencv::core::Mat_AUTO_STEP,
                        ).context("❌ OpenCV Mat 映射失败")?
                    };

                    // 仿生感知解算
                    let 势场 = 青蛙眼.处理图像帧(&视网膜帧, 0.0).map_err(|e| eyre!(e))?;

                    // 将势场梯度转化为 NMPC 参考轨迹偏移，并写入状态金库
                    let mut lock = 状态金库.write().unwrap();
                    lock.期望_x = 0.3 + 势场.逃逸方向.0 as f64;
                    lock.期望_y = 0.0 + 势场.逃逸方向.1 as f64;
                }
            }
            Event::Stop(_) => {
                println!("🛑 [快系统] 接收到 DORA 全局停止信号，安全释放 C 语言内存胶囊并退出...");
                break;
            }
            _ => {} 
        }
    }
    
    // 优雅终止规控节拍器
    规控句柄.abort();
    Ok(())
}