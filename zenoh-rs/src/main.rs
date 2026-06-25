use postcard::to_allocvec;
use serde::{Deserialize, Serialize};
use std::time::Duration;
use tokio::time;

// 🛡️ 架构师修正：并网神经元特征导入
// 引入 Zenoh 0.11.0 的异步预导入模块 (Prelude)
// 这会引入 AsyncResolve 特征，同时让编译器自动推断出 Session 和 Future 的类型，完美解决 E0599 和 E0282 错误。
use zenoh::prelude::r#async::*; 
/// 【全简体中文业务逻辑主权】
/// 物理主权守卫：运动指令契约 (严格 8 字节对齐)
/// 为什么用 #[repr(C)]？为了保证在 PC (x86_64/ARM64) 和 ESP32 (RISC-V) 之间
/// 内存布局绝对一致，防止字节对齐导致的幽灵 Bug。
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[repr(C)]
pub struct 运动指令 {
    pub v: f32, // 线速度 (m/s)
    pub w: f32, // 角速度 (rad/s)
}

#[tokio::main]
async fn main() {
    println!("🚀 [大脑节点启动] 正在初始化 Zenoh 神经通路...");

    // 1. 初始化 Zenoh 配置 (默认使用点对点组网，自动发现局域网内的路由器或节点)
    let config = Config::default();
    
    // 2. 建立 Zenoh 会话 (Session)
    // 架构师注：这里的所有权 (Ownership) 归属 main 函数，
    // session 的生命周期贯穿整个大脑进程。
    let session = zenoh::open(config)
        .res()
        .await
        .expect("❌ 致命错误：无法连接到 Zenoh 网络！");

    println!("✅ [神经通路已连接] Zenoh Session 建立成功！");

    // 3. 声明发布者 (Publisher)
    // 话题 (Key Expression) 设定为 "fsd/cmd_vel"
    let publisher = session
        .declare_publisher("fsd/cmd_vel")
        .res()
        .await
        .expect("❌ 致命错误：无法声明 Publisher！");

    println!("📡 [发布者就绪] 正在向主题 'fsd/cmd_vel' 广播运动指令...");

    // 4. 模拟快系统 (System 2) 的 30Hz NMPC 控制循环
    // 使用 Tokio 的 Interval 保证绝对的调度精度，不受代码执行耗时影响
    let mut interval = time::interval(Duration::from_millis(33)); // 约 30Hz

    let mut step: u64 = 0;

    loop {
        // 异步等待下一个 33ms 滴答 (让出 CPU 线程给其他任务，绝不阻塞)
        interval.tick().await;

        // 模拟 NMPC 算法生成的平滑指令 (正弦波测试)
        let simulated_v = 0.3 + 0.1 * (step as f32 * 0.1).sin();
        let simulated_w = 0.5 * (step as f32 * 0.1).cos();

        // 实例化契约
        let cmd = 运动指令 {
            v: simulated_v,
            w: simulated_w,
        };

        // 5. 零拷贝序列化 (Zero-copy Serialization)
        // 使用 Postcard 将结构体压缩为极其紧凑的二进制字节流 (正好 8 字节)
        // 绝对禁止在这里使用 JSON 导致单片机解析耗时过长！
        match to_allocvec(&cmd) {
            Ok(payload) => {
                // 6. 通过 Zenoh 神经通路发射！
                if let Err(e) = publisher.put(payload.clone()).res().await {
                    eprintln!("⚠️ [坠网拦截] 数据发送失败: {:?}", e);
                } else {
                    // 每 30 帧打印一次遥测数据，防止终端刷屏 (保护 I/O 性能)
                    if step % 30 == 0 {
                        println!(
                            "🌊 [神经脉冲] 发送指令 -> v: {:.3} m/s, w: {:.3} rad/s | 负载大小: {} bytes",
                            cmd.v, cmd.w, payload.len()
                        );
                    }
                }
            }
            Err(e) => {
                eprintln!("❌ [序列化崩溃] 无法编码运动指令: {:?}", e);
            }
        }

        step += 1;
    }
}