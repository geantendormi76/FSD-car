// 🛡️协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
use postcard::from_bytes;
use serde::{Deserialize, Serialize};
use zenoh::prelude::r#async::*;

/// 【全简体中文业务逻辑主权】
/// 物理主权守卫：运动指令契约 (严格 8 字节对齐，必须与大脑发送端完全一致)
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
#[repr(C)]
pub struct 运动指令 {
    pub v: f32, // 线速度 (m/s)
    pub w: f32, // 角速度 (rad/s)
}

#[tokio::main]
async fn main() {
    println!("🦵 [虚拟脊髓启动] 正在初始化模拟接收通路...");

    // 1. 初始化 Zenoh 默认配置
    let config = Config::default();

    // 2. 建立本地会话连接
    let session = zenoh::open(config)
        .res()
        .await
        .expect("❌ 致命错误：模拟接收端无法连接至 Zenoh 网络！");

    println!("✅ [神经通路建立] 虚拟脊髓已成功并网！");

    // 3. 声明订阅者 (Subscriber) 监听 "fsd/cmd_vel" 话题
    let subscriber = session
        .declare_subscriber("fsd/cmd_vel")
        .res()
        .await
        .expect("❌ 致命错误：无法订阅大脑话题 'fsd/cmd_vel'！");

    println!("📡 [接收端就绪] 正在等待大脑发送 8 字节二进制控制脉冲...");

    // 🛡️ 架构师并网控制：在接收循环外部显式初始化计数器
    let mut step: u64 = 0;

    // 4. 异步事件接收循环
    while let Ok(sample) = subscriber.recv_async().await {
        // 通过 Zenoh 0.11.0 官方原生 TryInto 转换特征，将 ZBuf 转换为连续的 Vec<u8>
        let payload_bytes: Vec<u8> = sample.value.try_into().unwrap();
        
        // 5. 零拷贝反序列化
        match from_bytes::<运动指令>(&payload_bytes) {
            Ok(cmd) => {
                // 🛡️ 架构师并网控制：给接收端也装上 I/O 节流阀！
                // 每 30 帧才向屏幕输出一次。现在，两端的打印滚动速度将在视觉上达到完美的 100% 同步！
                if step % 30 == 0 {
                    println!(
                        "🚗 [指令接收成功] -> 线速度: {:.3} m/s, 角速度: {:.3} rad/s (契约校验：{} 字节)",
                        cmd.v, cmd.w, payload_bytes.len()
                    );
                }
            }
            Err(e) => {
                eprintln!("❌ [接收崩溃] 反序列化解析失败，数据可能损坏: {:?}", e);
            }
        }
        
        // 递增计数器，推动节流阀节拍
        step += 1;
    }
}