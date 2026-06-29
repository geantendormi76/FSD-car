// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{eyre, Context};
use core_decision::messages::运动指令;
use zenoh::config::{Config, WhatAmI};
use zenoh::prelude::r#async::*; // 🛡️ 引入 Zenoh 异步解析特质

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🔗 [神经通路] 核心并网网关已启动，等待 DORA 共享内存注入...");

    // 1. 接入 DORA 数据流网络
    let (mut _node, mut events) = DoraNode::init_from_env()?;

    // 2. 初始化 Zenoh 客户端 (Client 模式)
    // 🛡️ 架构师指令：必须使用 Client 模式，防止与 DORA 底层的 Zenoh Router 产生端口冲突！
    let mut z_config = Config::default();
    // 🛡️ 架构师修正：使用 Peer 模式防止启动时因找不到 Router 而超时；同时禁用 listen 端口以 100% 避让 DORA 占用的 7447 端口！
    z_config.set_mode(Some(WhatAmI::Peer))
        .expect("❌ Zenoh 设置 Peer 模式失败");
    z_config
        .insert_json5("listen", "[]")
        .expect("❌ Zenoh 禁用监听配置失败");
    
    // Zenoh 0.11.0 引入了 Builder 模式，调用 .res().await 执行异步操作
    let z_session = zenoh::open(z_config)
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 会话打开失败: {}", e))?;
    
    // 3. 声明 Zenoh 发布者 (Publisher)
    let 发布主题 = "fsd/spinal_cord/cmd_vel";
    let publisher = z_session
        .declare_publisher(发布主题)
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 发布者声明失败: {}", e))?;
        
    println!("✅ [神经通路] 物理并网连接成功！下发通道 -> 主题 [{}]", 发布主题);

    // 4. 异步事件驱动循环
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() == "cmd" {
                    // ---------------------------------------------------------
                    // [阶段 A]：从 DORA 零拷贝提取 Arrow 浮点数组
                    // ---------------------------------------------------------
                    let 控制量: Vec<f32> = into_vec(&data).context("❌ Arrow 数组解析失败")?;
                    if 控制量.len() < 2 {
                        eprintln!("⚠️ 接收到的控制指令长度不足 2");
                        continue;
                    }

                    let 指令 = 运动指令 {
                        v: 控制量[0],
                        w: 控制量[1],
                    };

                    // ---------------------------------------------------------
                    // [阶段 B]：极致压缩序列化 (Postcard)
                    // ---------------------------------------------------------
                    let 序列化字节 = postcard::to_allocvec(&指令).context("❌ Postcard 序列化失败")?;

                    // ---------------------------------------------------------
                    // [阶段 C]：通过 Zenoh 下发至 ESP32-C6 脊髓
                    // ---------------------------------------------------------
                    publisher
                        .put(序列化字节)
                        .res()
                        .await
                        .map_err(|e| eyre!("❌ Zenoh 消息发送失败: {}", e))?;
                }
            }
            Event::Stop(_) => {
                println!("🛑 [神经通路] 接收到 DORA 停止信号，安全关闭 Zenoh 桥接器...");
                break;
            }
            _ => {}
        }
    }

    // ---------------------------------------------------------
    // [阶段 D]：优雅退出与生命周期安全回收
    // ---------------------------------------------------------
    drop(publisher);

    z_session
        .close()
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 会话关闭异常: {}", e))?;
        
    println!("🔌 [神经通路] 会话已安全释放，并网网关优雅退出。");
    Ok(())
}