// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

/*
=================================================================
🛡️ FSD-car V3.0: WSL2 核心并网桥接器 (自适应路由自愈版)
设计哲学: DORA 环境变量清洗 | 8字节比特流零拷贝直通 | 动态路由自愈
=================================================================
*/

use dora_node_api::{DoraNode, Event};
use eyre::eyre;
use zenoh::config::{Config, WhatAmI};
use zenoh::prelude::r#async::*; // 🛡️ 引入 异步解析特质

/// 🎯 2026 自愈核心：运行时直接向 Linux 路由表索要当前的 Windows 网关 IP，彻底解决 NAT 重启 IP 漂移问题
fn get_wsl_gateway_ip() -> Option<String> {
    let output = std::process::Command::new("sh")
        .args(&["-c", "ip route | grep default | awk '{print $3}'"])
        .output()
        .ok()?;
    let ip = String::from_utf8(output.stdout).ok()?.trim().to_string();
    if ip.is_empty() {
        None
    } else {
        Some(ip)
    }
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🔗 [神经通路] 核心并网网关已启动，等待 DORA 共享内存注入...");

    // 1. 接入 DORA 数据流网络 (必须在此处最先初始化，它需要读取 DORA 环境变量)
    let (mut _node, mut events) = DoraNode::init_from_env()?;

    // 🛡️ 架构师避绑架神技：强行从当前进程内存中清洗掉所有 DORA 注入的 Zenoh 环境变量干扰！
    let hijacked_vars: Vec<String> = std::env::vars()
        .map(|(k, _)| k)
        .filter(|k| k.starts_with("ZENOH_"))
        .collect();
        
    for var in hijacked_vars {
        std::env::remove_var(&var); // 强硬切除 DORA 环境变量的寄生干扰
    }
    println!("🧹 [神经通路] DORA 寄生环境变量清洗完成，已恢复业务级通信主权。");

    // 2. 动态捕获 Windows 网关 IP 并对齐端口 17449
    let host_ip = get_wsl_gateway_ip().unwrap_or_else(|| "127.0.0.1".to_string());
    println!("📡 [神经通路] 动态检测到 Windows 宿主机网关 IP: {}", host_ip);

    let mut z_config = Config::default();
    z_config.set_mode(Some(WhatAmI::Client)).expect("❌ Zenoh 设置 Client 模式失败");
    
    // 动态生成连接端点，彻底斩断硬编码
    let endpoint = format!("[\"tcp/{}:17449\"]", host_ip);
    z_config.insert_json5("connect/endpoints", &endpoint).expect("❌ Zenoh 连接配置失败");
    
    // 🎯 2026 SOTA 确定性并网：显式关闭组播，免除代理和网络策略冲突
    z_config.insert_json5("scouting/multicast/enabled", "false").expect("❌ Zenoh 禁用组播失败");
    
    // 打开会话
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
        
    println!("✅ [神经通路] 物理并网连接成功！单播直连通道 -> [tcp/{}:17449]", host_ip);

    // 4. 异步事件驱动循环
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() == "cmd" {
                    // ---------------------------------------------------------
                    // [阶段 A]：使用 DORA 官方 API 逆向还原 8 字节控制量
                    // ---------------------------------------------------------
                    let 裸数据_vec: Vec<u8> = dora_node_api::into_vec(&data)
                        .map_err(|e| eyre!("❌ 无法将 ArrowData 转换为 u8 向量: {}", e))?;
                    
                    let 裸数据 = &裸数据_vec;
                    if 裸数据.len() < 8 {
                        eprintln!("⚠️ 接收到的控制指令长度不足 8 字节，放弃本次下发");
                        continue;
                    }

                    // 零拷贝直通字节流
                    let 零拷贝载荷 = &裸数据[0..8];

                    // ---------------------------------------------------------
                    // [阶段 B]：通过 Zenoh 单播下发至 Windows 物理界网关
                    // ---------------------------------------------------------
                    publisher
                        .put(零拷贝载荷)
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
    // [阶段 C]：优雅退出与生命周期安全回收
    // ---------------------------------------------------------
    drop(publisher);

    z_session
        .close()
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 会话关闭异常: {}", e))?;
        
    println!("🔌 [神经通路] 物理并网单播通道已安全释放，优雅退出。");
    Ok(())
}