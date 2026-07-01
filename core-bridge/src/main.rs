// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

/*
=================================================================
🛡️ FSD-car V3.0: WSL2 核心并网桥接器 (非阻塞高确定性并网版)
设计哲学: 环境变量强力清洗 | 异步 try_send 零阻塞自愈 | 8字节/KB特征多路复用
=================================================================
*/

use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use zenoh::config::{Config, WhatAmI};
use zenoh::prelude::r#async::*; // 🛡️ 引入 异步解析特质

/// 运行时直接从 Linux 路由表抓取 Windows 宿主机在 NAT 子网中的网关 IP
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
    z_config.insert_json5("scouting/multicast/enabled", "false").expect("❌ Zenoh 禁用组播失败");
    
    // 打开会话
    let z_session = zenoh::open(z_config)
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 会话打开失败: {}", e))?;
    
    // 3. 声明发送控制指令的 Zenoh 发布者
    let 发布主题 = "fsd/spinal_cord/cmd_vel";
    let publisher = z_session
        .declare_publisher(发布主题)
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 发布者声明失败: {}", e))?;
            
    println!("✅ [神经通路] 物理控制并网连接成功！单播直连通道 -> [tcp/{}:17449]", host_ip);

    // 4. 建立多线程并发管道：异步接收 Windows 发送过来的避障逃逸矢量
    let (tx, mut rx) = tokio::sync::mpsc::channel::<Vec<u8>>(30);
    
    // 🎯 核心重构：声明订阅 Windows 17449 单播发来的仿生势场逃逸力
    let _sub = z_session
        .declare_subscriber("fsd/perception/frog_eye")
        .callback(move |sample| {
            let binding = sample.payload.contiguous();
            let payload: &[u8] = binding.as_ref();
            if payload.len() == 8 {
                // 🛡️ 架构师自愈：改用 try_send 避免因阻塞 Tokio 线程引发的 panic 崩溃
                let _ = tx.try_send(payload.to_vec());
            }
        })
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 注册 17449 避障力监听器失败: {}", e))?;
        
    println!("✅ [神经通路] 避障势场接收通道就绪！已注册 -> [fsd/perception/frog_eye]");

    // 🎯 新增：声明订阅 Windows 发来的 XFeat 稀疏特征二进制契约
    let (tx_xfeat, mut rx_xfeat) = tokio::sync::mpsc::channel::<Vec<u8>>(5);
    let _sub_xfeat = z_session
        .declare_subscriber("fsd/perception/xfeat_features")
        .callback(move |sample| {
            let binding = sample.payload.contiguous();
            // 🛡️ 架构师自愈：改用 try_send 避免因阻塞 Tokio 线程引发的 panic 崩溃
            let _ = tx_xfeat.try_send(binding.as_ref().to_vec());
        })
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 注册 XFeat 监听器失败: {}", e))?;
        
    println!("✅ [神经通路] XFeat 稀疏特征接收通道就绪！已注册 -> [fsd/perception/xfeat_features]");

    // 5. 🎯 异步事件驱动选择器 (0-Copy 双向直通循环)
    loop {
        tokio::select! {
            // 通道 A：监听 DORA 快大脑输出的底层电机控制量，零拷贝直接透传给 Windows
            dora_event = events.recv_async() => {
                match dora_event {
                    Some(Event::Input { id, data, .. }) => {
                        if id.as_str() == "cmd" {
                            let 裸数据_vec: Vec<u8> = dora_node_api::into_vec(&data)
                                .map_err(|e| eyre!("❌ 无法将 ArrowData 转换为 u8 向量: {}", e))?;
                            let 零拷贝载荷 = &裸数据_vec[0..8];
                            
                            // 极速单播直发
                            if let Err(e) = publisher.put(零拷贝载荷).res().await {
                                eprintln!("⚠️ Zenoh 控制下发失败: {:?}", e);
                            }
                        }
                    }
                    Some(Event::Stop(_)) => {
                        println!("🛑 [神经通路] 接收到 DORA 全局停止信号，安全卸载网关...");
                        break;
                    }
                    None => break,
                    _ => {}
                }
            }
            
            // 通道 B：监听到 Windows 网关通过 17449 传过来的 8 字节避障力，零拷贝塞回 DORA 共享内存总线
            Some(fe_payload) = rx.recv() => {
                if let Err(e) = _node.send_output_bytes(
                    "obstacle_force".to_string().into(),
                    MetadataParameters::default(),
                    8,
                    &fe_payload,
                ) {
                    eprintln!("❌ DORA 势场力广播失败: {}", e);
                }
            }
            
            // 通道 C：监听到 Windows 发来的 XFeat 二进制契约，塞回 DORA 共享内存总线
            Some(xfeat_payload) = rx_xfeat.recv() => {
                if let Err(e) = _node.send_output_bytes(
                    "xfeat_features".to_string().into(),
                    MetadataParameters::default(),
                    xfeat_payload.len(),
                    &xfeat_payload,
                ) {
                    eprintln!("❌ DORA XFeat 特征广播失败: {}", e);
                }
            }
        }
    }

    // ---------------------------------------------------------
    // 6. 优雅释放会话
    // ---------------------------------------------------------
    drop(publisher);
    drop(_sub);
    drop(_sub_xfeat); // 显式释放

    z_session
        .close()
        .res()
        .await
        .map_err(|e| eyre!("❌ Zenoh 会话关闭异常: {}", e))?;
        
    println!("🔌 [神经通路] 物理通道与避障总线已安全注销。");
    Ok(())
}