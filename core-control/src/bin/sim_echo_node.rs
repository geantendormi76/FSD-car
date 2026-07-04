// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

/*
=================================================================
🛡️ FSD-car V3.1: DORA 神经反射弧时序诊断节点 (异步自愈版)
设计哲学: 毫秒级时序透视 | 传输抖动审计 | 极速 Echo 反馈
=================================================================
*/

use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use std::time::Instant;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🛰️  [Rust 诊断端] 神经反射时序审计节点已启动...");

    // 1. 接入数据流网络
    let (mut node, mut events) = DoraNode::init_from_env()?;

    let mut 循环计数: u64 = 0;
    let mut 上一帧接收时间 = Instant::now();
    let mut 时间差累计_ms = 0.0;
    let mut 最大时间差_ms = 0.0;
    let mut 最小时间差_ms = f64::MAX;

    // 2. 接收事件循环 (使用异步 .await 实现非阻塞高效装载)
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() == "obstacle_force" {
                    循环计数 += 1;
                    let 现在 = Instant::now();
                    
                    // 计算与上一帧到达的真实物理时间差 (Jitter 诊断)
                    let 间隔时间_ms = 现在.duration_since(上一帧接收时间).as_secs_f64() * 1000.0;
                    上一帧接收时间 = 现在;

                    if 循环计数 > 5 { // 略过前 5 帧的温启动抖动
                        时间差累计_ms += 间隔时间_ms;
                        if 间隔时间_ms > 最大时间差_ms { 最大时间差_ms = 间隔时间_ms; }
                        if 间隔时间_ms < 最小时间差_ms { 最小时间差_ms = 间隔时间_ms; }
                    }

                    // 极速解析并重塑虚拟势场力
                    let 裸数据_vec: Vec<u8> = dora_node_api::into_vec(&data)
                        .map_err(|e| eyre!("❌ 无法解析 DORA 数据: {}", e))?;
                    
                    if 裸数据_vec.len() >= 8 {
                        let _f_x = f32::from_le_bytes(裸数据_vec[0..4].try_into().unwrap());
                        let _f_y = f32::from_le_bytes(裸数据_vec[4..8].try_into().unwrap());
                    }

                    // 每 100 帧输出一次时序统计报告
                    if 循环计数 % 100 == 0 && 循环计数 > 5 {
                        let 平均间隔 = 时间差累计_ms / 95.0;
                        println!(
                            "[Rust 时序探针] 接收帧数: {:<6} | 平均到达间隔: {:.2} ms | Jitter范围: [{:.2} ms - {:.2} ms]",
                            循环计数, 平均间隔, 最小时间差_ms, 最大时间差_ms
                        );
                        // 重置统计滑动窗口
                        时间差累计_ms = 0.0;
                        最大时间差_ms = 0.0;
                        最小时间差_ms = f64::MAX;
                    }

                    // 🎯 极速二进制回传 Echo 指令：驱动小车向前缓行 (v=0.1, w=0.0)
                    let 运动指令_裸数组 = [0.1f32, 0.0f32];
                    let 裸内存切片: &[u8] = unsafe {
                        std::slice::from_raw_parts(运动指令_裸数组.as_ptr() as *const u8, 8)
                    };
                    
                    if let Err(e) = node.send_output_bytes(
                        "control_cmd".to_string().into(),
                        MetadataParameters::default(),
                        8,
                        裸内存切片,
                    ) {
                        eprintln!("❌ 反馈指令 Echo 失败: {}", e);
                    }
                }
            }
            Event::Stop(_) => {
                println!("🛑 [Rust 诊断端] 收到 DORA 停止信号，退出诊断。");
                break;
            }
            _ => {}
        }
    }

    Ok(())
}