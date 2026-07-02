// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
use dora_node_api::{DoraNode, Event};
use eyre::eyre;

use core_decision::topo_graph::graph::TopologicalGraph;
use core_decision::topo_graph::node::{Pose, TopologicalNode};
use core_perception::perception::xfeat_engine::稀疏特征点; // 仅复用数据结构

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🧠 [慢系统] 语义大脑与拓扑建图节点已启动，等待 DORA 共享内存注入...");

    // 1. 接入 DORA 数据流网络
    let (mut _node, mut events) = DoraNode::init_from_env()?;

    // 2. 初始化全简体中文业务逻辑主权对象：拓扑地图
    let mut 拓扑地图 = TopologicalGraph::new();
    let mut 节点计数器 = 0;

    // 3. 异步事件驱动循环
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() == "xfeat_features" {
                    // 🎯 核心重构：直接解析 Windows 端传来的二进制特征契约
                    let 裸数据_vec: Vec<u8> = dora_node_api::into_vec(&data)
                        .map_err(|e| eyre!("❌ 无法解析 DORA 特征数据: {}", e))?;
                        
                    if 裸数据_vec.len() < 4 { continue; }
                    
                    let 特征数量 = u32::from_le_bytes(裸数据_vec[0..4].try_into().unwrap()) as usize;
                    let mut offset = 4;
                    let mut 特征结果 = Vec::with_capacity(特征数量);
                    
                    for _ in 0..特征数量 {
                        if offset + 268 > 裸数据_vec.len() { break; } // 防越界保护
                        
                        let x = f32::from_le_bytes(裸数据_vec[offset..offset+4].try_into().unwrap()); offset += 4;
                        let y = f32::from_le_bytes(裸数据_vec[offset..offset+4].try_into().unwrap()); offset += 4;
                        let score = f32::from_le_bytes(裸数据_vec[offset..offset+4].try_into().unwrap()); offset += 4;
                        
                        let mut desc = vec![0.0f32; 64];
                        for i in 0..64 {
                            desc[i] = f32::from_le_bytes(裸数据_vec[offset..offset+4].try_into().unwrap()); 
                            offset += 4;
                        }
                        
                        特征结果.push(稀疏特征点 {
                            x,
                            y,
                            置信度: score,
                            描述子: desc,
                        });
                    }

                    println!(
                        "✅ [慢系统] 成功接收并解析 {} 个 XFeat 骨干特征点 (0 图像解码开销!)",
                        特征结果.len()
                    );

                    // ---------------------------------------------------------
                    // 🗺️ 拓扑建图逻辑：将当前帧特征存入记忆金库
                    // ---------------------------------------------------------
                    节点计数器 += 1;

                    let mut descriptors = Vec::with_capacity(特征结果.len() * 64);
                    let mut keypoints = Vec::with_capacity(特征结果.len() * 2);

                    for pt in 特征结果 {
                        keypoints.push(pt.x);
                        keypoints.push(pt.y);
                        descriptors.extend(pt.描述子);
                    }

                    let 新地标 = TopologicalNode {
                        id: 节点计数器,
                        name: format!("自动探索地标_{}", 节点计数器),
                        pose: Pose {
                            x: 0.0,
                            y: 0.0,
                            yaw: 0.0,
                        }, // 实际应由里程计/SLAM提供
                        descriptors,
                        keypoints,
                    };

                    拓扑地图.add_node(新地标);
                    println!(
                        "🗺️ [慢系统] 拓扑地图已更新，当前脑海记忆节点数: {}",
                        拓扑地图.nodes.len()
                    );
                }
            }
            Event::Stop(_) => {
                println!("🛑 [慢系统] 接收到 DORA 停止信号，正在将拓扑记忆持久化到硬盘...");
                if let Err(e) = 拓扑地图.save_to_file("topo_memory.json") {
                    eprintln!("❌ 拓扑地图保存失败: {}", e);
                } else {
                    println!("💾 拓扑地图已安全保存至 topo_memory.json");
                }
                break;
            }
            _ => {}
        }
    }

    Ok(())
}
