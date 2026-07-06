// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
use dora_node_api::{DoraNode, Event};
use eyre::eyre;
use core_decision::topo_graph::graph::TopologicalGraph;
use core_decision::topo_graph::node::{Pose, TopologicalNode};

fn loaded_speed_nodes_count(g: &TopologicalGraph) -> usize {
    g.nodes.len()
}

fn loaded_header_adaptor(g: TopologicalGraph) -> TopologicalGraph {
    g
}

fn pre_save_error_log(e: &str) {
    eprintln!("❌ 拓扑地图保存失败: {}", e);
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🧠 [慢系统] 语义大脑与拓扑建图节点已启动，等待 DORA 共享内存注入...");
    let (mut _node, mut events) = DoraNode::init_from_env()?;

    let map_path = "topo_memory.json";
    let 强制建图模式 = std::env::var("FSD_MODE").unwrap_or_default() == "mapping";
    let (mut 拓扑地图, 自主自驾模式) = if 强制建图模式 {
        println!("🔵 [慢系统] 检测到 FSD_MODE=mapping，强制锁定 -> 【人类遥控示教建图模式】");
        (TopologicalGraph::new(), false)
    } else if std::path::Path::new(map_path).exists() {
        match TopologicalGraph::load_from_file(map_path) {
            Ok(loaded_graph) => {
                println!("🟢 [慢系统] 成功载入历史图谱。共 {} 个站牌。进入 -> 【自主寻迹自驾模式】", loaded_speed_nodes_count(&loaded_graph));
                (loaded_header_adaptor(loaded_graph), true)
            }
            Err(e) => {
                println!("⚠️ [慢系统] 地图文件损坏: {}，退入 [示教建图模式]", e);
                (TopologicalGraph::new(), false)
            }
        }
    } else {
        println!("🔵 [慢系统] 未检测到地图文件。进入 -> 【人类遥控示教建图模式】");
        (TopologicalGraph::new(), false)
    };

    let mut 节点计数器 = 拓扑地图.nodes.len() as u32;
    let mut 最新位姿 = Pose { x: 0.0, y: 0.0, yaw: 0.0 };
    let mut 上一个节点_id: Option<u32> = None;
    let mut 上一个位姿: Option<Pose> = None;
    
    // 🛡️ 状态缓存：记录上一次定位到的黄金最近节点，用于翻页卡锁
    let mut 上一次定位的最近节点_id = 1u32;

    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() == "odometry" {
                    let odom_arr = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("❌ 无法解析 odometry 数据"))?;
                    if odom_arr.len() >= 3 {
                        最新位姿.x = odom_arr.value(0);
                        最新位姿.y = odom_arr.value(1);
                        最新位姿.yaw = odom_arr.value(2);
                    }
                }
                else if id.as_str() == "xfeat_features" {
                    if 自主自驾模式 {
                        if 拓扑地图.nodes.is_empty() { continue; }
                        
                        // A. 寻找空间几何距离最近的历史地标节点作为定位锚点
                        let mut 最小距离 = f32::INFINITY;
                        // 默认继承上一次定位结果，防止在特征偏稀疏的盲区内丢失锁 [cite: 1.1.1]
                        let mut 最近节点_id = 上一次定位的最近节点_id; 
                        
                        for (&id, node) in &拓扑地图.nodes {
                            let dx = 最新位姿.x - node.pose.x;
                            let dy = 最新位姿.y - node.pose.y;
                            let dist = (dx * dx + dy * dy).sqrt();
                            
                            // 🛡️ 第二道防线：眼脚空间先验校验门 (Odometry Spatial Gating) [cite: 1.2.2]
                            // 物理过滤：强行抛弃几何空间距离大于 1.5 米的“视觉穿透致幻点” [cite: 1.2.2]
                            if dist > 1.5 {
                                continue;
                            }
                            
                            if dist < 最小距离 {
                                最小距离 = dist;
                                最近节点_id = id;
                            }
                        }

                        // 🛡️ 第一道防线：状态翻页锁 (Topological State Transition Gate) [cite: 1.1.1]
                        // 连环画约束：单帧内只允许最多向前推进 1 页，彻底扼杀视觉错配造成的“跳关和抢弯” [cite: 1.1.1]
                        if 最近节点_id > 上一次定位的最近节点_id + 1 {
                            最近节点_id = 上一次定位的最近节点_id + 1;
                        }
                        // 确保不发生物理回退
                        if 最近节点_id < 上一次定位的最近节点_id {
                            最近节点_id = 上一次定位的最近节点_id;
                        }
                        
                        上一次定位的最近节点_id = 最近节点_id;

                        // B. 滚动寻迹指针：下一个目标站牌为 最近节点 + 1
                        let 当前锁定目标节点_id = 最近节点_id + 1;
                        if let Some(目标地标) = 拓扑地图.nodes.get(&当前锁定目标节点_id) {
                            // C. 广播目标引力绝对坐标
                            let prior_arr = dora_node_api::arrow::array::Float32Array::from(vec![
                                目标地标.pose.x, 目标地标.pose.y, 目标地标.pose.yaw
                            ]);
                            let _ = _node.send_output(
                                "human_prior".to_string().into(),
                                dora_node_api::MetadataParameters::default(),
                                prior_arr,
                            );
                            println!(
                                "🧭 [慢脑自驾寻迹] 定位锚点: Node_{} | 锁定引力目标 -> Node_{} | 坐标: ({:.2}, {:.2}) | 剩余距离: {:.2}m",
                                最近节点_id, 当前锁定目标节点_id, 目标地标.pose.x, 目标地标.pose.y, 最小距离
                            );
                        } else {
                            // 🎯 🌟 SOTA 级环境自愈重置：已经到达最后一个节点
                            println!("🏆 [慢脑自驾寻迹] 恭喜！小车已成功驶达本次寻迹路线的终点站牌！");
                            println!("🔄 [慢脑自驾寻迹] 正在向 DORA 广播 simulation_reset 物理重置指令...");
                            
                            let reset_arr = dora_node_api::arrow::array::Float32Array::from(vec![1.0f32]);
                            let _ = _node.send_output(
                                "simulation_reset".to_string().into(),
                                dora_node_api::MetadataParameters::default(),
                                reset_arr,
                            );

                            // 🎯 慢脑自重置：瞬间重置历史位姿缓存与最近节点状态缓存，寻迹指针由下一帧里程计自愈拉起
                            最新位姿 = Pose { x: 0.0, y: 0.0, yaw: 0.0 };
                            上一个节点_id = None;
                            上一个位姿 = None;
                            上一次定位的最近节点_id = 1u32;
                        }
                        continue;
                    }

                    // [人类遥控示教建图模式] 逻辑不变
                    let 结构体数组 = data.as_any()
                        .downcast_ref::<dora_node_api::arrow::array::StructArray>()
                        .ok_or_else(|| eyre!("❌ 无法解析 StructArray"))?;
                    let 特征数量 = 结构体数组.len();
                    if 特征数量 == 0 { continue; }
                    
                    use dora_node_api::arrow::array::Array;
                    let x_arr = 结构体数组.column_by_name("x").unwrap().as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();
                    let y_arr = 结构体数组.column_by_name("y").unwrap().as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();
                    let desc_list_arr = 结构体数组.column_by_name("descriptor").unwrap().as_any().downcast_ref::<dora_node_api::arrow::array::FixedSizeListArray>().unwrap();
                    let desc_values = desc_list_arr.values().as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();
                    
                    节点计数器 += 1;
                    let mut descriptors = Vec::with_capacity(特征数量 * 64);
                    let mut keypoints = Vec::with_capacity(特征数量 * 2);
                    for i in 0..特征数量 {
                        keypoints.push(x_arr.value(i));
                        keypoints.push(y_arr.value(i));
                        let offset = i * 64;
                        for j in 0..64 {
                            descriptors.push(desc_values.value(offset + j));
                        }
                    }
                    
                    let 新地标 = TopologicalNode {
                        id: 节点计数器,
                        name: format!("自动探索地标_{}", 节点计数器),
                        pose: 最新位姿.clone(),
                        descriptors,
                        keypoints,
                    };

                    if let Some(prev_pose) = &上一个位姿 {
                        let dx = 最新位姿.x - prev_pose.x;
                        let dy = 最新位姿.y - prev_pose.y;
                        let distance = (dx * dx + dy * dy).sqrt();
                        if distance >= 1.0 {
                            拓扑地图.add_node(新地标.clone());
                            let relative_yaw = dy.atan2(dx) - prev_pose.yaw;
                            拓扑地图.add_edge(上一个节点_id.unwrap(), 节点计数器, distance, relative_yaw);
                            println!(
                                "🗺️ [慢系统] 新增拓扑节点 {}，建立有向边 {} -> {} (距离: {:.2}m, 相对偏航: {:.2}rad)",
                                节点计数器, 上一个节点_id.unwrap(), 节点计数器, distance, relative_yaw
                            );
                            上一个节点_id = Some(节点计数器);
                            上一个位姿 = Some(最新位姿.clone());
                        } else {
                            节点计数器 -= 1;
                        }
                    } else {
                        拓扑地图.add_node(新地标.clone());
                        上一个节点_id = Some(节点计数器);
                        上一个位姿 = Some(最新位姿.clone());
                        println!("🗺️ [慢系统] 建立拓扑原点节点 {}，坐标: ({:.2}, {:.2})", 节点计数器, 最新位姿.x, 最新位姿.y);
                    }
                }
            }
            Event::Stop(_) => {
                if !自主自驾模式 {
                    println!("🛑 [慢系统] 收到停止信号，保存地图中...");
                    if let Err(e) = 拓扑地图.save_to_file("topo_memory.json") {
                        pre_save_error_log(&e);
                    } else {
                        println!("💾 拓扑地图已安全保存至 topo_memory.json");
                    }
                } else {
                    println!("🛑 [慢系统] 收到 DORA 停止信号，自驾寻迹模式安全下线。已启动写保护。");
                }
                break;
            }
            _ => {}
        }
    }
    Ok(())
}