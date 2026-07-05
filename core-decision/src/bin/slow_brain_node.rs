// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
use dora_node_api::{DoraNode, Event};
use eyre::eyre;
use core_decision::topo_graph::graph::TopologicalGraph;
use core_decision::topo_graph::node::{Pose, TopologicalNode};

// 🎯 辅助工具：获取载入地图的节点数
fn loaded_speed_nodes_count(g: &TopologicalGraph) -> usize {
    g.nodes.len()
}

// 🎯 辅助工具：自适应转换地图头
fn loaded_header_adaptor(g: TopologicalGraph) -> TopologicalGraph {
    g
}

// 🎯 辅助工具：写保护日志
fn pre_save_error_log(e: &str) {
    eprintln!("❌ 拓扑地图保存失败: {}", e);
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🧠 [慢系统] 语义大脑与拓扑建图节点已启动，等待 DORA 共享内存注入...");
    // 1. 接入 DORA 数据流网络
    let (mut _node, mut events) = DoraNode::init_from_env()?;
    
    // 2. 初始化全简体中文业务逻辑主权对象：拓扑地图
    // 🎯 里程碑 2.3：自适应自愈检测。如果硬盘存在历史地图，一键载入脑海，直接激活“自主寻迹自驾模式”
    let map_path = "topo_memory.json";
    let (mut 拓扑地图, 自主自驾模式) = if std::path::Path::new(map_path).exists() {
        match TopologicalGraph::load_from_file(map_path) {
            Ok(loaded_graph) => {
                println!("🟢 [慢系统] 成功从硬盘唤醒历史图谱记忆！全图共 {} 个站牌。进入 -> 【自主寻迹自驾模式】", loaded_speed_nodes_count(&loaded_graph));
                (loaded_header_adaptor(loaded_graph), true)
            }
            Err(e) => {
                println!("⚠️ [慢系统] 地图文件损坏: {}，自动退入 [示教建图模式]", e);
                (TopologicalGraph::new(), false)
            }
        }
    } else {
        println!("🔵 [慢系统] 未检测到历史地图文件。进入 -> 【人类遥控示教建图模式】");
        (TopologicalGraph::new(), false)
    };

    let mut 节点计数器 = 拓扑地图.nodes.len() as u32;
    
    // 🎯 里程碑 1.1：建立物理小脑状态缓存
    let mut 最新位姿 = Pose { x: 0.0, y: 0.0, yaw: 0.0 };
    let mut 上一个节点_id: Option<u32> = None;
    let mut 上一个位姿: Option<Pose> = None;
    
    // 🎯 里程碑 2.3：自驾寻迹局部指针
    // 允许冗余初始化，以保证在循环首帧中变量的生命周期安全
    #[allow(unused_assignments)]
    let mut 当前锁定目标节点_id = 1u32;

    // 3. 异步事件驱动循环
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                // 🎯 接收 100Hz 物理小脑高频里程计
                if id.as_str() == "odometry" {
                    let odom_arr = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("❌ 无法将 DORA 数据转换为 Float32Array"))?;
                    if odom_arr.len() >= 3 {
                        最新位姿.x = odom_arr.value(0);
                        最新位姿.y = odom_arr.value(1);
                        最新位姿.yaw = odom_arr.value(2);
                    }
                }
                else if id.as_str() == "xfeat_features" {
                    // 🎯 里程碑 2.3：如果在自驾寻迹重放模式下，直接利用高精里程计与拓扑网络检索引力点
                    if 自主自驾模式 {
                        if 拓扑地图.nodes.is_empty() { continue; }
                        
                        // A. 寻找空间几何距离最近的历史地标节点作为定位锚点
                        let mut 最小距离 = f32::INFINITY;
                        let mut 最近节点_id = 1u32;
                        
                        for (&id, node) in &拓扑地图.nodes {
                            let dx = 最新位姿.x - node.pose.x;
                            let dy = 最新位姿.y - node.pose.y;
                            let dist = (dx * dx + dy * dy).sqrt();
                            if dist < 最小距离 {
                                最小距离 = dist;
                                最近节点_id = id;
                            }
                        }
                        
                        // B. 滚动寻迹指针：下一个目标站牌 (Target Waypoint) 为 最近节点 + 1
                        当前锁定目标节点_id = 最近节点_id + 1;
                        
                        if let Some(目标地标) = 拓扑地图.nodes.get(&当前锁定目标节点_id) {
                            // C. 向 DORA 零拷贝总线广播目标引力绝对坐标 [x, y, yaw]
                            let prior_arr = dora_node_api::arrow::array::Float32Array::from(vec![
                                目标地标.pose.x, 目标地标.pose.y, 目标地标.pose.yaw
                            ]);
                            if let Err(e) = _node.send_output(
                                "human_prior".to_string().into(),
                                dora_node_api::MetadataParameters::default(),
                                prior_arr,
                            ) {
                                eprintln!("❌ 人类引力锚点发射失败: {}", e);
                            }
                            println!(
                                "🧭 [慢脑自驾寻迹] 状态: 运行中 | 当前定位锚点: Node_{} | 锁定引力目标 -> Node_{} | 坐标: ({:.2}, {:.2}) | 剩余距离: {:.2}m",
                                最近节点_id, 当前锁定目标节点_id, 目标地标.pose.x, 目标地标.pose.y, 最小距离
                            );
                        } else {
                            // 已经到达最后一个节点
                            println!("🏆 [慢脑自驾寻迹] 恭喜！小车已完全自主安全驶达本次寻迹路线的终点站牌！");
                            // 持续广播终点坐标，稳稳锁住制动
                            if let Some(终点地标) = 拓扑地图.nodes.get(&最近节点_id) {
                                let prior_arr = dora_node_api::arrow::array::Float32Array::from(vec![
                                    终点地标.pose.x, 终点地标.pose.y, 终点地标.pose.yaw
                                ]);
                                let _ = _node.send_output(
                                    "human_prior".to_string().into(),
                                    dora_node_api::MetadataParameters::default(),
                                    prior_arr,
                                );
                            }
                        }
                        continue; // 跳过后续的建图存储逻辑，防止在自驾时污染原始地图数据！
                    }

                    // 下面是原有的 [人类遥控示教建图模式] 逻辑
                    let 结构体数组 = data.as_any()
                        .downcast_ref::<dora_node_api::arrow::array::StructArray>()
                        .ok_or_else(|| eyre!("❌ 无法将 DORA 数据转换为 StructArray"))?;
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

                    // 🎯 里程碑 1.2：计算相对距离，编织有向拓扑路网
                    if let Some(prev_pose) = &上一个位姿 {
                        let dx = 最新位姿.x - prev_pose.x;
                        let dy = 最新位姿.y - prev_pose.y;
                        let distance = (dx * dx + dy * dy).sqrt();

                        // 空间稀疏化：只有当移动超过 1.0 米时，才记录新站牌，防止原地堆积导致内存爆炸
                        if distance >= 1.0 {
                            拓扑地图.add_node(新地标.clone());
                            // 计算相对偏航角 (驶向目标节点的期望角度)
                            let relative_yaw = dy.atan2(dx) - prev_pose.yaw;
                            拓扑地图.add_edge(上一个节点_id.unwrap(), 节点计数器, distance, relative_yaw);

                            println!(
                                "🗺️ [慢系统] 新增拓扑节点 {}，建立有向边 {} -> {} (距离: {:.2}m, 相对偏航: {:.2}rad)",
                                节点计数器, 上一个节点_id.unwrap(), 节点计数器, distance, relative_yaw
                            );

                            上一个节点_id = Some(节点计数器);
                            上一个位姿 = Some(最新位姿.clone());
                        } else {
                            // 距离太近，回滚计数器，丢弃该帧
                            节点计数器 -= 1;
                        }
                    } else {
                        // 录制第一个原点站牌
                        拓扑地图.add_node(新地标.clone());
                        上一个节点_id = Some(节点计数器);
                        上一个位姿 = Some(最新位姿.clone());
                        println!("🗺️ [慢系统] 建立拓扑原点节点 {}，坐标: ({:.2}, {:.2})", 节点计数器, 最新位姿.x, 最新位姿.y);
                    }
                }
            }
            Event::Stop(_) => {
                if !自主自驾模式 {
                    println!("🛑 [慢系统] 接收到 DORA 停止信号，正在将拓扑记忆持久化到硬盘...");
                    if let Err(e) = 拓扑地图.save_to_file("topo_memory.json") {
                        pre_save_error_log(&e);
                    } else {
                        println!("💾 拓扑地图已安全保存至 topo_memory.json");
                    }
                } else {
                    println!("🛑 [慢系统] 接收到 DORA 停止信号，自驾寻迹模式安全下线，已对历史图谱 `topo_memory.json` 启动写保护！");
                }
                break;
            }
            _ => {}
        }
    }
    Ok(())
}