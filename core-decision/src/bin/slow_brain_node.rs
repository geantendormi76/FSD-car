// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
use dora_node_api::{DoraNode, Event};
use eyre::eyre;

use core_decision::topo_graph::graph::TopologicalGraph;
use core_decision::topo_graph::node::{Pose, TopologicalNode};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🧠 [慢系统] 语义大脑与拓扑建图节点已启动，等待 DORA 共享内存注入...");
    // 1. 接入 DORA 数据流网络
    let (mut _node, mut events) = DoraNode::init_from_env()?;
    // 2. 初始化全简体中文业务逻辑主权对象：拓扑地图
    let mut 拓扑地图 = TopologicalGraph::new();
    let mut 节点计数器 = 0;
    
    // 🎯 里程碑 1.1：建立物理小脑状态缓存
    let mut 最新位姿 = Pose { x: 0.0, y: 0.0, yaw: 0.0 };
    let mut 上一个节点_id: Option<u32> = None;
    let mut 上一个位姿: Option<Pose> = None;

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
                    // 🎯 架构师对齐：利用 downcast_ref 将 DORA 共享内存中的二进制泛型指针直接还原为 StructArray [cite: 1.1.4]
                    let 结构体数组 = data.as_any()
                        .downcast_ref::<dora_node_api::arrow::array::StructArray>()
                        .ok_or_else(|| eyre!("❌ 无法将 DORA 数据转换为 StructArray"))?;
                    
                    let 特征数量 = 结构体数组.len();
                    if 特征数量 == 0 { continue; }

                    // 引入 Array 核心特征
                    use dora_node_api::arrow::array::Array;

                    // 🎯 零拷贝映射：分别将列式存储中的各列数据直接下转型为特定类型数组 [cite: 1.1.4]
                    let x_arr = 结构体数组
                        .column_by_name("x")
                        .ok_or_else(|| eyre!("❌ StructArray 中未找到 'x' 列"))?
                        .as_any()
                        .downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("❌ 无法将 'x' 列转换为 Float32Array"))?;

                    let y_arr = 结构体数组
                        .column_by_name("y")
                        .ok_or_else(|| eyre!("❌ StructArray 中未找到 'y' 列"))?
                        .as_any()
                        .downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("❌ 无法将 'y' 列转换为 Float32Array"))?;

                    let desc_list_arr = 结构体数组
                        .column_by_name("descriptor")
                        .ok_or_else(|| eyre!("❌ StructArray 中未找到 'descriptor' 列"))?
                        .as_any()
                        .downcast_ref::<dora_node_api::arrow::array::FixedSizeListArray>()
                        .ok_or_else(|| eyre!("❌ 无法将 'descriptor' 列转换为 FixedSizeListArray"))?;

                    let desc_values = desc_list_arr
                        .values()
                        .as_any()
                        .downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("❌ 无法将描述子底层数纽转换为 Float32Array"))?;

                    println!(
                        "✅ [慢系统] 成功通过共享内存零拷贝解算出 {} 个 XFeat 骨干特征 (解析延迟 0!)",
                        特征数量
                    );

                    // ---------------------------------------------------------
                    // 🗺️ 拓扑建图与记忆存储
                    // ---------------------------------------------------------
                    节点计数器 += 1;

                    let mut descriptors = Vec::with_capacity(特征数量 * 64);
                    let mut keypoints = Vec::with_capacity(特征数量 * 2);

                    // 🚀 架构师对齐优化：直接读取列数据，由于内存连续分布，循环将极大受益于 CPU 缓存命中率 [cite: 1.1.4]
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
                        pose: 最新位姿.clone(), // 🎯 里程碑 1.1：注入真实物理坐标
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
                            
                            // 🎯 里程碑 2.2：向快大脑发射“人类引力锚点 (Human Prior)”
                            // 将当前站牌的绝对坐标发送给 NMPC，引导其沿着人类示教路线行驶
                            let prior_arr = dora_node_api::arrow::array::Float32Array::from(vec![
                                最新位姿.x, 最新位姿.y, 最新位姿.yaw
                            ]);
                            if let Err(e) = _node.send_output(
                                "human_prior".to_string().into(),
                                dora_node_api::MetadataParameters::default(),
                                prior_arr,
                            ) {
                                eprintln!("❌ 人类引力锚点发送失败: {}", e);
                            }
                            
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
