// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
use dora_node_api::{DoraNode, Event};
use eyre::eyre;
use core_decision::topo_graph::graph::TopologicalGraph;
use core_decision::topo_graph::node::{Pose, TopologicalNode};

// 🛡️ 核心并网：引入视觉感知显微镜与特征契约
use core_perception::perception::xfeat_engine::稀疏特征点;
use core_perception::perception::matcher::仿生匹配器;

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

    // 🎯 商业级 RPA 状态机：去程 -> 驻车 -> 返程
    #[derive(PartialEq)]
    enum 调度状态 { 去程, 驻车静默, 返程, 任务结束 }
    let mut 当前调度状态 = 调度状态::去程;
    let mut 驻车开始时间: Option<std::time::Instant> = None;
    let 最大节点数 = 拓扑地图.nodes.len() as u32;
    let mut 循环计数 = 0u64;

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
                    let 结构体数组 = data.as_any()
                        .downcast_ref::<dora_node_api::arrow::array::StructArray>()
                        .ok_or_else(|| eyre!("❌ 无法将 DORA 数据转换为 StructArray"))?;
                    let 特征数量 = 结构体数组.len();
                    if 特征数量 == 0 { continue; }

                    if 自主自驾模式 {
                        if 拓扑地图.nodes.is_empty() { continue; }
                        
                        use dora_node_api::arrow::array::Array;
                        let x_arr = 结构体数组.column_by_name("x").unwrap().as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();
                        let y_arr = 结构体数组.column_by_name("y").unwrap().as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();
                        let score_arr = 结构体数组.column_by_name("score").unwrap().as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();
                        let desc_list_arr = 结构体数组.column_by_name("descriptor").unwrap().as_any().downcast_ref::<dora_node_api::arrow::array::FixedSizeListArray>().unwrap();
                        let desc_values = desc_list_arr.values().as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>().unwrap();

                        // A. 零拷贝解构 DORA 共享内存，构建当前帧实时特征点集 [cite: 1.1.4]
                        let mut 实时特征 = Vec::with_capacity(特征数量);
                        for i in 0..特征数量 {
                            let mut 描述子 = vec![0.0f32; 64];
                            let offset = i * 64;
                            for j in 0..64 {
                                描述子[j] = desc_values.value(offset + j);
                            }
                            实时特征.push(稀疏特征点 {
                                x: x_arr.value(i),
                                y: y_arr.value(i),
                                置信度: score_arr.value(i),
                                描述子,
                            });
                        }

                        // B. 双通道自适应空间先验重定位检索
                        let mut 最小距离 = f32::INFINITY;
                        let mut 最大匹配内点数 = 0;
                        let mut 最佳匹配节点_id = 上一次定位的最近节点_id; // 默认继承历史

                        for (&id, node) in &拓扑地图.nodes {
                            // 1. 空间粗筛：抛弃物理跨度大于 5.0 米的超远节点，防止无谓的特征解算，大开销避让
                            let dx = 最新位姿.x - node.pose.x;
                            let dy = 最新位姿.y - node.pose.y;
                            let odom_dist = (dx * dx + dy * dy).sqrt();
                            if odom_dist > 5.0 {
                                continue;
                            }

                            // 2. 逆向重构该地图节点的多维历史描述子
                            let n_features = node.descriptors.len() / 64;
                            let mut 历史特征 = Vec::with_capacity(n_features);
                            for i in 0..n_features {
                                let mut desc = vec![0.0f32; 64];
                                desc.copy_from_slice(&node.descriptors[i*64..(i+1)*64]);
                                历史特征.push(稀疏特征点 {
                                    x: node.keypoints[i*2],
                                    y: node.keypoints[i*2 + 1],
                                    置信度: 1.0,
                                    描述子: desc,
                                });
                            }

                            // 3. 执行双向 MNN 交叉比对
                            let 原始匹配 = 仿生匹配器::交叉匹配(&实时特征, &历史特征, 0.81);
                            
                            // 4. 执行 RANSAC 外点说谎者剪枝 [cite: 1.1.2]
                            if 原始匹配.len() >= 8 {
                                if let Ok(干净匹配) = 仿生匹配器::几何纠偏过滤(&实时特征, &历史特征, &原始匹配, 3.0) {
                                    let 内点数 = 干净匹配.len();
                                    
                                    // 🛡️ 第二道防线：自适应弹性空间校验门 (Confidence-Proportional Gating) [cite: 1.2.2]
                                    let 弹性空间门限 = if 内点数 >= 15 { 5.0f32 } else { 1.2f32 };
                                    
                                    if odom_dist <= 弹性空间门限 {
                                        if 内点数 > 最大匹配内点数 {
                                            最大匹配内点数 = 内点数;
                                            最佳匹配节点_id = id;
                                            最小距离 = odom_dist;
                                        }
                                    }
                                }
                            }
                        }

                        // 3. 确保视觉重定位判定通过安全阈值
                        let mut 最近节点_id = 上一次定位的最近节点_id;
                        if 最大匹配内点数 >= 10 {
                            最近节点_id = 最佳匹配节点_id;
                        } else {
                            // 降级使用几何距离作为当前帧的最小距离
                            if let Some(node) = 拓扑地图.nodes.get(&最近节点_id) {
                                let dx = 最新位姿.x - node.pose.x;
                                let dy = 最新位姿.y - node.pose.y;
                                最小距离 = (dx * dx + dy * dy).sqrt();
                            }
                        }

                        // 🛡️ 第一道防线：状态翻页锁 (Topological State Transition Gate) [cite: 1.1.1]
                        // 🛡️ 第一道防线：状态翻页锁 (Topological State Transition Gate)
                        // 🎯 架构师重塑：根据当前调度状态，动态调整物理卡锁的方向，支持双向寻迹
                        match 当前调度状态 {
                            调度状态::去程 => {
                                if 最近节点_id > 上一次定位的最近节点_id + 1 { 最近节点_id = 上一次定位的最近节点_id + 1; }
                                if 最近节点_id < 上一次定位的最近节点_id { 最近节点_id = 上一次定位的最近节点_id; }
                            },
                            调度状态::返程 => {
                                // 返程时，节点 ID 递减
                                if 最近节点_id < 上一次定位的最近节点_id.saturating_sub(1) { 最近节点_id = 上一次定位的最近节点_id.saturating_sub(1); }
                                if 最近节点_id > 上一次定位的最近节点_id { 最近节点_id = 上一次定位的最近节点_id; }
                            },
                            _ => {
                                // 驻车或结束时，锁死当前节点
                                最近节点_id = 上一次定位的最近节点_id;
                            }
                        }
                        上一次定位的最近节点_id = 最近节点_id;

                        // B. 商业级状态机流转：去程 -> 驻车静默 -> 返程
                        if 当前调度状态 == 调度状态::去程 && 最近节点_id >= 最大节点数 {
                            println!("📦 [业务调度] 已抵达终点 (Node_{})！进入【驻车静默】状态，等待装载货物...", 最近节点_id);
                            当前调度状态 = 调度状态::驻车静默;
                            驻车开始时间 = Some(std::time::Instant::now());
                        } else if 当前调度状态 == 调度状态::驻车静默 {
                            if let Some(start_time) = 驻车开始时间 {
                                let 驻车时长 = start_time.elapsed().as_secs();
                                if 驻车时长 > 10 { // 模拟 10 秒装货/静默时间，后续可改为接收外部指令
                                    println!("🚀 [业务调度] 货物装载完毕/静默结束！启动【返程】模式，目标：原点！");
                                    当前调度状态 = 调度状态::返程;
                                } else {
                                    if 循环计数 % 10 == 0 {
                                        println!("⏳ [业务调度] 驻车静默中... 已等待 {} 秒", 驻车时长);
                                    }
                                }
                            }
                        } else if 当前调度状态 == 调度状态::返程 && 最近节点_id <= 1 {
                            if 循环计数 % 50 == 0 {
                                println!("🏆 [业务调度] 任务圆满完成！小车已安全返回原点。进入【任务结束】休眠状态。");
                            }
                            当前调度状态 = 调度状态::任务结束;
                        }

                        // C. 滚动寻迹指针：根据状态机决定下一个目标站牌
                        let 当前锁定目标节点_id = match 当前调度状态 {
                            调度状态::去程 => 最近节点_id + 1,
                            调度状态::返程 => 最近节点_id.saturating_sub(1).max(1),
                            _ => 最近节点_id, // 驻车或结束时，目标就是当前节点，保持静止
                        };

                        if let Some(目标地标) = 拓扑地图.nodes.get(&当前锁定目标节点_id) {
                            // D. 广播目标引力绝对坐标
                            let prior_arr = dora_node_api::arrow::array::Float32Array::from(vec![
                                目标地标.pose.x, 目标地标.pose.y, 目标地标.pose.yaw
                            ]);
                            let _ = _node.send_output(
                                "human_prior".to_string().into(),
                                dora_node_api::MetadataParameters::default(),
                                prior_arr,
                            );
                            
                            let 状态文本 = match 当前调度状态 {
                                调度状态::去程 => "去程寻迹",
                                调度状态::返程 => "返程归家",
                                调度状态::驻车静默 => "驻车装货",
                                调度状态::任务结束 => "任务结束",
                            };
                            
                            循环计数 += 1;
                            if 循环计数 % 10 == 0 {
                                println!(
                                    "🧭 [慢脑-{}] 定位锚点: Node_{} | 锁定引力目标 -> Node_{} | 坐标: ({:.2}, {:.2}) | 剩余距离: {:.2}m",
                                    状态文本, 最近节点_id, 当前锁定目标节点_id, 目标地标.pose.x, 目标地标.pose.y, 最小距离
                                );
                            }
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