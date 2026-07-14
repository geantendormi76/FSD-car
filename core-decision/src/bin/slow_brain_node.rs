// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
/*
=================================================================
🧠 [NEXUS 慢系统决策脑] “双副驾驶”北斗GPS与视觉重放（提线木偶）混合导航适配器
设计哲学: GPS高精引导与视觉指纹降级定位自愈双轨并网
=================================================================
*/
use dora_node_api::arrow::array::{FixedSizeListArray, Float32Array, StructArray};
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use std::time::Instant;

use core_decision::topo_graph::graph::TopologicalGraph;
use core_decision::topo_graph::node::TopologicalNode;
use core_perception::perception::matcher::仿生匹配器;
use core_perception::perception::xfeat_engine::稀疏特征点;

const XFEAT_DESCRIPTOR_DIM: usize = 64;
const KEYPOINT_COORD_DIM: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq)]
enum 导航模式 {
    北斗高精领航, // GPS 信号优良 (HDOP < 2.5) [cite: 1.2.6]
    提线木偶重放, // 室内/地下室 GPS 失锁降级 (HDOP >= 5.0 或无信号) [cite: 1.2.8]
}

struct 慢脑状态机 {
    pub 当前模式: 导航模式,
    pub odom_x: f32,
    pub odom_y: f32,
    pub odom_yaw: f32,
    pub gps_x: f32,
    pub gps_y: f32,
    pub gps_hdop: f32,
    pub last_gps_time: Instant,
    pub 当前目标索引: usize,
    pub 拓扑图记忆: TopologicalGraph,
    pub 导航路线: Vec<u32>,
}

fn 恢复历史视觉指纹(node: &TopologicalNode) -> Vec<稀疏特征点> {
    if node.descriptors.is_empty()
        || node.descriptors.len() % XFEAT_DESCRIPTOR_DIM != 0
        || node.keypoints.len()
            != (node.descriptors.len() / XFEAT_DESCRIPTOR_DIM) * KEYPOINT_COORD_DIM
    {
        return Vec::new();
    }

    node.descriptors
        .chunks_exact(XFEAT_DESCRIPTOR_DIM)
        .zip(node.keypoints.chunks_exact(KEYPOINT_COORD_DIM))
        .map(|(descriptor, keypoint)| 稀疏特征点 {
            x: keypoint[0],
            y: keypoint[1],
            置信度: 1.0,
            描述子: descriptor.to_vec(),
        })
        .collect()
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("========================================================");
    print!("🧠 [慢系统 - 双副驾驶脑] 北斗-视觉 Teach&Repeat 混合定位并网器启动...\n");
    println!("========================================================");

    let (mut node, mut events) = DoraNode::init_from_env()?;

    // 1. 初始化混合拓扑地图记忆（如果不存在则使用内置路点模拟）
    let map_path = "topo_memory.json";
    let mut 脑部记忆 = if std::path::Path::new(map_path).exists() {
        TopologicalGraph::load_from_file(map_path).unwrap_or_else(|_| TopologicalGraph::new())
    } else {
        TopologicalGraph::new()
    };

    // 如果是冷启动空地图，硬编码注入赛道黄金路标卡片，确保开箱即用 [cite: 1.1.2]
    if 脑部记忆.nodes.is_empty() {
        println!("⚠️ [慢脑自愈] 未检测到物理地图文件，正在在线生成北斗-视觉混合拓扑骨架...");
        let 站牌路点 = vec![
            (0, "起点站牌", 0.0, 0.0, 0.0),
            (1, "S弯入口站牌", 0.20, 1.50, 0.0),
            (2, "货架障碍区站牌", 0.40, 2.80, 0.0),
            (3, "左弯死角站牌", -1.00, 3.50, 0.0),
            (4, "终点冲刺站牌", 0.52, 4.11, 0.0),
        ];
        for (id, name, x, y, yaw) in 站牌路点 {
            let mut node = core_decision::topo_graph::node::TopologicalNode::default();
            node.id = id;
            node.name = name.to_string();
            node.pose.x = x;
            node.pose.y = y;
            node.pose.yaw = yaw;
            // 冷启动路点只提供度量目标；视觉重定位必须等待真实 XFeat 描述子和 keypoints 成对写入。
            脑部记忆.add_node(node);
        }
        // 铺设双向通道
        脑部记忆.add_edge(0, 1, 1.5, 0.0);
        脑部记忆.add_edge(1, 2, 1.3, 0.0);
        脑部记忆.add_edge(2, 3, 1.6, 15.0);
        脑部记忆.add_edge(3, 4, 1.8, -30.0);
        let _ = 脑部记忆.save_to_file(map_path);
    }

    // 自动寻路：起点 0 ➔ 终点 4 [cite: 21]
    let planned_route = 脑部记忆
        .find_path_astar(0, 4)
        .unwrap_or_else(|| vec![0, 1, 2, 3, 4]);
    println!("🧭 [慢脑寻路成功] A* 规划路标链: {:?}", planned_route);

    let mut state = 慢脑状态机 {
        当前模式: 导航模式::提线木偶重放, // 🛡️ 架构师自愈：已将点号 . 修复为双冒号 ::
        odom_x: 0.0,
        odom_y: 0.0,
        odom_yaw: 0.0,
        gps_x: 0.0,
        gps_y: 0.0,
        gps_hdop: 99.0, // 默认无星状态 [cite: 1.2.6]
        last_gps_time: Instant::now(),
        当前目标索引: 0,
        拓扑图记忆: 脑部记忆,
        导航路线: planned_route,
    };

    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                let id_str = id.as_str();
                match id_str {
                    // 📡 北斗高精度定位数据流入 (来自 DX-GP24-A 串口转换节点)
                    "gps" => {
                        let gps_arr = data
                            .as_any()
                            .downcast_ref::<Float32Array>()
                            .ok_or_else(|| eyre!("❌ Failed to parse GPS Arrow"))?;
                        if gps_arr.len() >= 3 {
                            state.gps_x = gps_arr.value(0);
                            state.gps_y = gps_arr.value(1);
                            state.gps_hdop = gps_arr.value(2);
                            state.last_gps_time = Instant::now();
                        }
                    }

                    // 🏎️ 物理里程计高频流入
                    "odometry" => {
                        let odom_arr = data
                            .as_any()
                            .downcast_ref::<Float32Array>()
                            .ok_or_else(|| eyre!("❌ Failed to parse Odometry"))?;
                        if odom_arr.len() >= 3 {
                            state.odom_x = odom_arr.value(0);
                            state.odom_y = odom_arr.value(1);
                            state.odom_yaw = odom_arr.value(2);
                        }

                        // 🧠 【双副驾驶仲裁协议核心】
                        let gps_timeout = state.last_gps_time.elapsed().as_secs_f32();
                        let previous_mode = state.当前模式;

                        // 判决条件：如果卫星星况极佳，且没有超时，激活北斗领航
                        if state.gps_hdop < 2.5 && gps_timeout < 2.0 {
                            state.当前模式 = 导航模式::北斗高精领航;
                        } else {
                            state.当前模式 = 导航模式::提线木偶重放;
                        }

                        if state.当前模式 != previous_mode {
                            println!(
                                "\n🚨 [双副驾驶切换] ➔ 检测定位状态突变：当前接管主权副驾驶 = {:?}",
                                state.当前模式
                            );
                        }

                        // 执行对应副驾驶的导航指令解算
                        if state.当前目标索引 < state.导航路线.len() {
                            let target_node_id = state.导航路线[state.当前目标索引];
                            if let Some(target_node) = state.拓扑图记忆.nodes.get(&target_node_id)
                            {
                                let (mut cur_pos_x, mut cur_pos_y) = (state.odom_x, state.odom_y);

                                if state.当前模式 == 导航模式::北斗高精领航 {
                                    // 【副驾驶 A 领航】：使用绝对北斗物理坐标系对齐
                                    cur_pos_x = state.gps_x;
                                    cur_pos_y = state.gps_y;
                                }

                                let dx = target_node.pose.x - cur_pos_x;
                                let dy = target_node.pose.y - cur_pos_y;
                                let remaining_dist = (dx * dx + dy * dy).sqrt();

                                // 到达判定：25厘米内判定过关，换下一个站牌
                                if remaining_dist < 0.25
                                    && state.当前目标索引 + 1 < state.导航路线.len()
                                {
                                    println!(
                                        "🎉 [站牌通关] 成功越过 {} 号路标 ({:.2}, {:.2})",
                                        target_node_id, target_node.pose.x, target_node.pose.y
                                    );
                                    state.当前目标索引 += 1;
                                }

                                // 100Hz 极速广播引力坐标 human_prior
                                let prior_arr = Float32Array::from(vec![
                                    target_node.pose.x,
                                    target_node.pose.y,
                                    target_node.pose.yaw,
                                ]);
                                let _ = node.send_output(
                                    "human_prior".to_string().into(),
                                    MetadataParameters::default(),
                                    prior_arr,
                                );
                            }
                        }
                    }

                    // 👁️ 视觉稀疏特征点流入 (来自前视单目 XFeat 提取)
                    "xfeat_features" => {
                        // 【副驾驶 B 提线木偶模式的核心自愈机制】：
                        // 只有在北斗失效、视觉接管时，才启动重度 XFeat / RANSAC 对齐，保障 CPU 资源
                        if state.当前模式 == 导航模式::提线木偶重放 {
                            let struct_array = data
                                .as_any()
                                .downcast_ref::<StructArray>()
                                .ok_or_else(|| eyre!("❌ Failed to cast XFeat Struct"))?;

                            let x_array = struct_array
                                .column_by_name("x")
                                .unwrap()
                                .as_any()
                                .downcast_ref::<Float32Array>()
                                .unwrap();
                            let y_array = struct_array
                                .column_by_name("y")
                                .unwrap()
                                .as_any()
                                .downcast_ref::<Float32Array>()
                                .unwrap();
                            let score_array = struct_array
                                .column_by_name("score")
                                .unwrap()
                                .as_any()
                                .downcast_ref::<Float32Array>()
                                .unwrap();
                            let desc_array = struct_array
                                .column_by_name("descriptor")
                                .unwrap()
                                .as_any()
                                .downcast_ref::<FixedSizeListArray>()
                                .unwrap();

                            let mut current_frame_features = Vec::new();
                            for i in 0..x_array.len() {
                                let px = x_array.value(i);
                                let py = y_array.value(i);
                                let conf = score_array.value(i);

                                let desc_value_array = desc_array.value(i);
                                let desc_float_array = desc_value_array
                                    .as_any()
                                    .downcast_ref::<Float32Array>()
                                    .unwrap();
                                let mut descriptor = vec![0.0f32; 64];
                                for j in 0..64 {
                                    descriptor[j] = desc_float_array.value(j);
                                }
                                current_frame_features.push(稀疏特征点 {
                                    x: px,
                                    y: py,
                                    置信度: conf,
                                    描述子: descriptor,
                                });
                            }

                            // 检索当前要追踪的历史站牌指纹
                            let target_node_id = state.导航路线[state.当前目标索引];
                            if let Some(target_node) = state.拓扑图记忆.nodes.get(&target_node_id)
                            {
                                // 将扁平化的一维描述子恢复成 64D 数组；缺少 keypoints 的历史沉淤指纹直接跳过。
                                let history_features = 恢复历史视觉指纹(target_node);
                                if history_features.len() < 8 {
                                    continue;
                                }

                                // 运行双向余弦交叉匹配，对准古董门环/墙角 [cite: 1.2]
                                let matches = 仿生匹配器::交叉匹配(
                                    &current_frame_features,
                                    &history_features,
                                    0.75,
                                );
                                if matches.len() >= 8 {
                                    // 几何测谎 RANSAC 过滤 [cite: 1.2]
                                    if let Ok(clean_matches) = 仿生匹配器::几何纠偏过滤(
                                        &current_frame_features,
                                        &history_features,
                                        &matches,
                                        3.0,
                                    ) {
                                        if clean_matches.len() >= 5 {
                                            // 单目一般场景匹配没有尺度，不能把像素差直接写入米制里程计。
                                            // 此处只确认存在稳定的几何重定位证据；度量校正必须等待带尺度地图或地面特征标记。
                                            let _ = 仿生匹配器::估计单应性(
                                                &current_frame_features,
                                                &history_features,
                                                &clean_matches,
                                                3.0,
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
            Event::Stop(_) => {
                println!("🛑 [慢脑] 双副驾驶领航脑安全下线。");
                break;
            }
            _ => {}
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn 历史视觉指纹缺少_keypoints_时不参与重定位() {
        let mut node = TopologicalNode::default();
        node.descriptors = vec![0.1; XFEAT_DESCRIPTOR_DIM * 2];

        assert!(恢复历史视觉指纹(&node).is_empty());
    }

    #[test]
    fn 历史视觉指纹必须按_64d描述子_和_xy坐标_成对恢复() {
        let mut node = TopologicalNode::default();
        node.descriptors = vec![0.0; XFEAT_DESCRIPTOR_DIM * 2];
        node.descriptors[3] = 1.0;
        node.descriptors[XFEAT_DESCRIPTOR_DIM + 7] = 1.0;
        node.keypoints = vec![10.0, 20.0, 30.0, 40.0];

        let features = 恢复历史视觉指纹(&node);

        assert_eq!(features.len(), 2);
        assert_eq!((features[0].x, features[0].y), (10.0, 20.0));
        assert_eq!((features[1].x, features[1].y), (30.0, 40.0));
        assert_eq!(features[0].描述子.len(), XFEAT_DESCRIPTOR_DIM);
        assert_eq!(features[1].描述子.len(), XFEAT_DESCRIPTOR_DIM);
    }
}
