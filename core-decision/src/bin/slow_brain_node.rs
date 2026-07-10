// =======
// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
/*
=================================================================
🧠 [NEXUS 慢系统] 无图自引力领航脑 (Mapless PointGoal-Nav Goal Provider)
设计哲学: 完全抛弃示教线与地图依赖 | 100Hz 极速生命喂狗 | 定位绝对引力牵引
=================================================================
*/
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("========================================================");
    println!("🧠 [慢脑] 无图自引力领航脑 (PointGoal-Nav Provider) 已启动");
    println!("设计哲学: 0% 图像图像开销 | 100Hz 极速生命喂狗 | 定位绝对引力牵引");
    println!("========================================================");

    // 1. 接入 DORA 数据流分布式网络
    let (mut node, mut events) = DoraNode::init_from_env()?;

    // 2. 动态物理自愈：优先从环境变量读取终点绝对坐标，默认自适应 fallback 为你刚才设置的赛道终点附近 (0.52, 4.11)
    let goal_x = std::env::var("FSD_GOAL_X")
        .unwrap_or_default()
        .parse::<f32>()
        .unwrap_or(0.52f32);
    let goal_y = std::env::var("FSD_GOAL_Y")
        .unwrap_or_default()
        .parse::<f32>()
        .unwrap_or(4.11f32);
    let goal_yaw = std::env::var("FSD_GOAL_YAW")
        .unwrap_or_default()
        .parse::<f32>()
        .unwrap_or(0.0f32);

    println!("🧭 [慢脑] 成功载入自引力导航终点坐标 (World Frame):");
    println!("   -> 物理坐标 X_goal   : {:.4} m", goal_x);
    println!("   -> 物理坐标 Y_goal   : {:.4} m", goal_y);
    println!("   -> 目标航向 Yaw_goal : {:.4} rad", goal_yaw);
    println!("--------------------------------------------------------");

    let mut loop_count = 0u64;

    // 3. 进入极速 100Hz 领航事件循环
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                // 订阅 100Hz 物理里程计高频绝对位置
                if id.as_str() == "odometry" {
                    let odom_arr = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("❌ 无法解析 odometry 数据"))?;
                    
                    if odom_arr.len() >= 3 {
                        let curr_x = odom_arr.value(0);
                        let curr_y = odom_arr.value(1);
                        
                        // 📐 实数精算：求取自车当前绝对坐标与终点之间的物理直线距离
                        let dx = goal_x - curr_x;
                        let dy = goal_y - curr_y;
                        let remaining_dist = (dx * dx + dy * dy).sqrt();

                        loop_count += 1;
                        // 每 100 帧（1秒）向控制台输出一次高保真自引力寻迹监测
                        if loop_count % 100 == 0 {
                            let status_text = if remaining_dist < 0.15 {
                                "🏆 已成功抵达终点！小车进入驻车静默..."
                            } else {
                                "🧭 正在执行无图自引力领航..."
                            };
                            println!(
                                "[慢脑 100Hz] 当前位置: ({:>5.2}, {:>5.2}) | 目标: ({:.2}, {:.2}) | 剩余距离: {:>5.2}m | 状态: {}",
                                curr_x, curr_y, goal_x, goal_y, remaining_dist, status_text
                            );
                        }

                        // 🎯 核心动作：高频（100Hz）将绝对终点坐标广播至 DORA 总线！
                        // 彻底平息 5 秒超时！让快脑的“生命看门狗”每 10ms 都能被美味的新鲜目标数据喂饱！
                        let prior_arr = dora_node_api::arrow::array::Float32Array::from(vec![
                            goal_x, goal_y, goal_yaw
                        ]);
                        
                        // 🎯 3.3.2 极速对齐：直接使用头部导入的 MetadataParameters，净化代码，消除警告
                        let _ = node.send_output(
                            "human_prior".to_string().into(),
                            MetadataParameters::default(),
                            prior_arr,
                        );
                    }
                }
            }
            Event::Stop(_) => {
                println!("🛑 [慢脑] 收到 DORA 停止信号，自引力领航脑安全下线。开启写保护。");
                break;
            }
            _ => {}
        }
    }
    Ok(())
}