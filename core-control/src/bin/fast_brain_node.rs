use core_control::预测控制求解器;
use core_control::solver::凸包走廊;
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use std::time::Instant;
use std::io::Write;

fn 生成局部凸包走廊(当前x: f64, 当前y: f64, 目标x: f64, 目标y: f64) -> 凸包走廊 {
    let dx = 目标x - 当前x;
    let dy = 目标y - 当前y;
    let dist = (dx * dx + dy * dy).sqrt().max(0.01);
    
    let dir_x = dx / dist;
    let dir_y = dy / dist;
    let norm_x = -dir_y;
    let norm_y = dir_x;

    let half_width = 0.6; 
    let length_margin = 0.5; 

    let mut a_mat = [[0.0; 2]; 4];
    let mut b_vec = [0.0; 4];

    a_mat[0] = [norm_x, norm_y];
    b_vec[0] = norm_x * (当前x + half_width * norm_x) + norm_y * (当前y + half_width * norm_y);
    a_mat[1] = [-norm_x, -norm_y];
    b_vec[1] = -norm_x * (当前x - half_width * norm_x) - norm_y * (当前y - half_width * norm_y);
    a_mat[2] = [dir_x, dir_y];
    b_vec[2] = dir_x * (目标x + dir_x * length_margin) + dir_y * (目标y + dir_y * length_margin);
    a_mat[3] = [-dir_x, -dir_y];
    b_vec[3] = -dir_x * (当前x - dir_x * length_margin) - dir_y * (当前y - dir_y * length_margin);

    凸包走廊 { a_mat, b_vec }
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("========================================================");
    println!("🚀 NEXUS SOTA 2026 - 规控一元化快脑 (Convex Corridor NMPC) 已启动");
    println!("设计哲学: 斥力几何形变 | 纯凸空间硬约束 | 光流 TTC 物理抱闸");
    println!("========================================================");

    let (mut node, mut events) = DoraNode::init_from_env()?;
    
    let mut is_initialized = false;
    let mut current_velocity = 0.0f64;
    let mut current_x = 0.0f64;
    let mut current_y = 0.0f64;
    let mut current_yaw = 0.0f64;
    
    let mut target_x = 0.0f64;
    let mut target_y = 0.0f64;
    
    let mut filtered_force_y = 0.0f64; 
    
    // 🛡️ 新增：碰撞时间 (Time-to-Collision)，默认 10.0 秒 (绝对安全)
    let mut current_ttc = 10.0f64;
    
    let mut last_human_prior_time = Instant::now();
    let mut brain = 预测控制求解器::new().expect("NMPC Init Failed");
    let mut tick_count: u64 = 0;

    let mut log_file = std::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open("nmpc_telemetry.csv")
        .expect("Failed to create NMPC telemetry log");

    let _ = writeln!(log_file, "tick,cur_x,cur_y,cur_yaw,target_x,target_y,force_y,ttc,v_cmd,w_cmd,final_v");

    println!("🟢 规控一元化事件循环已就绪，等待 DORA 物理时钟同步...");

    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                let id_str = id.as_str();
                
                // 1. 接收光流节点的 TTC 碰撞时间
                if id_str == "ttc" {
                    let ttc_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map ttc"))?;
                    if ttc_array.len() >= 1 {
                        current_ttc = ttc_array.value(0) as f64;
                    }
                }
                // 2. 接收青蛙眼斥力，用于几何形变
                else if id_str == "obstacle_force" {
                    let force_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map obstacle force"))?;
                    if force_array.len() >= 2 {
                        let raw_force_y = force_array.value(1) as f64;
                        filtered_force_y += 0.15 * (raw_force_y - filtered_force_y);
                    }
                }
                // 3. 接收慢脑的绝对引力目标点
                else if id_str == "human_prior" {
                    let prior_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map human prior"))?;
                    if prior_array.len() >= 3 {
                        target_x = prior_array.value(0) as f64;
                        target_y = prior_array.value(1) as f64;
                        
                        if !is_initialized {
                            is_initialized = true;
                            println!("🔓 慢脑引力目标已锁定，NMPC 求解器解除封印！");
                        }
                        last_human_prior_time = Instant::now();
                    }
                } 
                // 4. 接收 100Hz 物理里程计，驱动 NMPC 闭环
                else if id_str == "odometry" {
                    let odom_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map odometry"))?;
                    if odom_array.len() >= 3 {
                        current_x = odom_array.value(0) as f64;
                        current_y = odom_array.value(1) as f64;
                        current_yaw = odom_array.value(2) as f64;
                    }

                    if !is_initialized { continue; }
                    tick_count += 1;

                    if last_human_prior_time.elapsed() > std::time::Duration::from_millis(5000) {
                        let zero_cmd = dora_node_api::arrow::array::Float32Array::from(vec![0.0f32, 0.0f32]);
                        let _ = node.send_output("control_cmd".to_string().into(), MetadataParameters::default(), zero_cmd);
                        continue;
                    }

                    if let Err(e) = brain.设置当前状态(0.0, 0.0, 0.0, current_velocity) {
                        eprintln!("状态注入失败: {}", e);
                        continue;
                    }

                    let dx = target_x - current_x;
                    let dy = target_y - current_y;
                    let local_target_x = dx * current_yaw.cos() + dy * current_yaw.sin();
                    let local_target_y = -dx * current_yaw.sin() + dy * current_yaw.cos();
                    
                    let corridor_shift_gain = 2.5; 
                    let shifted_target_y = local_target_y + filtered_force_y * corridor_shift_gain;

                    let target_distance = (local_target_x * local_target_x + shifted_target_y * shifted_target_y).sqrt();
                    let shifted_target_yaw = shifted_target_y.atan2(local_target_x).clamp(-0.35, 0.35);

                    let target_velocity = if target_distance < 0.2 { 0.0 } else { 0.80f64.min(target_distance * 0.5) };

                    let d_ff = local_target_x / 3.0;
                    for k in 0..=20 {
                        let t = (k as f64) / 20.0;
                        let ref_x = 3.0 * (1.0 - t).powi(2) * t * d_ff + 3.0 * (1.0 - t) * t.powi(2) * (local_target_x - d_ff * shifted_target_yaw.cos()) + t.powi(3) * local_target_x;
                        let ref_y = 3.0 * (1.0 - t) * t.powi(2) * (shifted_target_y - d_ff * shifted_target_yaw.sin()) + t.powi(3) * shifted_target_y;
                        let ref_yaw = shifted_target_yaw * (t * t * (3.0 - 2.0 * t)); 
                        
                        let _ = brain.设置参考轨迹点(k, ref_x, ref_y, ref_yaw, target_velocity);
                    }

                    let corridor = 生成局部凸包走廊(0.0, 0.0, local_target_x, shifted_target_y);
                    if let Err(e) = brain.设置安全走廊硬约束(&corridor) {
                        eprintln!("凸包走廊注入失败: {}", e);
                    }

                    let (v_cmd, w_cmd) = match brain.求解最优控制量(current_velocity) {
                        Ok((v, w)) => (v, w),
                        Err(_) => (0.0, 0.0)
                    };
                    
                    // 🌟 核心重构：Sigmoid 物理抱闸反射弧
                    let tau_safe = 0.8f64; // 安全临界时间 0.8 秒
                    let k_sigmoid = 8.0f64; // 阻尼陡峭程度
                    // Sigmoid 阻尼计算：当 TTC < 0.8 时，brake_factor 迅速衰减至 0
                    let brake_factor = 1.0 / (1.0 + (-k_sigmoid * (current_ttc - tau_safe)).exp());
                    
                    // 强制物理截断
                    let final_v_cmd = v_cmd * brake_factor;
                    current_velocity = final_v_cmd;

                    let _ = writeln!(log_file, "{}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}",
                        tick_count, current_x, current_y, current_yaw, target_x, target_y, filtered_force_y, current_ttc, v_cmd, w_cmd, final_v_cmd);

                    if tick_count % 100 == 0 {
                        let brake_status = if brake_factor < 0.5 { "🔴 紧急抱闸" } else { "🟢 正常" };
                        print!("[NMPC 规控] Tick: {:<5} | TTC: {:>4.1}s [{}] | 原始v: {:.2} | 最终v: {:.2} m/s\r",
                            tick_count, current_ttc, brake_status, v_cmd, final_v_cmd);
                        std::io::stdout().flush().unwrap();
                    }

                    let cmd_arrow = dora_node_api::arrow::array::Float32Array::from(vec![final_v_cmd as f32, w_cmd as f32]);
                    let _ = node.send_output("control_cmd".to_string().into(), MetadataParameters::default(), cmd_arrow);
                }
            }
            Event::Stop(_) => {
                println!("\n🛑 收到 DORA 停止信号，规控大脑安全下线。");
                break;
            }
            _ => {}
        }
    }
    Ok(())
}
