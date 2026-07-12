#![allow(non_snake_case)]
use core_control::预测控制求解器;
use core_control::solver::凸包走廊;
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use std::time::Instant;
use std::io::Write;

fn local_to_bev(x: f64, y: f64) -> (i32, i32) {
    let res = 0.03125;
    let col = 96 - (y / res) as i32;
    let row = 191 - (x / res) as i32;
    (row, col)
}

fn 生成BEV自适应凸包走廊(
    _start_x: f64, _start_y: f64,
    target_x: f64, target_y: f64,
    bev_grid: &[u8]
) -> (凸包走廊, f64, f64) {
    let dx = target_x;
    let dy = target_y;
    let dist = (dx * dx + dy * dy).sqrt().max(0.01);
    let dir_x = dx / dist;
    let dir_y = dy / dist;
    let norm_x = -dir_y;
    let norm_y = dir_x;
    let length_margin = 0.5;
    let mut min_left_dist = 0.6;
    let mut min_right_dist = 0.6;
    let steps = 15;
    for i in 1..=steps {
        let t = i as f64 / steps as f64;
        let px = t * target_x;
        let py = t * target_y;
        for d_step in 1..=12 {
            let d = d_step as f64 * 0.05;
            let lx = px + d * norm_x;
            let ly = py + d * norm_y;
            let (row, col) = local_to_bev(lx, ly);
            let mut hit = false;
            for r_offset in -3..=3 {
                let r_check = row + r_offset;
                if r_check >= 0 && r_check < 192 && col >= 0 && col < 192 {
                    if bev_grid[(r_check * 192 + col) as usize] == 255 {
                        hit = true;
                        break;
                    }
                }
            }
            if hit {
                if d < min_left_dist {
                    min_left_dist = d;
                }
                break;
            }
        }
        for d_step in 1..=12 {
            let d = d_step as f64 * 0.05;
            let rx = px - d * norm_x;
            let ry = py - d * norm_y;
            let (row, col) = local_to_bev(rx, ry);
            let mut hit = false;
            for r_offset in -3..=3 {
                let r_check = row + r_offset;
                if r_check >= 0 && r_check < 192 && col >= 0 && col < 192 {
                    if bev_grid[(r_check * 192 + col) as usize] == 255 {
                        hit = true;
                        break;
                    }
                }
            }
            if hit {
                if d < min_right_dist {
                    min_right_dist = d;
                }
                break;
            }
        }
    }
    let half_width_left = min_left_dist.max(0.20);
    let half_width_right = min_right_dist.max(0.20);
    let mut a_mat = [[0.0; 2]; 4];
    let mut b_vec = [0.0; 4];
    a_mat[0] = [norm_x, norm_y];
    b_vec[0] = norm_x * (half_width_left * norm_x) + norm_y * (half_width_left * norm_y);
    a_mat[1] = [-norm_x, -norm_y];
    b_vec[1] = -norm_x * (-half_width_right * norm_x) - norm_y * (-half_width_right * norm_y);
    a_mat[2] = [dir_x, dir_y];
    b_vec[2] = dir_x * (target_x + dir_x * length_margin) + dir_y * (target_y + dir_y * length_margin);
    a_mat[3] = [-dir_x, -dir_y];
    b_vec[3] = -dir_x * (-dir_x * length_margin) - dir_y * (-dir_y * length_margin);
    (凸包走廊 { a_mat, b_vec }, half_width_left, half_width_right)
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut is_initialized = false;
    let mut current_velocity = 0.0f64;
    let mut current_x = 0.0f64;
    let mut current_y = 0.0f64;
    let mut current_yaw = 0.0f64;
    let mut target_x = 0.0f64;
    let mut target_y = 0.0f64;
    let mut latest_bev_grid = vec![0u8; 192 * 192];
    let mut current_ttc = 10.0f64;
    let mut last_human_prior_time = Instant::now();
    let mut brain = 预测控制求解器::new().expect("NMPC Init Failed");
    let mut tick_count: u64 = 0;
    
    // 🛡️ 架构师自愈：将详细高维遥测 CSV 写入 /tmp，保持主根目录绝对纯净
    let mut log_file = std::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open("/tmp/fsd_nmpc_telemetry.csv")
        .expect("Failed to create NMPC telemetry log");
        
    let _ = writeln!(
        log_file, 
        "tick,x,y,yaw,g_lx,g_ly,g_dist,min_l,min_r,solver_status,ttc,brake,v_raw,w_cmd,v_final"
    );

    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                let id_str = id.as_str();
                if id_str == "ttc" {
                    let ttc_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map ttc"))?;
                    if ttc_array.len() >= 1 {
                        current_ttc = ttc_array.value(0) as f64;
                    }
                }
                else if id_str == "bev_grid" {
                    let grid_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::UInt8Array>()
                        .ok_or_else(|| eyre!("Failed to map bev_grid"))?;
                    if grid_array.len() == 192 * 192 {
                        for idx in 0..(192 * 192) {
                            latest_bev_grid[idx] = grid_array.value(idx);
                        }
                    }
                }
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
                        eprintln!("Status injection failed: {}", e);
                        continue;
                    }
                    let dx = target_x - current_x;
                    let dy = target_y - current_y;
                    let local_target_x = dx * current_yaw.cos() + dy * current_yaw.sin();
                    let local_target_y = -dx * current_yaw.sin() + dy * current_yaw.cos();
                    let target_distance = (local_target_x * local_target_x + local_target_y * local_target_y).sqrt();
                    let target_velocity = if target_distance < 0.2 { 0.0 } else { 0.80f64.min(target_distance * 0.5) };
                    let d_ff = local_target_x / 3.0;
                    for k in 0..=20 {
                        let t = (k as f64) / 20.0;
                        let ref_x = 3.0 * (1.0 - t).powi(2) * t * d_ff + 3.0 * (1.0 - t) * t.powi(2) * (local_target_x - d_ff * local_target_y.atan2(local_target_x).cos()) + t.powi(3) * local_target_x;
                        let ref_y = 3.0 * (1.0 - t) * t.powi(2) * (local_target_y - d_ff * local_target_y.atan2(local_target_x).sin()) + t.powi(3) * local_target_y;
                        let ref_yaw = local_target_y.atan2(local_target_x) * (t * t * (3.0 - 2.0 * t)); 
                        let _ = brain.设置参考轨迹点(k, ref_x, ref_y, ref_yaw, target_velocity);
                    }
                    
                    let (corridor, min_l, min_r) = 生成BEV自适应凸包走廊(0.0, 0.0, local_target_x, local_target_y, &latest_bev_grid);
                    if let Err(e) = brain.设置安全走廊硬约束(&corridor) {
                        eprintln!("Convex corridor injection failed: {}", e);
                    }
                    
                    // 🛡️ 对齐接口：解包获取 C 求解器真实的运行状态码 (solver_status)
                    let (v_cmd, w_cmd, solver_status) = match brain.求解最优控制量(current_velocity) {
                        Ok((v, w, stat)) => (v, w, stat),
                        Err(_) => (0.0, 0.0, -1)
                    };
                    
                    let tau_safe = 0.8f64;
                    let k_sigmoid = 8.0f64;
                    let brake_factor = 1.0 / (1.0 + (-k_sigmoid * (current_ttc - tau_safe)).exp());
                    let final_v_cmd = v_cmd * brake_factor;
                    current_velocity = final_v_cmd;
                    
                    // 100Hz 高频无损落盘至 /tmp
                    let _ = writeln!(
                        log_file, 
                        "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{:.4},{:.4},{:.4},{:.4},{:.4}",
                        tick_count, current_x, current_y, current_yaw, 
                        local_target_x, local_target_y, target_distance,
                        min_l, min_r, solver_status, current_ttc, brake_factor,
                        v_cmd, w_cmd, final_v_cmd
                    );
                    
                    // 100Hz 原地静默，不刷屏，保留纯净后台
                    if tick_count % 100 == 0 {
                        print!("[NMPC CORE 100Hz] Tracking Goal Distance: {:.2}m | Solver Status: {}\r", target_distance, solver_status);
                        std::io::stdout().flush().unwrap();
                    }
                    
                    let cmd_arrow = dora_node_api::arrow::array::Float32Array::from(vec![final_v_cmd as f32, w_cmd as f32]);
                    let _ = node.send_output("control_cmd".to_string().into(), MetadataParameters::default(), cmd_arrow);
                }
            }
            Event::Stop(_) => {
                println!("\n🛑 Received DORA stop signal, exiting controller.");
                break;
            }
            _ => {}
        }
    }
    Ok(())
}
