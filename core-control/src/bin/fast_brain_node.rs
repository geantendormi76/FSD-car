use core_control::预测控制求解器;
use core_control::solver::动态障碍物;
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use std::sync::Arc;
use std::sync::RwLock;
use std::time::Duration;

struct ExecutionContext {
    pub is_initialized: bool,
    pub desired_force_x: f64,
    pub desired_force_y: f64,
    pub current_velocity: f64,
    pub current_x: f64,
    pub current_y: f64,
    pub current_yaw: f64,
    pub target_x: f64,
    pub target_y: f64,
    pub target_yaw: f64,
    pub last_update_time: std::time::Instant,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("NEXUS Fast Brain planning node with Spline Generator has started...");
    let (mut node, mut events) = DoraNode::init_from_env()?;
    
    let context = Arc::new(RwLock::new(ExecutionContext {
        is_initialized: false,
        desired_force_x: 0.0,
        desired_force_y: 0.0,
        current_velocity: 0.0,
        current_x: 0.0,
        current_y: 0.0,
        current_yaw: 0.0,
        target_x: 0.0,
        target_y: 0.0,
        target_yaw: 0.0,
        last_update_time: std::time::Instant::now(),
    }));
    
    let ctrl_context = context.clone();
    
    let control_handle = tokio::spawn(async move {
        let mut brain = 预测控制求解器::new().expect("Failed to initialize NMPC solver");
        let mut solver_ready = false;
        let mut tick_count: u64 = 0;
        
        let mut filtered_force_x = 0.0f64;
        let mut filtered_force_y = 0.0f64;
        let mut filtered_target_x = 0.0f64;
        let mut filtered_target_y = 0.0f64;
        let mut filtered_target_yaw = 0.0f64;
        
        let mut filtered_obs_x = 1000.0f64;
        let mut filtered_obs_y = 1000.0f64;
        let mut last_omega_w = 0.0f64;
        
        use std::io::Write as _;
        let mut log_file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("nmpc_telemetry.csv")
            .expect("Failed to create NMPC telemetry log");
            
        let _ = writeln!(
            log_file,
            "tick,cur_x,cur_y,cur_yaw,target_x,target_y,target_yaw,force_x,force_y,v_cmd,w_cmd,cur_v"
        );
        
        let mut ticker = tokio::time::interval(Duration::from_millis(10));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        
        loop {
            ticker.tick().await;
            tick_count += 1;
            
            let (initialized, force_x, force_y, cur_v, last_time) = {
                let lock = ctrl_context.read().unwrap();
                (lock.is_initialized, lock.desired_force_x, lock.desired_force_y, lock.current_velocity, lock.last_update_time)
            };
            
            if !initialized {
                continue;
            }
            
            let watchdog_timeout = last_time.elapsed();
            if watchdog_timeout > std::time::Duration::from_millis(5000) {
                eprintln!("Failsafe Watchdog triggered: Timeout {:.2?}s! Stop vehicle.", watchdog_timeout.as_secs_f64());
                let zero_cmd = dora_node_api::arrow::array::Float32Array::from(vec![0.0f32, 0.0f32]);
                let _ = node.send_output(
                    "control_cmd".to_string().into(),
                    MetadataParameters::default(),
                    zero_cmd,
                );
                continue;
            }
            
            if !solver_ready {
                println!("NMPC Solver Warm-start completed. Control mapped.");
                solver_ready = true;
            }
            
            if let Err(e) = brain.设置当前状态(0.0, 0.0, 0.0, cur_v) {
                eprintln!("Failed to anchor local coordinates: {}", e);
                continue;
            }
            
            let (cur_x, cur_y, cur_yaw) = {
                let lock = ctrl_context.read().unwrap();
                (lock.current_x, lock.current_y, lock.current_yaw)
            };
            
            let (tgt_x, tgt_y, tgt_yaw) = {
                let lock = ctrl_context.read().unwrap();
                (lock.target_x, lock.target_y, lock.target_yaw)
            };
            
            let (active_force_x, active_force_y) = if cur_v.abs() < 0.04 {
                (0.0f64, 0.0f64)
            } else {
                (force_x, force_y)
            };
            
            let cross_coupling_decel = -(active_force_y.abs() * 0.45);
            let combined_force_x = active_force_x + cross_coupling_decel;
            
            filtered_force_x += 0.125f64 * (combined_force_x - filtered_force_x);
            filtered_force_y += 0.125f64 * (active_force_y - filtered_force_y);
            
            let mut suppressed_force_y = filtered_force_y;
            let abs_omega = last_omega_w.abs();
            if abs_omega > 0.12 {
                let suppression_factor = (-4.5 * (abs_omega - 0.12)).exp();
                suppressed_force_y *= suppression_factor.clamp(0.15, 1.0);
            }
            
            if tick_count == 1 || (tgt_x == 0.0 && tgt_y == 0.0) {
                filtered_target_x = tgt_x;
                filtered_target_y = tgt_y;
                filtered_target_yaw = tgt_yaw;
            } else {
                let spatial_filter_coeff = 0.15f64;
                filtered_target_x += spatial_filter_coeff * (tgt_x - filtered_target_x);
                filtered_target_y += spatial_filter_coeff * (tgt_y - filtered_target_y);
                filtered_target_yaw += spatial_filter_coeff * (tgt_yaw - filtered_target_yaw);
            }
            
            let dx = filtered_target_x - cur_x;
            let dy = filtered_target_y - cur_y;
            let local_target_x = dx * cur_yaw.cos() + dy * cur_yaw.sin();
            let local_target_y = -dx * cur_yaw.sin() + dy * cur_yaw.cos();
            let local_target_yaw = (filtered_target_yaw - cur_yaw).clamp(-0.25, 0.25);
            
            let target_distance = (local_target_x * local_target_x + local_target_y * local_target_y).sqrt();
            let target_limit = 1.2f64;
            let (scaled_x, scaled_y) = if target_distance > target_limit {
                let scale = target_limit / target_distance;
                (local_target_x * scale, local_target_y * scale)
            } else {
                (local_target_x, local_target_y)
            };
            
            let mut rebound_y = 0.0f64;
            if suppressed_force_y.abs() > 0.02 {
                rebound_y = suppressed_force_y.signum() * (suppressed_force_y.abs() * 0.75).min(0.35);
            }
            
            let target_velocity = (0.80 + filtered_force_x).clamp(0.0, 0.80);
            let mut injection_success = true;
            
            let d_ff = scaled_x / 3.0;
            
            for k in 0..=20 {
                let t = (k as f64) / 20.0;
                
                let (ref_x, ref_y) = if t == 0.0 {
                    (0.0, 0.0)
                } else {
                    let bx = 3.0 * (1.0 - t).powi(2) * t * d_ff 
                           + 3.0 * (1.0 - t) * t.powi(2) * (scaled_x - d_ff * local_target_yaw.cos()) 
                           + t.powi(3) * scaled_x;
                    let by = 3.0 * (1.0 - t) * t.powi(2) * (scaled_y - d_ff * local_target_yaw.sin()) 
                           + t.powi(3) * scaled_y;
                    (bx, by)
                };
                
                let spiced_ref_x = ref_x + (filtered_force_x * t);
                let spiced_ref_y = ref_y + (rebound_y * t);
                let ref_yaw = local_target_yaw * (t * t * (3.0 - 2.0 * t));
                
                if let Err(e) = brain.设置参考轨迹点(k, spiced_ref_x, spiced_ref_y, ref_yaw, target_velocity) {
                    eprintln!("Failed to inject spline reference at stage {}: {}", k, e);
                    injection_success = false;
                    break;
                }
            }
            
            if !injection_success {
                continue;
            }
            
            let (obs_x, obs_y, axis_a, axis_b) = if suppressed_force_y.abs() > 0.04 || filtered_force_x < -0.04 {
                let ox = 0.65;
                let oy = -suppressed_force_y.clamp(-0.35, 0.35);
                (ox, oy, 0.35, 0.25)
            } else {
                (1000.0, 1000.0, 0.1, 0.1)
            };
            
            let obstacle_damping = 0.125f64;
            if obs_x > 500.0 {
                filtered_obs_x = obs_x;
                filtered_obs_y = obs_y;
            } else {
                if filtered_obs_x > 500.0 {
                    filtered_obs_x = obs_x;
                    filtered_obs_y = obs_y;
                } else {
                    filtered_obs_x += obstacle_damping * (obs_x - filtered_obs_x);
                    filtered_obs_y += obstacle_damping * (obs_y - filtered_obs_y);
                }
            }
            
            let mut active_obstacles = Vec::new();
            if filtered_obs_x < 500.0 {
                active_obstacles.push(动态障碍物 {
                    x: filtered_obs_x,
                    y: filtered_obs_y,
                    a: axis_a,
                    b: axis_b,
                });
            }
            
            let _ = brain.设置动态障碍物硬约束(&active_obstacles);
            
            let (v_cmd, w_cmd) = match brain.求解最优控制量(cur_v) {
                Ok((v, w)) => (v, w),
                Err(e) => {
                    eprintln!("NMPC calculation error: {}", e);
                    (0.0, 0.0)
                }
            };
            
            {
                let mut lock = ctrl_context.write().unwrap();
                lock.current_velocity = v_cmd;
            }
            
            last_omega_w = w_cmd;
            
            let _ = writeln!(
                log_file,
                "{}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}",
                tick_count, cur_x, cur_y, cur_yaw, tgt_x, tgt_y, tgt_yaw, active_force_x, active_force_y, v_cmd, w_cmd, cur_v
            );
            
            if tick_count % 100 == 0 {
                let status_str = if filtered_obs_x < 100.0 { "DEFENSIVE" } else { "CLEAR" };
                println!(
                    "[NMPC 100Hz Telemetry] Tick: {:<5} | Status: {} | Target dist: {:.2}m | Force: (Fx:{:>6.3}, Fy:{:>6.3}) | v_cmd: {:.3} m/s | w_cmd: {:>6.3} rad/s",
                    tick_count, status_str, target_distance, force_x, force_y, v_cmd, w_cmd
                );
            }
            
            let cmd_arrow = dora_node_api::arrow::array::Float32Array::from(vec![
                v_cmd as f32,
                w_cmd as f32
            ]);
            
            if let Err(e) = node.send_output(
                "control_cmd".to_string().into(),
                MetadataParameters::default(),
                cmd_arrow,
            ) {
                eprintln!("Failed to send control command to DORA: {}", e);
            }
        }
    });
    
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() == "obstacle_force" {
                    let force_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map obstacle force to Float32Array"))?;
                    if force_array.len() >= 2 {
                        let f_x = force_array.value(0) as f64;
                        let f_y = force_array.value(1) as f64;
                        let mut lock = context.write().unwrap();
                        lock.desired_force_x = f_x;
                        lock.desired_force_y = f_y;
                    }
                }
                else if id.as_str() == "odometry" {
                    let odom_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map odometry to Float32Array"))?;
                    if odom_array.len() >= 3 {
                        let mut lock = context.write().unwrap();
                        lock.current_x = odom_array.value(0) as f64;
                        lock.current_y = odom_array.value(1) as f64;
                        lock.current_yaw = odom_array.value(2) as f64;
                    }
                }
                else if id.as_str() == "human_prior" {
                    let prior_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map human prior to Float32Array"))?;
                    if prior_array.len() >= 3 {
                        let mut lock = context.write().unwrap();
                        lock.target_x = prior_array.value(0) as f64;
                        lock.target_y = prior_array.value(1) as f64;
                        lock.target_yaw = prior_array.value(2) as f64;
                        if !lock.is_initialized {
                            lock.is_initialized = true;
                            println!("Warmstart gate unsealed: Slow brain homing signal linked!");
                        }
                        lock.last_update_time = std::time::Instant::now();
                    }
                }
            }
            Event::Stop(_) => {
                println!("DORA Stop event caught. Terminating fast brain...");
                break;
            }
            _ => {}
        }
    }
    
    control_handle.abort();
    Ok(())
}
