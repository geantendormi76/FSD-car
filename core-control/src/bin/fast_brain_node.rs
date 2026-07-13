#![allow(non_snake_case)]
use core_control::预测控制求解器;
use core_control::solver::动态障碍物;
use core_control::{车辆运动控制器, 神经事件路由器, 智能自驾信号载荷}; 
use dora_node_api::{DoraNode, MetadataParameters};
use std::sync::Arc;
use std::sync::RwLock;
use std::time::Duration;
use std::io::Write;
struct ExecutionContext {
    pub is_initialized: bool,
    pub current_velocity: f64,
    pub current_x: f64,
    pub current_y: f64,
    pub current_yaw: f64,
    pub target_x: f64,
    pub target_y: f64,
    pub target_yaw: f64,
    pub last_update_time: std::time::Instant,
    pub goal_reached: bool, 
    pub bev_grid: Vec<u8>,  
}
#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("NEXUS SOTA 2026 - Cleaned 100Hz NMPC Brain active...");
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let context = Arc::new(RwLock::new(ExecutionContext {
        is_initialized: false,
        current_velocity: 0.0,
        current_x: 0.0,
        current_y: 0.0,
        current_yaw: 0.0,
        target_x: 0.0,
        target_y: 0.0,
        target_yaw: 0.0,
        last_update_time: std::time::Instant::now(),
        goal_reached: false,
        bev_grid: vec![0u8; 36864], 
    }));
    let ctrl_context = context.clone();
    let control_handle = tokio::spawn(async move {
        let mut brain: Box<dyn 车辆运动控制器<状态=(f64,f64,f64,f64), 轨迹=(f64,f64,f64,f64), 障碍=动态障碍物, 指令=(f64,f64)> + Send> = 
            Box::new(预测控制求解器::new().expect("NMPC Init Failed"));
        let mut solver_ready = false;
        let mut tick_count: u64 = 0;
        let mut log_file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("nmpc_telemetry.csv")
            .expect("Failed to create NMPC telemetry log");
        let _ = writeln!(log_file, "tick,cur_x,cur_y,cur_yaw,v_cmd,w_cmd");
        let mut ticker = tokio::time::interval(Duration::from_millis(10));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            ticker.tick().await;
            tick_count += 1;
            let (initialized, cur_v, last_time, cur_x, cur_y, cur_yaw, tgt_x, tgt_y, _tgt_yaw, mut goal_reached, bev_grid) = {
                let lock = ctrl_context.read().unwrap();
                (
                    lock.is_initialized, lock.current_velocity, lock.last_update_time,
                    lock.current_x, lock.current_y, lock.current_yaw,
                    lock.target_x, lock.target_y, lock.target_yaw,
                    lock.goal_reached, lock.bev_grid.clone()
                )
            };
            if !initialized {
                continue;
            }
            let watchdog_timeout = last_time.elapsed();
            if watchdog_timeout > std::time::Duration::from_millis(5000) {
                let zero_cmd = dora_node_api::arrow::array::Float32Array::from(vec![0.0f32, 0.0f32]);
                let _ = node.send_output("control_cmd".to_string().into(), MetadataParameters::default(), zero_cmd);
                continue;
            }
            if !solver_ready {
                println!("✓ Pure BEV-guided NMPC Solver running. No conflicting forces active.");
                solver_ready = true;
            }
            let dx = tgt_x - cur_x;
            let dy = tgt_y - cur_y;
            let target_distance = (dx * dx + dy * dy).sqrt();
            if target_distance < 0.20 {
                if !goal_reached {
                    println!("\n🏆 [NEXUS Goal Guard] Destination Reached cleanly.");
                    goal_reached = true;
                    let mut lock = ctrl_context.write().unwrap();
                    lock.goal_reached = true;
                }
            }
            let mut active_obstacles = Vec::new();
            let (v_cmd, w_cmd) = if goal_reached {
                (0.0, 0.0) 
            } else {
                if let Err(e) = brain.设置当前状态(&(0.0, 0.0, 0.0, cur_v)) {
                    eprintln!("Failed to anchor local coordinates: {}", e);
                    continue;
                }
                let local_target_x = dx * cur_yaw.cos() + dy * cur_yaw.sin();
                let local_target_y = -dx * cur_yaw.sin() + dy * cur_yaw.cos();
                let local_target_yaw = local_target_y.atan2(local_target_x).clamp(-0.25, 0.25);
                let target_limit = 1.2f64;
                let (scaled_x, scaled_y) = if target_distance > target_limit {
                    let scale = target_limit / target_distance;
                    (local_target_x * scale, local_target_y * scale)
                } else {
                    (local_target_x, local_target_y)
                };
                let mut closest_left: Option<(f64, f64)> = None;
                let mut closest_center: Option<(f64, f64)> = None;
                let mut closest_right: Option<(f64, f64)> = None;
                for row in 0..192 {
                    for col in 0..192 {
                        let idx = row * 192 + col;
                        if idx < bev_grid.len() && bev_grid[idx] == 255 {
                            let xl = (191.0 - row as f64) * 0.03125;
                            let yl = (96.0 - col as f64) * 0.03125;
                            if xl >= 0.1 && xl <= 2.2 && yl.abs() <= 0.8 {
                                if yl > 0.15 {
                                    if closest_left.map_or(true, |(best_x, _)| xl < best_x) {
                                        closest_left = Some((xl, yl));
                                    }
                                } else if yl < -0.15 {
                                    if closest_right.map_or(true, |(best_x, _)| xl < best_x) {
                                        closest_right = Some((xl, yl));
                                    }
                                } else {
                                    if closest_center.map_or(true, |(best_x, _)| xl < best_x) {
                                        closest_center = Some((xl, yl));
                                    }
                                }
                            }
                        }
                    }
                }
                if let Some((xl, yl)) = closest_left {
                    active_obstacles.push(动态障碍物 { x: xl, y: yl, a: 0.35, b: 0.25 });
                }
                if let Some((xl, yl)) = closest_center {
                    active_obstacles.push(动态障碍物 { x: xl, y: yl, a: 0.35, b: 0.25 });
                }
                if let Some((xl, yl)) = closest_right {
                    active_obstacles.push(动态障碍物 { x: xl, y: yl, a: 0.35, b: 0.25 });
                }
                let target_velocity = {
                    let speed_limit = 0.80f64.min(target_distance * 0.5);
                    let decel_factor = if let Some((xl, _)) = closest_center {
                        if xl < 1.5 { (xl / 1.5).powi(2).clamp(0.1, 1.0) } else { 1.0 }
                    } else {
                        1.0
                    };
                    speed_limit * decel_factor
                };
                let d_ff = scaled_x / 3.0;
                let mut injection_success = true;
                for k in 0..=20 {
                    let t = (k as f64) / 20.0;
                    let ref_x = 3.0 * (1.0 - t).powi(2) * t * d_ff 
                              + 3.0 * (1.0 - t) * t.powi(2) * (scaled_x - d_ff * local_target_yaw.cos()) 
                              + t.powi(3) * scaled_x;
                    let ref_y = 3.0 * (1.0 - t) * t.powi(2) * (scaled_y - d_ff * local_target_yaw.sin()) 
                              + t.powi(3) * scaled_y;
                    let ref_yaw = local_target_yaw * (t * t * (3.0 - 2.0 * t));
                    if let Err(e) = brain.设置参考轨迹点(k, &(ref_x, ref_y, ref_yaw, target_velocity)) {
                        eprintln!("Trajectory injection failed: {}", e);
                        injection_success = false;
                        break;
                    }
                }
                if !injection_success {
                    continue;
                }
                let _ = brain.设置动态障碍物硬约束(&active_obstacles);
                match brain.求解最优控制量(cur_v) {
                    Ok((v, w)) => (v, w),
                    Err(_) => (0.0, 0.0),
                }
            };
            {
                let mut lock = ctrl_context.write().unwrap();
                lock.current_velocity = v_cmd;
            }
            let _ = writeln!(log_file, "{},{:.4},{:.4}", tick_count, v_cmd, w_cmd);
            let _ = log_file.flush();
            if tick_count % 100 == 0 {
                let status_str = if goal_reached { "PARKED 🏁" } else if !active_obstacles.is_empty() { "AVOIDING 🔴" } else { "CLEAR 🟢" };
                println!(
                    "[NMPC] Tick: {:<5} | {} | Obstacles: {} | Dist: {:.2}m | v_cmd: {:.3} m/s | w_cmd: {:>6.3} rad/s",
                    tick_count, status_str, active_obstacles.len(), target_distance, v_cmd, w_cmd
                );
            }
            let cmd_arrow = dora_node_api::arrow::array::Float32Array::from(vec![
                v_cmd as f32,
                w_cmd as f32
            ]);
            let _ = node.send_output("control_cmd".to_string().into(), MetadataParameters::default(), cmd_arrow);
        }
    });
    while let Some(event) = events.recv_async().await {
        match 神经事件路由器::分流路由事件(event) {
            Ok(智能自驾信号载荷::物理里程计 { x, y, yaw }) => {
                let mut lock = context.write().unwrap();
                lock.current_x = x;
                lock.current_y = y;
                lock.current_yaw = yaw;
            },
            Ok(智能自驾信号载荷::慢脑自引力领航 { goal_x, goal_y, goal_yaw }) => {
                let mut lock = context.write().unwrap();
                lock.target_x = goal_x;
                lock.target_y = goal_y;
                lock.target_yaw = goal_yaw;
                if !lock.is_initialized {
                    lock.is_initialized = true;
                }
                lock.last_update_time = std::time::Instant::now();
            },
            Ok(智能自驾信号载荷::实相鸟瞰图(grid)) => {
                let mut lock = context.write().unwrap();
                lock.bev_grid = grid;
            },
            Ok(智能自驾信号载荷::系统下线信号) => {
                break;
            },
            _ => {}
        }
    }
    control_handle.abort();
    Ok(())
}
