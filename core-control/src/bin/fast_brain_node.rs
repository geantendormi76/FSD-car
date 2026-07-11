use core_control::预测控制求解器;
use core_control::solver::动态障碍物;
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use ort::session::Session;
use ort::value::Value;
use std::time::Instant;
use std::collections::VecDeque;
use std::io::Write;

fn auto_load_onnx_dylib() {
    if std::env::var("ORT_DYLIB_PATH").is_ok() {
        return;
    }
    let capi_dir = "/home/zhz/isaacsim/kit/python/lib/python3.12/site-packages/onnxruntime/capi";
    if std::path::Path::new(capi_dir).exists() {
        if let Ok(entries) = std::fs::read_dir(capi_dir) {
            for entry in entries {
                if let Ok(entry) = entry {
                    let path = entry.path();
                    if let Some(file_name) = path.file_name() {
                        let name_str = file_name.to_string_lossy();
                        if name_str.starts_with("libonnxruntime.so") {
                            let abs_path = path.to_string_lossy().into_owned();
                            println!("✓ Self-healing loaded versioned onnx dylib: {}", abs_path);
                            std::env::set_var("ORT_DYLIB_PATH", abs_path);
                            return;
                        }
                    }
                }
            }
        }
    }
    let fallback_path = "/home/zhz/fsd-car/core-perception/lib_dylib/libonnxruntime.so";
    if std::path::Path::new(fallback_path).exists() {
        std::env::set_var("ORT_DYLIB_PATH", fallback_path);
    }
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    auto_load_onnx_dylib();
    println!("NEXUS SOTA 2026 - PPO & NMPC Unified FSD Control Node starting...");

    let (mut node, mut events) = DoraNode::init_from_env()?;

    let mut onnx_path = std::path::PathBuf::from("model/spiced_brain.onnx");
    if !onnx_path.exists() {
        onnx_path = std::path::PathBuf::from("/home/zhz/fsd-car/model/spiced_brain.onnx");
    }
    if !onnx_path.exists() {
        return Err(eyre!("Critical Error: Spiced PPO ONNX weights missing at {:?}", onnx_path));
    }

    let mut ppo_session = Session::builder()
        .and_then(|b| b.with_intra_threads(1))
        .and_then(|b| b.commit_from_file(&onnx_path))
        .expect("Failed to mount Spiced PPO Brain");
    println!("✓ Spiced PPO Brain model loaded. Ready for 100Hz/10Hz Cascade inference.");

    let mut is_initialized = false;
    let mut desired_force_x = 0.0f64;
    let mut desired_force_y = 0.0f64;
    let mut current_velocity = 0.0f64;
    let mut current_x = 0.0f64;
    let mut current_y = 0.0f64;
    let mut current_yaw = 0.0f64;
    let mut target_x = 0.0f64;
    let mut target_y = 0.0f64;
    let mut target_yaw = 0.0f64;
    let mut last_human_prior_time = Instant::now();

    let mut brain = 预测控制求解器::new().expect("Failed to initialize NMPC solver");
    let mut obs_history: VecDeque<Vec<f32>> = VecDeque::with_capacity(5);
    let mut solver_ready = false;
    let mut tick_count: u64 = 0;

    let mut filtered_force_x = 0.0f64;
    let mut filtered_force_y = 0.0f64;
    let mut filtered_target_x = 0.0f64;
    let mut filtered_target_y = 0.0f64;
    let mut filtered_target_yaw = 0.0f64;
    let mut filtered_obs_x = 1000.0f64;
    let mut filtered_obs_y = 1000.0f64;

    let mut current_ppo_v = 0.0f64;
    let mut current_ppo_w = 0.0f64;
    let mut last_omega_w = 0.0f64;
    let mut printed_unseal = false;

    let mut obs_memory_x = 1000.0f64;
    let mut obs_memory_y = 1000.0f64;
    let mut obs_memory_timer = 0i32;

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

    let mut audit_file = std::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open("/tmp/nexus_audit_trace.csv")
        .expect("Failed to create audit log");
    let _ = writeln!(
        audit_file,
        "tick,cur_x,cur_y,cur_yaw,tgt_x,tgt_y,tgt_yaw,loc_x,loc_y,dist,raw_fx,raw_fy,ppo_v,ppo_w,nmpc_obs_y,nmpc_v,nmpc_w,ref_yaw_pull"
    );

    println!("NEXUS Active Event Loop online. Waiting for synchronous triggers...");

    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                let id_str = id.as_str();

                if id_str == "obstacle_force" {
                    let force_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map obstacle force to Float32Array"))?;
                    if force_array.len() >= 2 {
                        desired_force_x = force_array.value(0) as f64;
                        desired_force_y = force_array.value(1) as f64;
                    }
                } else if id_str == "human_prior" {
                    let prior_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map human prior to Float32Array"))?;
                    if prior_array.len() >= 3 {
                        target_x = prior_array.value(0) as f64;
                        target_y = prior_array.value(1) as f64;
                        target_yaw = prior_array.value(2) as f64;
                        if !is_initialized {
                            is_initialized = true;
                            println!("Warmstart gate unsealed: Slow brain homing signal linked!");
                        }
                        last_human_prior_time = Instant::now();
                    }
                } else if id_str == "odometry" {
                    let odom_array = data.as_any().downcast_ref::<dora_node_api::arrow::array::Float32Array>()
                        .ok_or_else(|| eyre!("Failed to map odometry to Float32Array"))?;
                    if odom_array.len() >= 3 {
                        current_x = odom_array.value(0) as f64;
                        current_y = odom_array.value(1) as f64;
                        current_yaw = odom_array.value(2) as f64;
                    }

                    if !is_initialized {
                        continue;
                    }

                    tick_count += 1;

                    let watchdog_timeout = last_human_prior_time.elapsed();
                    if watchdog_timeout > std::time::Duration::from_millis(5000) {
                        eprintln!(
                            "Failsafe Watchdog triggered: Timeout {:.2?}s! Stop vehicle.",
                            watchdog_timeout.as_secs_f64()
                        );
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

                    if let Err(e) = brain.设置当前状态(0.0, 0.0, 0.0, current_velocity) {
                        eprintln!("Failed to anchor local coordinates: {}", e);
                        continue;
                    }

                    let compensated_force_y = desired_force_y - (1.25f64 * last_omega_w);
                    let compensated_force_x = desired_force_x + (0.15f64 * last_omega_w.abs());

                    let (active_force_x, active_force_y) = if current_velocity.abs() < 0.04 {
                        (0.0f64, 0.0f64)
                    } else {
                        (compensated_force_x.clamp(-1.5, 0.0), compensated_force_y.clamp(-1.5, 1.5))
                    };

                    let cross_coupling_decel = -(active_force_y.abs() * 0.45);
                    let combined_force_x = active_force_x + cross_coupling_decel;

                    filtered_force_x += 0.125f64 * (combined_force_x - filtered_force_x);
                    filtered_force_y += 0.125f64 * (active_force_y - filtered_force_y);

                    let mut suppressed_force_y = filtered_force_y;
                    let abs_omega = last_omega_w.abs();
                    if abs_omega > 0.12 {
                        let suppression_factor = (-4.5_f64 * (abs_omega - 0.12_f64)).exp();
                        suppressed_force_y *= suppression_factor.clamp(0.15, 1.0);
                    }

                    if tick_count == 1 || (target_x == 0.0 && target_y == 0.0) {
                        filtered_target_x = target_x;
                        filtered_target_y = target_y;
                        filtered_target_yaw = target_yaw;
                    } else {
                        let spatial_filter_coeff = 0.15f64;
                        filtered_target_x += spatial_filter_coeff * (target_x - filtered_target_x);
                        filtered_target_y += spatial_filter_coeff * (target_y - filtered_target_y);
                        filtered_target_yaw += spatial_filter_coeff * (target_yaw - filtered_target_yaw);
                    }

                    let dx = filtered_target_x - current_x;
                    let dy = filtered_target_y - current_y;
                    let local_target_x = dx * current_yaw.cos() + dy * current_yaw.sin();
                    let local_target_y = -dx * current_yaw.sin() + dy * current_yaw.cos();
                    let local_target_yaw_final = (filtered_target_yaw - current_yaw).clamp(-0.25, 0.25);
                    let target_distance = (local_target_x * local_target_x + local_target_y * local_target_y).sqrt();

                    let local_target_yaw = if target_distance > 0.60 {
                        local_target_y.atan2(local_target_x).clamp(-0.25, 0.25)
                    } else {
                        let los_yaw = local_target_y.atan2(local_target_x);
                        let alpha = ((target_distance - 0.15) / 0.45).clamp(0.0, 1.0);
                        (alpha * los_yaw + (1.0 - alpha) * local_target_yaw_final).clamp(-0.25, 0.25)
                    };

                    let target_limit = 1.2f64;
                    let (scaled_x, scaled_y) = if target_distance > target_limit {
                        let scale = target_limit / target_distance;
                        (local_target_x * scale, local_target_y * scale)
                    } else {
                        (local_target_x, local_target_y)
                    };

                    if tick_count % 10 == 0 || tick_count == 1 {
                        let obs_single = vec![
                            (local_target_x * 0.20) as f32,
                            (local_target_y * 0.20) as f32,
                            (target_distance * 0.20) as f32,
                            active_force_x as f32,
                            active_force_y as f32,
                        ];
                        if obs_history.is_empty() {
                            for _ in 0..5 {
                                obs_history.push_back(obs_single.clone());
                            }
                        } else {
                            obs_history.push_back(obs_single);
                            if obs_history.len() > 5 {
                                obs_history.pop_front();
                            }
                        }

                        let mut input_data = Vec::with_capacity(25);
                        for frame in &obs_history {
                            input_data.extend_from_slice(frame);
                        }

                        let input_tensor = Value::from_array(([1, 25], input_data))
                            .expect("Failed to build input tensor");
                        let inputs = ort::inputs![input_tensor];
                        let outputs = ppo_session.run(inputs).expect("ONNX inference failed");
                        let (_, outputs_data) = outputs[0].try_extract_tensor::<f32>()
                            .expect("Failed to extract output tensor");

                        current_ppo_v = outputs_data[0] as f64;
                        current_ppo_w = outputs_data[1] as f64;
                    }

                    // Physics-Aware Action Mapping (PAM) scaling decoding
                    let v_max = 0.80f64;
                    let kappa_max = 1.25f64;
                    let ppo_velocity = current_ppo_v.clamp(0.0, 1.0) * v_max;
                    let ppo_curvature = current_ppo_w.clamp(-1.0, 1.0) * kappa_max;
                    let w_ref = ppo_curvature * ppo_velocity;

                    let mut max_speed_limit = 0.80f64;
                    if tick_count <= 200 {
                        max_speed_limit = 0.25f64;
                        if tick_count == 1 {
                            println!("🚀 [NEXUS AR] Speed Gating active. Capped at 0.25 m/s.");
                        }
                    } else if !printed_unseal {
                        println!("🚀 [NEXUS AR] Speed Gating unsealed! Unleashing full 0.80 m/s PPO potential.");
                        printed_unseal = true;
                    }

                    let target_velocity = ppo_velocity.clamp(0.0, max_speed_limit);
                    let rebound_yaw = w_ref;

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

                        let spiced_ref_x = ref_x;
                        let spiced_ref_y = ref_y;
                        let ref_yaw = local_target_yaw * (t * t * (3.0 - 2.0 * t)) + (rebound_yaw * t);

                        if let Err(e) = brain.设置参考轨迹点(k, spiced_ref_x, spiced_ref_y, ref_yaw, target_velocity) {
                            eprintln!("Failed to inject spline reference at stage {}: {}", k, e);
                            injection_success = false;
                            break;
                        }
                    }

                    if !injection_success {
                        continue;
                    }

                    let sensor_active = suppressed_force_y.abs() > 0.04 || filtered_force_x < -0.04;
                    let (obs_x, obs_y, axis_a, axis_b) = if sensor_active {
                        obs_memory_x = 0.65;
                        obs_memory_y = -suppressed_force_y.clamp(-0.35, 0.35);
                        obs_memory_timer = 40;
                        (obs_memory_x, obs_memory_y, 0.35, 0.25)
                    } else if obs_memory_timer > 0 {
                        obs_memory_timer -= 1;
                        obs_memory_x -= current_velocity * 0.01;
                        if obs_memory_x < -0.20 {
                            obs_memory_timer = 0;
                            obs_memory_x = 1000.0;
                            obs_memory_y = 1000.0;
                        }
                        (obs_memory_x, obs_memory_y, 0.35, 0.25)
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

                    let (v_cmd_solved, w_cmd) = match brain.求解最优控制量(current_velocity) {
                        Ok((v, w)) => (v, w),
                        Err(e) => {
                            eprintln!("NMPC calculation error: {}", e);
                            (0.0, 0.0)
                        }
                    };

                    let v_cmd = v_cmd_solved;
                    current_velocity = v_cmd;

                    let _ = writeln!(
                        log_file,
                        "{}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}",
                        tick_count, current_x, current_y, current_yaw, target_x, target_y, target_yaw, active_force_x, active_force_y, v_cmd, w_cmd, current_velocity
                    );
                    let _ = log_file.flush();

                    let _ = writeln!(
                        audit_file,
                        "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                        tick_count, current_x, current_y, current_yaw, target_x, target_y, target_yaw, local_target_x, local_target_y, target_distance, active_force_x, active_force_y, current_ppo_v, current_ppo_w, obs_y, v_cmd, w_cmd, local_target_yaw
                    );
                    let _ = audit_file.flush();

                    last_omega_w = w_cmd;

                    if tick_count % 100 == 0 {
                        let status_str = if filtered_obs_x < 100.0 { "DEFENSIVE 🔴" } else { "CLEAR 🟢" };
                        print!(
                            "[FSD SOTA 2026] Tick: {:<5} | {} | Target dist: {:.2}m | PPO Cmd: v={:.3}, w={:>6.3} | v_cmd: {:.3} m/s | w_cmd: {:>6.3} rad/s\r",
                            tick_count, status_str, target_distance, current_ppo_v, current_ppo_w, v_cmd, w_cmd
                        );
                        std::io::stdout().flush().unwrap();
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
            }
            Event::Stop(_) => {
                println!("DORA Stop event caught. Terminating fast brain...");
                break;
            }
            _ => {}
        }
    }

    Ok(())
}
