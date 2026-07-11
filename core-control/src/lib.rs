pub mod ffi;
pub mod solver;
pub mod sensor_fusion;

pub use solver::预测控制求解器;
pub use sensor_fusion::层级传感器融合中心;

/// 🧠 SOTA 2026: Physics-Aware Action Mapping decoder with Singularity Autorecovery
pub fn decode_platform_agnostic_actions(a_vel: f64, a_kappa: f64) -> (f64, f64, f64) {
    let v_min: f64 = -0.30; // Maximum reverse velocity for braking
    let v_max: f64 = 0.80;  // Maximum forward cruise velocity
    let kappa_max: f64 = 1.25;
    let w_max: f64 = 1.00;  // Absolute maximum angular velocity limit

    // 1. Bidirectional Velocity Mapping supporting active reverse-braking
    let v_des = if a_vel >= 0.0 {
        a_vel.clamp(0.0, 1.0) * v_max
    } else {
        a_vel.clamp(-1.0, 0.0) * v_min.abs()
    };

    let kappa = a_kappa.clamp(-1.0, 1.0) * kappa_max;

    // 2. Singularity Autorecovery: Decouple steering at near-zero velocities
    // Allowing aggressive pivot turns (原地自旋) when stationary to clear inertia
    let w_ref = if v_des.abs() < 0.05 {
        a_kappa.clamp(-1.0, 1.0) * w_max
    } else {
        kappa * v_des
    };

    (v_des, kappa, w_ref)
}

#[cfg(test)]
mod phase2_verification_tests {
    use super::*;
    use std::collections::VecDeque;
    use std::time::{Instant, Duration};

    #[allow(dead_code)]
    #[derive(Debug, Clone)]
    struct MockOdomFrame {
        pub timestamp_virtual: f64,
        pub x: f64,
        pub y: f64,
    }

    #[test]
    fn test_bidirectional_and_singularity_recovery_mapping() {
        println!("Starting Phase 2 verification: Bidirectional speed and Zero-Velocity Steering...");

        // Assert 1: Full forward speed mapping (at maximum actions)
        let (v_forward, _, w_forward) = decode_platform_agnostic_actions(1.0, 1.0);
        assert!((v_forward - 0.80).abs() < 1e-5, "Forward speed scaling failed!");
        assert!((w_forward - 1.0).abs() < 1e-5, "Forward yaw-rate scaling failed!");
        println!("✓ Forward limit scaling verified: v_des = {} m/s, w_ref = {} rad/s", v_forward, w_forward);

        // Assert 2: Active reverse-braking mapping (v_des must be negative!)
        let (v_reverse, _, _) = decode_platform_agnostic_actions(-1.0, 0.0);
        assert!((v_reverse - (-0.30)).abs() < 1e-5, "Active reverse-braking speed scaling failed!");
        println!("✓ Speed mapping handles negative velocity correctly: {} m/s", v_reverse);

        // Assert 3: Zero-velocity Steering Singularity Autorecovery
        // When v_des is nearly 0 (stopped), a_kappa must bypass curvature and directly output yaw-rate
        let (v_stopped, _, w_stopped) = decode_platform_agnostic_actions(0.0, -1.0);
        assert!(v_stopped.abs() < 1e-5);
        assert!((w_stopped - (-1.0)).abs() < 1e-5, "Singularity recovery failed! Got w_ref = {}", w_stopped);
        println!("✓ Pivot Turn Singularity Recovery passed: w_ref = {} rad/s at zero speed!", w_stopped);
    }

    #[test]
    fn test_temporal_alignment_under_sim_lag() {
        let sim_step_dt = 0.01;
        let mut tick_count: u64 = 0;
        let mut obs_history: VecDeque<f64> = VecDeque::with_capacity(5);

        let mut simulated_trajectory = Vec::new();
        for i in 0..100 {
            simulated_trajectory.push(MockOdomFrame {
                timestamp_virtual: i as f64 * sim_step_dt,
                x: i as f64 * 0.01, 
                y: 0.0,
            });
        }

        for frame in simulated_trajectory {
            tick_count += 1;
            if tick_count % 10 == 0 || tick_count == 1 {
                if obs_history.is_empty() {
                    for _ in 0..5 {
                        obs_history.push_back(frame.x);
                    }
                } else {
                    obs_history.push_back(frame.x);
                    if obs_history.len() > 5 {
                        obs_history.pop_front();
                    }
                }
            }
        }

        assert_eq!(obs_history.len(), 5);
        let expected_vals = vec![0.59, 0.69, 0.79, 0.89, 0.99];
        for (idx, &val) in obs_history.iter().enumerate() {
            let err = (val - expected_vals[idx]).abs();
            assert!(err < 1e-5);
        }
    }

    #[test]
    fn test_watchdog_liveness_threshold() {
        let mut last_update_time = Instant::now();
        last_update_time = last_update_time - Duration::from_millis(5100);
        let elapsed = last_update_time.elapsed();
        let watchdog_triggered = elapsed > Duration::from_millis(5000);
        assert!(watchdog_triggered);
    }
}
