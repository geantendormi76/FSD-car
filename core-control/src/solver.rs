use super::ffi::*;
use std::ffi::CString;
use std::os::raw::c_void;

#[derive(Debug, Clone, Copy)]
pub struct 动态障碍物 {
    pub x: f64,
    pub y: f64,
    pub a: f64,
    pub b: f64,
}

pub struct 预测控制求解器 {
    capsule: *mut diff_drive_car_solver_capsule,
}

unsafe impl Send for 预测控制求解器 {}

impl 预测控制求解器 {
    pub fn new() -> Result<Self, String> {
        let capsule = unsafe { diff_drive_car_acados_create_capsule() };
        if capsule.is_null() {
            return Err("Capsule allocation failed".to_string());
        }
        let status = unsafe { diff_drive_car_acados_create(capsule) };
        if status != 0 {
            unsafe { diff_drive_car_acados_free_capsule(capsule); }
            return Err(format!("Solver init failed, status: {}", status));
        }
        Ok(Self { capsule })
    }

    pub fn 设置当前状态(&mut self, x: f64, y: f64, yaw: f64, v: f64) -> Result<(), String> {
        let state = [x, y, yaw, v];
        let field_lbx = CString::new("lbx").unwrap();
        let field_ubx = CString::new("ubx").unwrap();
        unsafe {
            let config = diff_drive_car_acados_get_nlp_config(self.capsule);
            let dims = diff_drive_car_acados_get_nlp_dims(self.capsule);
            let nlp_in = diff_drive_car_acados_get_nlp_in(self.capsule);
            let nlp_out = diff_drive_car_acados_get_nlp_out(self.capsule);
            let status_lbx = ocp_nlp_constraints_model_set(config, dims, nlp_in, nlp_out, 0, field_lbx.as_ptr(), state.as_ptr() as *mut c_void);
            let status_ubx = ocp_nlp_constraints_model_set(config, dims, nlp_in, nlp_out, 0, field_ubx.as_ptr(), state.as_ptr() as *mut c_void);
            if status_lbx != 0 || status_ubx != 0 {
                return Err("Failed to inject current state (x0)".to_string());
            }
        }
        Ok(())
    }

    pub fn 设置参考轨迹点(&mut self, stage: i32, ref_x: f64, ref_y: f64, ref_yaw: f64, ref_v: f64) -> Result<(), String> {
        let field_yref = CString::new("yref").unwrap();
        unsafe {
            let config = diff_drive_car_acados_get_nlp_config(self.capsule);
            let dims = diff_drive_car_acados_get_nlp_dims(self.capsule);
            let nlp_in = diff_drive_car_acados_get_nlp_in(self.capsule);
            let status = if stage == 20 {
                let yref_e = [ref_x, ref_y, ref_yaw, ref_v];
                ocp_nlp_cost_model_set(config, dims, nlp_in, stage, field_yref.as_ptr(), yref_e.as_ptr() as *mut c_void)
            } else {
                let yref = [ref_x, ref_y, ref_yaw, ref_v, 0.0, 0.0];
                ocp_nlp_cost_model_set(config, dims, nlp_in, stage, field_yref.as_ptr(), yref.as_ptr() as *mut c_void)
            };
            if status != 0 {
                return Err(format!("Failed to set reference trajectory at stage {}", stage));
            }
        }
        Ok(())
    }

    // 🛡️ 架构师自愈：恢复标准的 3 圆参数硬约束，完美对齐已编译的 C 求解器模型！
    pub fn 设置动态障碍物硬约束(&mut self, 障碍物列表: &[动态障碍物]) -> Result<(), String> {
        let mut p = [1000.0f64; 12]; // Default to far away (1000m)
        for i in 0..3 {
            p[4 * i + 2] = 0.1;
            p[4 * i + 3] = 0.1;
        }
        for (i, obs) in 障碍物列表.iter().take(3).enumerate() {
            p[4 * i + 0] = obs.x;
            p[4 * i + 1] = obs.y;
            p[4 * i + 2] = obs.a;
            p[4 * i + 3] = obs.b;
        }
        unsafe {
            for stage in 0..=20 {
                let status = diff_drive_car_acados_update_params(self.capsule, stage, p.as_ptr(), 12);
                if status != 0 {
                    return Err(format!("Failed to inject obstacles parameters at stage {}", stage));
                }
            }
        }
        Ok(())
    }

    pub fn 求解最优控制量(&mut self, 当前线速度: f64) -> Result<(f64, f64), String> {
        unsafe {
            let status = diff_drive_car_acados_solve(self.capsule);
            if status != 0 {
                return Err(format!("NMPC solve failed, status: {}", status));
            }
            let config = diff_drive_car_acados_get_nlp_config(self.capsule);
            let dims = diff_drive_car_acados_get_nlp_dims(self.capsule);
            let nlp_out = diff_drive_car_acados_get_nlp_out(self.capsule);
            let field_u = CString::new("u").unwrap();
            let mut u_opt = [0.0f64; 2];
            ocp_nlp_out_get(config, dims, nlp_out, 0, field_u.as_ptr(), u_opt.as_mut_ptr() as *mut c_void);
            let a_opt = u_opt[0];
            let w_cmd_raw = u_opt[1];
            let mut v_cmd = 当前线速度 + a_opt * 0.01;
            if v_cmd > 0.80 { v_cmd = 0.80; }
            if v_cmd < 0.0 { v_cmd = 0.0; }
            let w_cmd = w_cmd_raw.clamp(-1.0, 1.0);
            Ok((v_cmd, w_cmd))
        }
    }
}

impl Drop for 预测控制求解器 {
    fn drop(&mut self) {
        unsafe {
            diff_drive_car_acados_free(self.capsule);
            diff_drive_car_acados_free_capsule(self.capsule);
        }
    }
}
