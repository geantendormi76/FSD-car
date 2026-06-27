use super::ffi::*;
use std::ffi::CString;
use std::os::raw::c_void;

/// 🛡️ 规控大脑：NMPC 求解器安全封装金库
/// 彻底隔离 C 语言的裸指针，对外提供 100% 内存安全的 Rust 接口
pub struct 预测控制求解器 {
    capsule: *mut diff_drive_car_solver_capsule,
}

// 🛡️ 架构师指令：显式声明线程安全。
// 只要我们保证在 Rust 层以 `&mut self` 独占借用的方式调用，C 求解器在多线程间传递就是安全的。
unsafe impl Send for 预测控制求解器 {}

impl 预测控制求解器 {
    /// 构造函数：在堆内存中分配 C 求解器
    pub fn new() -> Result<Self, String> {
        let capsule = unsafe { diff_drive_car_acados_create() };
        if capsule.is_null() {
            return Err("❌ NMPC 求解器 C 内存胶囊分配失败！请检查动态库链接。".to_string());
        }
        Ok(Self { capsule })
    }

    /// 注入当前物理状态 (对应 Python 中的 lbx, ubx)
    pub fn 设置当前状态(&mut self, x: f64, y: f64, yaw: f64, v: f64) -> Result<(), String> {
        let state = [x, y, yaw, v];
        let field_lbx = CString::new("lbx").unwrap();
        let field_ubx = CString::new("ubx").unwrap();
        
        unsafe {
            let config = diff_drive_car_acados_get_nlp_config(self.capsule);
            let dims = diff_drive_car_acados_get_nlp_dims(self.capsule);
            let nlp_in = diff_drive_car_acados_get_nlp_in(self.capsule);

            let status_lbx = ocp_nlp_in_set(config, dims, nlp_in, 0, field_lbx.as_ptr(), state.as_ptr() as *mut c_void);
            let status_ubx = ocp_nlp_in_set(config, dims, nlp_in, 0, field_ubx.as_ptr(), state.as_ptr() as *mut c_void);

            if status_lbx != 0 || status_ubx != 0 {
                return Err("❌ NMPC 初始状态 (x0) 注入失败！".to_string());
            }
        }
        Ok(())
    }

    /// 注入未来预测步的参考轨迹点 (对应 Python 中的 yref)
    pub fn 设置参考轨迹点(&mut self, 预测步: i32, ref_x: f64, ref_y: f64, ref_yaw: f64, ref_v: f64) -> Result<(), String> {
        let field_yref = CString::new("yref").unwrap();
        
        unsafe {
            let config = diff_drive_car_acados_get_nlp_config(self.capsule);
            let dims = diff_drive_car_acados_get_nlp_dims(self.capsule);
            let nlp_in = diff_drive_car_acados_get_nlp_in(self.capsule);

            let status = if 预测步 == 20 { 
                // 终端步 (Terminal Stage): ny_e = 4
                let yref_e = [ref_x, ref_y, ref_yaw, ref_v];
                ocp_nlp_in_set(config, dims, nlp_in, 预测步, field_yref.as_ptr(), yref_e.as_ptr() as *mut c_void)
            } else {
                // 中间步 (Intermediate Stage): ny = 6 (补齐 a 和 omega 的参考值 0.0)
                let yref = [ref_x, ref_y, ref_yaw, ref_v, 0.0, 0.0];
                ocp_nlp_in_set(config, dims, nlp_in, 预测步, field_yref.as_ptr(), yref.as_ptr() as *mut c_void)
            };

            if status != 0 {
                return Err(format!("❌ NMPC 第 {} 步参考轨迹注入失败！", 预测步));
            }
        }
        Ok(())
    }

    /// 执行 RTI-SQP 求解，并返回平滑处理后的 (线速度, 角速度)
    pub fn 求解最优控制量(&mut self, 当前线速度: f64) -> Result<(f64, f64), String> {
        unsafe {
            let status = diff_drive_car_acados_solve(self.capsule);
            if status != 0 {
                return Err(format!("⚠️ NMPC 求解器发散或失败，状态码: {}", status));
            }

            let config = diff_drive_car_acados_get_nlp_config(self.capsule);
            let dims = diff_drive_car_acados_get_nlp_dims(self.capsule);
            let nlp_out = diff_drive_car_acados_get_nlp_out(self.capsule);

            let field_u = CString::new("u").unwrap();
            let mut u_opt = [0.0f64; 2]; // [a, omega]

            let get_status = ocp_nlp_out_get(config, dims, nlp_out, 0, field_u.as_ptr(), u_opt.as_mut_ptr() as *mut c_void);
            if get_status != 0 {
                return Err("❌ NMPC 无法提取最优控制输出！".to_string());
            }

            let a_opt = u_opt[0];
            let w_cmd_raw = u_opt[1];

            // 物理运动学积分：v_cmd = current_v + a * dt (dt=0.01)
            let mut v_cmd = 当前线速度 + a_opt * 0.01;
            if v_cmd < 0.0 { v_cmd = 0.0; } // 限制不能倒车
            if v_cmd > 0.3 { v_cmd = 0.3; } // 最大速度限制

            // 角速度限幅 [-0.6, 0.6] (继承自 generate_solver.py 的硬约束)
            let w_cmd = w_cmd_raw.clamp(-0.6, 0.6);

            Ok((v_cmd, w_cmd))
        }
    }
}

// 🛡️ 内存守卫：RAII 机制。当 `预测控制求解器` 离开作用域时，自动调用 C 语言的 free 释放内存，彻底杜绝内存泄漏！
impl Drop for 预测控制求解器 {
    fn drop(&mut self) {
        unsafe {
            diff_drive_car_acados_free(self.capsule);
        }
    }
}