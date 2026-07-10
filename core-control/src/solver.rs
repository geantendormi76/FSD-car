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
        // 1. 分配物理内存胶囊
        let capsule = unsafe { diff_drive_car_acados_create_capsule() };
        if capsule.is_null() {
            return Err("❌ NMPC 求解器 C 内存胶囊分配失败！".to_string());
        }
        
        // 2. 执行内部矩阵与求解器初始化
        let status = unsafe { diff_drive_car_acados_create(capsule) };
        if status != 0 {
            // 初始化失败时，必须手动释放刚刚分配的胶囊，防止内存泄漏
            unsafe { diff_drive_car_acados_free_capsule(capsule); }
            return Err(format!("❌ NMPC 求解器内部初始化失败，状态码: {}", status));
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
            let nlp_out = diff_drive_car_acados_get_nlp_out(self.capsule); // 🛡️ SOTA 修正：获取 nlp_out 句柄 [cite: 3.1.6]

            // 🛡️ 架构师 2026 SOTA 修正：传入 7 个完备参数（新增 nlp_out），彻底根治由栈破坏引起的 SIGSEGV 段错误！
            let status_lbx = ocp_nlp_constraints_model_set(config, dims, nlp_in, nlp_out, 0, field_lbx.as_ptr(), state.as_ptr() as *mut c_void);
            let status_ubx = ocp_nlp_constraints_model_set(config, dims, nlp_in, nlp_out, 0, field_ubx.as_ptr(), state.as_ptr() as *mut c_void);

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

            // 🛡️ 架构师 2026 SOTA 修正：设置参考代价函数 yref 必须使用 ocp_nlp_cost_model_set
            let status = if 预测步 == 20 { 
                // 终端步 (Terminal Stage): ny_e = 4
                let yref_e = [ref_x, ref_y, ref_yaw, ref_v];
                ocp_nlp_cost_model_set(config, dims, nlp_in, 预测步, field_yref.as_ptr(), yref_e.as_ptr() as *mut c_void)
            } else {
                // 中间步 (Intermediate Stage): ny = 6 (补齐 a 和 omega 的参考值 0.0)
                let yref = [ref_x, ref_y, ref_yaw, ref_v, 0.0, 0.0];
                ocp_nlp_cost_model_set(config, dims, nlp_in, 预测步, field_yref.as_ptr(), yref.as_ptr() as *mut c_void)
            };

            if status != 0 {
                return Err(format!("❌ NMPC 第 {} 步参考轨迹注入失败！", 预测步));
            }
        }
        Ok(())
    }

    /// 🎯 战役三核心：注入动态障碍物非线性硬约束参数
    /// 将青蛙眼坍缩出的虚拟障碍物坐标，以 100Hz 频率物理烧录进求解器的 20 个预测步中
    pub fn 设置动态障碍物硬约束(&mut self, obs_x: f64, obs_y: f64, a_axis: f64, b_axis: f64) -> Result<(), String> {
        // 严格对齐 Python 侧定义的 parameters 顺序: [obs_x, obs_y, a_axis, b_axis]
        let p = [obs_x, obs_y, a_axis, b_axis];
        unsafe {
            // 必须为 NMPC 的每一个预测步 (0 到 20) 都更新这个参数
            for stage in 0..=20 {
                // np = 4 表示我们传入了 4 个 f64 参数
                let status = diff_drive_car_acados_update_params(self.capsule, stage, p.as_ptr(), 4);
                if status != 0 {
                    return Err(format!("❌ 致命错误：第 {} 步障碍物硬约束参数注入失败！", stage));
                }
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

            ocp_nlp_out_get(config, dims, nlp_out, 0, field_u.as_ptr(), u_opt.as_mut_ptr() as *mut c_void);

            let a_opt = u_opt[0];
            let w_cmd_raw = u_opt[1];
            // 物理运动学积分：v_cmd = current_v + a * dt (dt=0.01)
            let mut v_cmd = 当前线速度 + a_opt * 0.01;
            // 🎯 3.3.1 极速自愈：解锁 NMPC 求解器输出硬限幅至 0.80 m/s，并调大角速度至 1.0 以支撑中速切弯 [cite: Sim2Real-AD]
            if v_cmd > 0.80 { v_cmd = 0.80; }
            if v_cmd < 0.0 { v_cmd = 0.0; }
            // 🎯 3.3.1 极速自愈：删除残余的 0.6 覆盖锁，彻底释放 1.0 rad/s 极限大角度打舵潜能！
            let w_cmd = w_cmd_raw.clamp(-1.0, 1.0);
            Ok((v_cmd, w_cmd))
        }
    }
}

// 🛡️ 内存守卫：RAII 机制。当 `预测控制求解器` 离开作用域时，自动调用 C 语言的 free 释放内存，彻底杜绝内存泄漏！
impl Drop for 预测控制求解器 {
    fn drop(&mut self) {
        unsafe {
            // 必须严格按照逆序释放：先释放内部矩阵，再释放胶囊本身
            diff_drive_car_acados_free(self.capsule);
            diff_drive_car_acados_free_capsule(self.capsule);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nmpc_solver_lifecycle_and_compute() {
        println!("🛡️ [单测启动] 正在测试 NMPC 求解器 C FFI 边界...");
        
        // 1. 测试内存胶囊分配 (如果 C 动态库链接失败，这里会直接 panic)
        let mut solver = 预测控制求解器::new().expect("❌ 求解器初始化失败，请检查 C 动态库链接！");
        println!("✅ 求解器内存胶囊分配成功！");

        // 2. 测试状态注入 (温启动：当前处于原点，静止)
        solver.设置当前状态(0.0, 0.0, 0.0, 0.0).expect("❌ 初始状态注入失败");
        println!("✅ 初始状态注入成功！");

        // 3. 测试参考轨迹注入 (预测步长 N=20)
        for k in 0..=20 {
            // 假设目标在正前方 1.0 米处，期望线速度 0.3 m/s
            solver.设置参考轨迹点(k, 1.0, 0.0, 0.0, 0.3).expect(&format!("❌ 第 {} 步参考轨迹注入失败", k));
        }
        println!("✅ 参考轨迹注入成功！");

        // 4. 测试核心求解算子
        let (v_cmd, w_cmd) = solver.求解最优控制量(0.0).expect("❌ NMPC 求解失败");
        println!("✅ 求解成功！输出控制量 -> 线速度: {:.3} m/s, 角速度: {:.3} rad/s", v_cmd, w_cmd);

        // 5. 物理主权断言：确保输出严格遵守我们在 generate_solver.py 中定义的硬约束
        assert!(v_cmd >= 0.0 && v_cmd <= 0.3, "线速度超限！");
        assert!(w_cmd >= -0.6 && w_cmd <= 0.6, "角速度超限！");
        
        // 6. 隐式测试：当 `solver` 离开此作用域时，Rust 会自动调用 Drop trait。
        // 如果 C 侧的 free 逻辑有误，这里会触发段错误 (Segmentation Fault)。
        println!("✅ 测试结束，准备安全释放 C 语言内存胶囊...");
    }
}