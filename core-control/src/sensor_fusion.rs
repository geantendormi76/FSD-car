// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

/*
=================================================================
🛰️  NEXUS - FSD-car 2026 级层级多传感器融合标定中心 (Hierarchical Fusion)
设计哲学: PT1 信号阻尼防线 | Mahony 6轴姿态自愈 | 1Hz 视觉反向烧录状态机
=================================================================
*/

use std::f32::consts::PI;

/// 🛡️ 一阶低通滤波器 (PT1 Filter)
/// 1:1 移植并重构自 esp-fc 的 FilterStatePt1 [cite: 1.1.2]
pub struct Pt1滤波器 {
    pub 截止频率_hz: f32,
    pub 滤波输出_v: f32,
}

impl Pt1滤波器 {
    pub fn new(截止频率_hz: f32) -> Self {
        Self {
            截止频率_hz,
            滤波输出_v: 0.0,
        }
    }

    /// 极速一阶阻尼更新 [cite: 1.1.2]
    pub fn 更新(&mut self, 裸输入: f32, dt: f32) -> f32 {
        if self.截止频率_hz <= 0.0 {
            self.滤波输出_v = 裸输入;
            return 裸输入;
        }
        // 计算滤波系数 k = dt / (dt + RC) [cite: 1.1.2]
        let rc = 1.0 / (2.0 * PI * self.截止频率_hz);
        let k = dt / (dt + rc);
        
        self.滤波输出_v += k * (裸输入 - self.滤波输出_v);
        self.滤波输出_v
    }

    pub fn 重置(&mut self) {
        self.滤波输出_v = 0.0;
    }
}

/// 🛡️ 马奥尼 6轴 姿态估计器 (Mahony AHRS - 6-Axis)
/// 1:1 移植自 esp-fc 的 Mahony 经典核心，用于高频解算绝对航向角 [cite: 1.1.2]
pub struct 马奥尼姿态估计器 {
    // 姿态四元数状态机 [q0, q1, q2, q3]
    pub q0: f32,
    pub q1: f32,
    pub q2: f32,
    pub q3: f32,
    
    // 比例增益和积分增益 (Kp, Ki) [cite: 1.1.2]
    pub two_kp: f32,
    pub two_ki: f32,
    
    // 零偏积分累积器 [cite: 1.1.2]
    pub integral_fbx: f32,
    pub integral_fby: f32,
    pub integral_fbz: f32,
}

impl 马奥尼姿态估计器 {
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
            two_kp: 2.0 * kp,
            two_ki: 2.0 * ki,
            integral_fbx: 0.0,
            integral_fby: 0.0,
            integral_fbz: 0.0,
        }
    }

    /// 注入 6轴 IMU 裸数据，解算并更新当前车体姿态 [cite: 1.1.2]
    pub fn 步进更新(&mut self, gx: f32, gy: f32, gz: f32, ax: f32, ay: f32, az: f32, dt: f32) {
        // 如果加速度计没有有效读数，跳过反馈，仅进行陀螺仪开环积分 [cite: 1.1.2]
        let has_accel = !((ax == 0.0) && (ay == 0.0) && (az == 0.0));

        let mut gx_mod = gx;
        let mut gy_mod = gy;
        let mut gz_mod = gz;

        if has_accel {
            // A. 加速度计归一化 [cite: 1.1.2]
            // 🛡️ 架构师自愈：直接在作用域内部进行 let 声明，清除无用 mut 警告，提升内存安全性 [cite: 1.1.2]
            let recip_norm = 1.0 / (ax * ax + ay * ay + az * az).sqrt();
            let ax_n = ax * recip_norm;
            let ay_n = ay * recip_norm;
            let az_n = az * recip_norm;

            // B. 基于当前四元数估计重力方向 (引力矢量在传感器坐标系下的投影) [cite: 1.1.2]
            let halfvx = self.q1 * self.q3 - self.q0 * self.q2;
            let halfvy = self.q0 * self.q1 + self.q2 * self.q3;
            let halfvz = self.q0 * self.q0 - 0.5 + self.q3 * self.q3;

            // C. 计算估计重力与测得重力之间的叉积误差 [cite: 1.1.2]
            let halfex = ay_n * halfvz - az_n * halfvy;
            let halfey = az_n * halfvx - ax_n * halfvz;
            let halfez = ax_n * halfvy - ay_n * halfvx;

            // D. 计算并应用积分反馈 [cite: 1.1.2]
            if self.two_ki > 0.0 {
                self.integral_fbx += self.two_ki * halfex * dt;
                self.integral_fby += self.two_ki * halfey * dt;
                self.integral_fbz += self.two_ki * halfez * dt;
                gx_mod += self.integral_fbx;
                gy_mod += self.integral_fby;
                gz_mod += self.integral_fbz;
            } else {
                self.integral_fbx = 0.0;
                self.integral_fby = 0.0;
                self.integral_fbz = 0.0;
            }

            // E. 应用比例反馈 [cite: 1.1.2]
            gx_mod += self.two_kp * halfex;
            gy_mod += self.two_kp * halfey;
            gz_mod += self.two_kp * halfez;
        }

        // F. 积分四元数变率
        gx_mod *= 0.5 * dt;
        gy_mod *= 0.5 * dt;
        gz_mod *= 0.5 * dt;
        
        let qa = self.q0;
        let qb = self.q1;
        let qc = self.q2;
        
        self.q0 += -qb * gx_mod - qc * gy_mod - self.q3 * gz_mod;
        self.q1 += qa * gx_mod + qc * gz_mod - self.q3 * gy_mod;
        self.q2 += qa * gy_mod - qb * gz_mod + self.q3 * gx_mod;
        self.q3 += qa * gz_mod + qb * gy_mod - qc * gx_mod;

        // G. 归一化四元数，杜绝舍入误差累积
        let recip_norm = 1.0 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3).sqrt();
        self.q0 *= recip_norm;
        self.q1 *= recip_norm;
        self.q2 *= recip_norm;
        self.q3 *= recip_norm;
    }

    /// 从四元数状态机中提取当前的偏航角 (Yaw) [cite: 1.1.2]
    pub fn 获取偏航角_rad(&self) -> f32 {
        // 🛡️ 架构师 2026 级数学自愈：标准的四元数转偏航角公式。
        // 彻底修复将 C++ 飞控代码直译到 Rust 时发生的分子分母 4 倍比例失配。
        // 恢复为未简化的国际标准版（分子乘以 2.0，分母使用 1.0 - 2.0 * ...），实现绝对的数学对齐！ [cite: 1.1.2, 1.2.2]
        let numerator = 2.0 * (self.q1 * self.q2 + self.q0 * self.q3);
        let denominator = 1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3);
        numerator.atan2(denominator)
    }

    /// 🎯 2026 SOTA 反向烧录：根据外部校准后的 Yaw 角，强行校正并重置内部四元数！
    /// 将 1Hz 的视觉纠偏结果物理烧入 100Hz 的姿态小脑状态机中！
    pub fn 重置偏航角(&mut self, 目标偏航角_rad: f32) {
        let half_yaw = 目标偏航角_rad * 0.5;
        self.q0 = half_yaw.cos();
        self.q1 = 0.0;
        self.q2 = 0.0;
        self.q3 = half_yaw.sin();
        
        // 彻底清空积分器，防止由于状态突变产生积分饱和与剧烈振荡
        self.integral_fbx = 0.0;
        self.integral_fby = 0.0;
        self.integral_fbz = 0.0;
    }
}

/// 🛡️ 层级多传感器融合中心 (Hierarchical Sensor Fusion Core)
/// 管理小车的实时全局位姿，高低频双防线闭环
pub struct 层级传感器融合中心 {
    // 估计状态
    pub x: f32,
    pub y: f32,
    pub yaw_rad: f32,
    pub 过滤后的线速度: f32,

    // 子算法组件
    pub 马奥尼小脑: 马奥尼姿态估计器,
    pub 轮速滤波器: Pt1滤波器,
}

impl 层级传感器融合中心 {
    pub fn new(截止频率_hz: f32, kp: f32, ki: f32) -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw_rad: 0.0,
            过滤后的线速度: 0.0,
            马奥尼小脑: 马奥尼姿态估计器::new(kp, ki),
            轮速滤波器: Pt1滤波器::new(截止频率_hz),
        }
    }

    /// ⚡ [快通道 - 100Hz] 注入高频原始传感器信号，进行航位累积
    pub fn 注入高频传感器数据(
        &mut self,
        gx: f32, gy: f32, gz: f32, // 陀螺仪角速度 (rad/s)
        ax: f32, ay: f32, az: f32, // 加速度计 (m/s^2)
        裸轮速: f32,               // 轮速计瞬时线速度 (m/s)
        dt: f32,
    ) {
        // 1. PT1 滤波器扼杀轮速噪声 [cite: 1.1.2]
        self.过滤后的线速度 = self.轮速滤波器.更新(裸轮速, dt);

        // 2. 马奥尼互补滤波融合解算 Yaw 角 [cite: 1.1.2]
        self.马奥尼小脑.步进更新(gx, gy, gz, ax, ay, az, dt);
        self.yaw_rad = self.马奥尼小脑.获取偏航角_rad();

        // 3. 经典的非霍洛诺姆车辆死步累积 (航位推算) [cite: 1.2.5]
        self.x += self.过滤后的线速度 * self.yaw_rad.cos() * dt;
        self.y += self.过滤后的线速度 * self.yaw_rad.sin() * dt;
    }

    /// 🧠 [慢通道 - 1Hz] 注入 XFeat 视觉纠偏量，一键扼杀 $t^3$ 三次方漂移灾难！
    /// 采用 SOTA 级互补融合因子 (Complementary Blend Ratio) [cite: 1.2.5]
    pub fn 注入慢速视觉纠偏(
        &mut self,
        纠偏_dx: f32,
        纠偏_dy: f32,
        纠偏_dyaw: f32,
        融合因子_alpha: f32, // 范围 [0.0 - 1.0], 越接近 1.0 越信任视觉纠偏
    ) {
        // 1. 互补融合位置
        self.x += 融合因子_alpha * 纠偏_dx;
        self.y += 融合因子_alpha * 纠偏_dy;

        // 2. 互补融合航向角
        let mut 新_yaw = self.yaw_rad + 融合因子_alpha * 纠偏_dyaw;
        
        // 角度归一化限制在 [-PI, PI] 之间
        if 新_yaw > PI {
            新_yaw -= 2.0 * PI;
        } else if 新_yaw < -PI {
            新_yaw += 2.0 * PI;
        }
        self.yaw_rad = 新_yaw;

        // 3. 🎯 【并网联动核弹】将校正后的绝对 Yaw，反向强行烧录进马奥尼小脑的四元数状态机中！
        // 彻底切断上一周期的漂移惯性！
        self.马奥尼小脑.重置偏航角(self.yaw_rad);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hierarchical_sensor_fusion_flow() {
        // 🛡️ 架构师自愈：采用纯 Rust 静态字面量，避开 Python 字符串乘法，消除编译期错误并实现零内存分配！
        println!("\n================================================================================");
        println!("🛡️  [层级传感器融合测试] 启动 100Hz 物理小脑与 1Hz 视觉自愈级联测试...");
        println!("================================================================================");
        
        // 1. 初始化融合中心：轮速截止频率设为 15Hz，Mahony 比例增益 Kp=1.0, 积分增益 Ki=0.0
        let mut fusion = 层级传感器融合中心::new(15.0, 1.0, 0.0);
        let dt = 0.01; // 100Hz 控制节拍，10ms

        // 2. 模拟 100 帧 (1.0秒) 高频物理小脑的带噪步进推演
        let mut raw_vel_acc = 0.0;
        for step in 1..=100 {
            // 模拟带噪的轮速计原始输入 (期望速度为 0.3m/s，叠加由物理颠簸产生的强烈高频白噪声)
            let noise = ((step as f32 * 5.0).sin() * 0.05) + ((step as f32 * 11.0).cos() * 0.03);
            let raw_wheel_v = 0.3 + noise;
            raw_vel_acc += raw_wheel_v;

            // 模拟小车在水平地面上笔直行驶：Z轴重力 az = -9.81，绕 Z 轴角速度为 0.0
            let gx = 0.0;
            let gy = 0.0;
            let gz = 0.0;
            let ax = 0.0;
            let ay = 0.0;
            let az = -9.81;

            fusion.注入高频传感器数据(gx, gy, gz, ax, ay, az, raw_wheel_v, dt);
        }

        let avg_raw_vel = raw_vel_acc / 100.0;
        println!("   -> [100Hz 遥测] 原始轮速物理均值 : {:.4} m/s", avg_raw_vel);
        println!("   -> [100Hz 遥测] PT1 过滤后轮速结果: {:.4} m/s", fusion.过滤后的线速度);
        println!("   -> [100Hz 遥测] 累积航位推算位姿  : x={:.4}, y={:.4}, yaw={:.4} rad", fusion.x, fusion.y, fusion.yaw_rad);

        // 🛡️ 物理断言 1：PT1 滤波器必须成功阻断高频噪音，稳态值必须紧紧收敛在 0.3 m/s 附近 [cite: 1.1.2]
        assert!((fusion.过滤后的线速度 - 0.3).abs() < 0.02, "❌ PT1 滤波器未能将高频振动彻底阻断！");
        
        // 🛡️ 物理断言 2：小车前行 1 秒，全局位置 x 坐标应该大约推进了 0.3 米 [cite: 1.2.5]
        assert!((fusion.x - 0.3).abs() < 0.05, "❌ 航位推算发生严重累积漂移误差！");

        // 3. 模拟在 1.0 秒末端，DORA 神经总线送来了一发 1Hz 慢系统 XFeat 视觉重定位纠偏量
        // 假设视觉大脑判定当前小车由于地表打滑产生了漂移偏差，计算出纠偏修正量：
        let correction_dx = 0.02;   // 判定位置 x 发生漂移，需要向前补偿 2 厘米
        let correction_dy = -0.05;  // 判定位置 y 发生漂移，需要向右补偿 5 厘米
        let correction_dyaw = -0.1; // 判定偏航角发生漂移，需要向右转动 0.1 弧度

        let pre_x = fusion.x;
        let pre_y = fusion.y;
        let pre_yaw = fusion.yaw_rad;

        // 注入慢速纠偏，使用 2026 SOTA 级 0.8 的高信任互补因子
        fusion.注入慢速视觉纠偏(correction_dx, correction_dy, correction_dyaw, 0.8);

        println!("\n📸 [1Hz 视觉重定位触发] 全局位姿已被纠偏与重置：");
        println!("   -> 纠偏后位姿坐标: x={:.4}, y={:.4}, yaw={:.4} rad", fusion.x, fusion.y, fusion.yaw_rad);

        // 🛡️ 物理断言 3：验证 Complementary Blend 互补融合位置是否计算正确 [cite: 1.2.5]
        let expected_x = pre_x + 0.8 * correction_dx;
        let expected_y = pre_y + 0.8 * correction_dy;
        let expected_yaw = pre_yaw + 0.8 * correction_dyaw;

        assert!((fusion.x - expected_x).abs() < 1e-5, "❌ 视觉 X 位置纠偏发生计算错误！");
        assert!((fusion.y - expected_y).abs() < 1e-5, "❌ 视觉 Y 位置纠偏发生计算错误！");
        assert!((fusion.yaw_rad - expected_yaw).abs() < 1e-5, "❌ 视觉角度纠偏互补融和发生计算错误！");

        // 🛡️ 物理断言 4：【重中之重】验证马奥尼小脑内部的四元数状态机是否被反向重置并对齐！
        let quat_yaw = fusion.马奥尼小脑.获取偏航角_rad();
        assert!((quat_yaw - fusion.yaw_rad).abs() < 1e-5, "❌ 致命错误：马奥尼姿态估计器四元数状态机未同步反向烧录对齐！");

        println!("🏆 [层级传感器融合测试] 全部物理断言完美通过！大小脑级联完全符合预期。");
        println!("================================================================================\n");
    }
}