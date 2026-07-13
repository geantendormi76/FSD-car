// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
/*
=================================================================
🧠 [NEXUS 控制抽象层] 运动控制器契约与神经解耦路由器中心 (v3.6 - 自愈版)
设计哲学: 补齐 Sync 线程防线 | 路由器强类型分流 | 掐断原始 Arrow 侵入
=================================================================
*/
pub mod ffi;
pub mod solver;
pub mod sensor_fusion;

pub use solver::预测控制求解器;
pub use sensor_fusion::层级传感器融合中心;

use dora_node_api::Event;
use dora_node_api::arrow::array::{Float32Array, UInt8Array};

// 🛡️ 极速自愈：为底层包装了 raw 指针的 C-FFI 求解器手动实现 Sync，解除多线程安全屏障
unsafe impl Sync for solver::预测控制求解器 {}

/// 🛡️ 统称车辆运动控制器 Trait（控制器的生命周期只需要 Send 契约进行线程转移，无需 Sync 共享）
pub trait 车辆运动控制器: Send {
    type 状态;      // (x, y, yaw, v)
    type 轨迹;      // (ref_x, ref_y, ref_yaw, ref_v)
    type 障碍;      // 动态障碍物
    type 指令;      // (v_cmd, w_cmd)

    fn 设置当前状态(&mut self, state: &Self::状态) -> Result<(), String>;
    fn 设置参考轨迹点(&mut self, stage: i32, ref_point: &Self::轨迹) -> Result<(), String>;
    fn 设置动态障碍物硬约束(&mut self, obstacles: &[Self::障碍]) -> Result<(), String>;
    fn 求解最优控制量(&mut self, 当前线速度: f64) -> Result<Self::指令, String>;
}

// -------------------------------------------------------------------------
// 🛰️ MPC 求解器 Trait 契约并网实现
// -------------------------------------------------------------------------
impl 车辆运动控制器 for solver::预测控制求解器 {
    type 状态 = (f64, f64, f64, f64);
    type 轨迹 = (f64, f64, f64, f64);
    type 障碍 = solver::动态障碍物;
    type 指令 = (f64, f64);

    fn 设置当前状态(&mut self, state: &Self::状态) -> Result<(), String> {
        self.设置当前状态(state.0, state.1, state.2, state.3)
    }

    fn 设置参考轨迹点(&mut self, stage: i32, ref_point: &Self::轨迹) -> Result<(), String> {
        self.设置参考轨迹点(stage, ref_point.0, ref_point.1, ref_point.2, ref_point.3)
    }

    fn 设置动态障碍物硬约束(&mut self, obstacles: &[Self::障碍]) -> Result<(), String> {
        self.设置动态障碍物硬约束(obstacles)
    }

    fn 求解最优控制量(&mut self, 当前线速度: f64) -> Result<Self::指令, String> {
        self.求解最优控制量(当前线速度)
    }
}

// -------------------------------------------------------------------------
// 🛰️ 神经事件路由器：将原始异步 Dora 事件安全翻译并路由为自驾强类型信号枚举
// -------------------------------------------------------------------------

pub enum 智能自驾信号载荷 {
    物理里程计 { x: f64, y: f64, yaw: f64 },
    避障斥力场 { fx: f64, fy: f64 },
    慢脑自引力领航 { goal_x: f64, goal_y: f64, goal_yaw: f64 },
    实相鸟瞰图(Vec<u8>),
    神经反射时间(f32),
    系统下线信号,
    未校准信号,
}

pub struct 神经事件路由器;

impl 神经事件路由器 {
    /// 统一网关：阻断 Arrow 原始解包，强类型保护高层规控业务
    pub fn 分流路由事件(event: Event) -> Result<智能自驾信号载荷, String> {
        match event {
            Event::Input { id, data, .. } => {
                let id_str = id.as_str();
                match id_str {
                    "odometry" => {
                        let odom_array = data.as_any()
                            .downcast_ref::<Float32Array>()
                            .ok_or_else(|| "Failed to cast odometry to Float32Array".to_string())?;
                        if odom_array.len() >= 3 {
                            Ok(智能自驾信号载荷::物理里程计 {
                                x: odom_array.value(0) as f64,
                                y: odom_array.value(1) as f64,
                                yaw: odom_array.value(2) as f64,
                            })
                        } else {
                            Err("Odometry array size mismatch".to_string())
                        }
                    },
                    "obstacle_force" => {
                        let force_array = data.as_any()
                            .downcast_ref::<Float32Array>()
                            .ok_or_else(|| "Failed to cast obstacle_force to Float32Array".to_string())?;
                        if force_array.len() >= 2 {
                            Ok(智能自驾信号载荷::避障斥力场 {
                                fx: force_array.value(0) as f64,
                                fy: force_array.value(1) as f64,
                            })
                        } else {
                            Err("Obstacle force array size mismatch".to_string())
                        }
                    },
                    "human_prior" => {
                        let prior_array = data.as_any()
                            .downcast_ref::<Float32Array>()
                            .ok_or_else(|| "Failed to cast human_prior to Float32Array".to_string())?;
                        if prior_array.len() >= 3 {
                            Ok(智能自驾信号载荷::慢脑自引力领航 {
                                goal_x: prior_array.value(0) as f64,
                                goal_y: prior_array.value(1) as f64,
                                goal_yaw: prior_array.value(2) as f64,
                            })
                        } else {
                            Err("Human prior array size mismatch".to_string())
                        }
                    },
                    "bev_grid" => {
                        let grid_array = data.as_any()
                            .downcast_ref::<UInt8Array>()
                            .ok_or_else(|| "Failed to cast bev_grid to UInt8Array".to_string())?;
                        Ok(智能自驾信号载荷::实相鸟瞰图(grid_array.values().to_vec()))
                    },
                    "ttc" => {
                        let ttc_array = data.as_any()
                            .downcast_ref::<Float32Array>()
                            .ok_or_else(|| "Failed to cast ttc to Float32Array".to_string())?;
                        if ttc_array.len() >= 1 {
                            Ok(智能自驾信号载荷::神经反射时间(ttc_array.value(0)))
                        } else {
                            Err("TTC array empty".to_string())
                        }
                    },
                    _ => Ok(智能自驾信号载荷::未校准信号),
                }
            },
            Event::Stop(_) => Ok(智能自驾信号载荷::系统下线信号),
            _ => Ok(智能自驾信号载荷::未校准信号),
        }
    }
}

/// 🧠 SOTA: Physics-Aware Action Mapping decoder
pub fn decode_platform_agnostic_actions(a_vel: f64, a_kappa: f64) -> (f64, f64, f64) {
    let v_min: f64 = -0.30; 
    let v_max: f64 = 0.80;  
    let kappa_max: f64 = 1.25;
    let w_max: f64 = 1.00;  
    let v_des = if a_vel >= 0.0 {
        a_vel.clamp(0.0, 1.0) * v_max
    } else {
        a_vel.clamp(-1.0, 0.0) * v_min.abs()
    };
    let kappa = a_kappa.clamp(-1.0, 1.0) * kappa_max;
    let w_ref = if v_des.abs() < 0.05 {
        a_kappa.clamp(-1.0, 1.0) * w_max
    } else {
        kappa * v_des
    };
    (v_des, kappa, w_ref)
}
