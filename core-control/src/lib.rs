// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
pub mod ffi;
pub mod solver;

// 🛡️ 架构师 2026 级并网：注册高鲁棒性层级传感器融合算法库
pub mod sensor_fusion;

// 🛡️ 向上层暴露安全的规控接口
pub use solver::预测控制求解器;
pub use sensor_fusion::层级传感器融合中心;