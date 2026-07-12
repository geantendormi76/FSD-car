use core_control::预测控制求解器;
use core_control::solver::凸包走廊;
use std::time::Instant;

fn 生成局部凸包走廊(当前x: f64, 当前y: f64, 目标x: f64, 目标y: f64) -> 凸包走廊 {
    let dx = 目标x - 当前x;
    let dy = 目标y - 当前y;
    let dist = (dx * dx + dy * dy).sqrt().max(0.01);
    let dir_x = dx / dist;
    let dir_y = dy / dist;
    let norm_x = -dir_y;
    let norm_y = dir_x;
    let half_width = 0.6; 
    let length_margin = 0.5; 
    let mut a_mat = [[0.0; 2]; 4];
    let mut b_vec = [0.0; 4];
    a_mat[0] = [norm_x, norm_y];
    b_vec[0] = norm_x * (当前x + half_width * norm_x) + norm_y * (当前y + half_width * norm_y);
    a_mat[1] = [-norm_x, -norm_y];
    b_vec[1] = -norm_x * (当前x - half_width * norm_x) - norm_y * (当前y - half_width * norm_y);
    a_mat[2] = [dir_x, dir_y];
    b_vec[2] = dir_x * (目标x + dir_x * length_margin) + dir_y * (目标y + dir_y * length_margin);
    a_mat[3] = [-dir_x, -dir_y];
    b_vec[3] = -dir_x * (当前x - dir_x * length_margin) - dir_y * (当前y - dir_y * length_margin);
    凸包走廊 { a_mat, b_vec }
}

fn main() {
    println!("========================================================");
    println!("🛰️ NEXUS - 规控一元化台架验证 (Convex NMPC Bench) 启动");
    println!("========================================================");
    let mut 规控大脑 = 预测控制求解器::new().expect("❌ NMPC 求解器初始化失败！");
    let 当前线速度 = 0.5_f64; 
    let 工况列表 = vec![
        ("工况 A: 纯净直行 (无障碍物)", 0.0_f64),
        ("工况 B: 右侧遇险 (红毯向左形变)", 1.5_f64),  
        ("工况 C: 左侧遇险 (红毯向右形变)", -1.5_f64), 
    ];
    println!("{:<25} | {:<7} | {:<7} | {:<8} | {}", "测试工况", "v_cmd", "w_cmd", "耗时(us)", "诊断结论");
    println!("--------------------------------------------------------------------------------");
    for (工况名称, 横向偏移量) in 工况列表 {
        规控大脑.设置当前状态(0.0, 0.0, 0.0, 当前线速度).expect("状态注入失败");
        let 原始目标_x = 3.0_f64; 
        let 原始目标_y = 0.0_f64;
        let 形变后目标_y = 原始目标_y + 横向偏移量;
        let target_distance = (原始目标_x * 原始目标_x + 形变后目标_y * 形变后目标_y).sqrt();
        let shifted_target_yaw = 形变后目标_y.atan2(原始目标_x).clamp(-0.35, 0.35);
        let target_velocity = 0.80_f64.min(target_distance * 0.5);
        let d_ff = 原始目标_x / 3.0;
        for k in 0..=20 {
            let t = (k as f64) / 20.0;
            let ref_x = 3.0 * (1.0 - t).powi(2) * t * d_ff + 3.0 * (1.0 - t) * t.powi(2) * (原始目标_x - d_ff * shifted_target_yaw.cos()) + t.powi(3) * 原始目标_x;
            let ref_y = 3.0 * (1.0 - t) * t.powi(2) * (形变后目标_y - d_ff * shifted_target_yaw.sin()) + t.powi(3) * 形变后目标_y;
            let ref_yaw = shifted_target_yaw * (t * t * (3.0 - 2.0 * t)); 
            规控大脑.设置参考轨迹点(k, ref_x, ref_y, ref_yaw, target_velocity).unwrap();
        }
        let corridor = 生成局部凸包走廊(0.0, 0.0, 原始目标_x, 形变后目标_y);
        规控大脑.设置安全走廊硬约束(&corridor).unwrap();
        let 开始时间 = Instant::now();
        
        // 🛡️ 对齐接口：解包抛弃不使用的状态码
        let (v_cmd, w_cmd, _status) = 规控大脑.求解最优控制量(当前线速度).expect("❌ 求解器发散崩溃！");
        let 耗时_微秒 = 开始时间.elapsed().as_micros();
        assert!(耗时_微秒 < 2000, "❌ 求解超时！");
        if 横向偏移量 > 0.0 {
            assert!(w_cmd > 0.1, "❌ 转向极性错误！");
        } else if 横向偏移量 < 0.0 {
            assert!(w_cmd < -0.1, "❌ 转向极性错误！");
        }
        println!("{:<25} | {:<7.3} | {:<7.3} | {:<8} | {}", 工况名称, v_cmd, w_cmd, 耗时_微秒, "✅ 编译成功");
    }
}
