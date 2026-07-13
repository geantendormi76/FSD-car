# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "dora-rs==0.3.13",
#     "pyarrow>=14.0.0",
#     "numpy>=1.26.0"
# ]
# ///
"""
=================================================================
🗃️  NEXUS - SOTA 10维 platform-agnostic 时空状态舱记录仪 (动态对齐版)
设计哲学: 拒绝目标硬编码 | 动态订阅慢脑 Goal 广播 | 就地 PAM 动作归一化
=================================================================
"""
import os
import time
import numpy as np
import pyarrow as pa
from dora import Node

# 📐 SOTA 标定物理包线限幅 (PAM Limits - 强约束契约，全栈统一)
V_MAX = 0.80       # 最大巡航车速
KAPPA_MAX = 1.25   # 最大期望曲率 rad/m

def main():
    print("========================================================")
    print("🗃️  [黑匣子 SOTA] 10维跨具身自引力自洽数据记录仪启动...")
    print("特征契约: 1. 动态订阅目标  2. 写入即归一化 (PAM)  3. 瞬移自断分段")
    print("========================================================")
    
    dora_node = Node()
    os.makedirs("dataset", exist_ok=True)
    
    state_odom = [0.0, 0.0, 0.0]  # x, y, yaw
    last_odom = None
    
    # 🎯 核心解耦：初始化默认目标，随后高频接收慢脑广播的动态 Goal 更新
    current_goal_world = [0.52, 4.11] 
    
    run_id = 1
    csv_file = None
    record_count = 0
    
    def start_new_run(run_num):
        nonlocal csv_file, record_count
        if csv_file is not None:
            csv_file.close()
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = f"dataset/spice_run_{run_num:03d}_{timestamp_str}.csv"
        csv_file = open(filename, "w", encoding="utf-8")
        
        # 写入大一统 10 维自洽状态动作舱表头
        csv_file.write(
            "timestamp,"
            "odom_x,odom_y,odom_yaw,"
            "local_goal_x,local_goal_y,local_goal_dist,"
            "current_v,action_v_norm,action_kappa_norm\n"
        )
        csv_file.flush()
        record_count = 0
        print(f"\n📂 [黑匣子 SOTA] >>> 成功创建全新数据舱分段: Run {run_num:03d} <<<")
        print(f"   -> 存储路径: {filename}")
        return filename
        
    start_new_run(run_id)
    
    try:
        while True:
            event = dora_node.next(timeout=0.01)
            if event is not None:
                ev_type = event["type"]
                if ev_type == "INPUT":
                    ev_id = event["id"]
                    
                    # 🎯 核心重构：动态捕获慢脑广播的最新 Goal 坐标，杜绝空间数据撕裂
                    if ev_id == "human_prior":
                        prior_arr = event["value"].to_numpy()
                        if len(prior_arr) >= 2:
                            current_goal_world = [float(prior_arr[0]), float(prior_arr[1])]
                            
                    elif ev_id == "odometry":
                        data = event["value"].to_numpy()
                        if len(data) >= 3:
                            curr_x, curr_y, curr_yaw = float(data[0]), float(data[1]), float(data[2])
                            
                            # 检测仿真器重置 (拖动小车或 Reset) ➔ 瞬间无感分切文件
                            if last_odom is not None:
                                dist_jump = np.sqrt((curr_x - last_odom[0])**2 + (curr_y - last_odom[1])**2)
                                if dist_jump > 3.0:
                                    print(f"\n🔄 [黑匣子 SOTA] 探测到小车发生世界位置瞬移 (跨度: {dist_jump:.2f}米)！")
                                    print("   -> 开始保存上一段，无感切换至全新数据文件...")
                                    run_id += 1
                                    start_new_run(run_id)
                                    
                            state_odom = [curr_x, curr_y, curr_yaw]
                            last_odom = (curr_x, curr_y)
                            
                    elif ev_id == "control_cmd":
                        data = event["value"].to_numpy()
                        if len(data) >= 2:
                            cmd_v, cmd_w = float(data[0]), float(data[1])
                            
                            x_ego, y_ego, yaw_ego = state_odom[0], state_odom[1], state_odom[2]
                            
                            # 1. 📐 将动态接收到的目标进行车体局部坐标系投影
                            dx = current_goal_world[0] - x_ego
                            dy = current_goal_world[1] - y_ego
                            local_g_x = dx * np.cos(yaw_ego) + dy * np.sin(yaw_ego)
                            local_g_y = -dx * np.sin(yaw_ego) + dy * np.cos(yaw_ego)
                            local_g_dist = np.sqrt(local_g_x**2 + local_g_y**2)
                            
                            # 2. ⚡ PAM 对齐：就地折算为平台无关的目标速度比与目标曲率
                            action_v_norm = np.clip(cmd_v / V_MAX, 0.0, 1.0)
                            
                            # 避免静止时曲率除零奇异点，使用 eps = 0.01
                            kappa = cmd_w / max(abs(cmd_v), 0.01)
                            action_kappa_norm = np.clip(kappa / KAPPA_MAX, -1.0, 1.0)
                            
                            current_time = time.time()
                            
                            # 3. 10 维全状态舱无损物理落盘
                            csv_file.write(
                                f"{current_time:.4f},"
                                f"{x_ego:.4f},{y_ego:.4f},{yaw_ego:.4f},"
                                f"{local_g_x:.4f},{local_g_y:.4f},{local_g_dist:.4f},"
                                f"{cmd_v:.4f},{action_v_norm:.4f},{action_kappa_norm:.4f}\n"
                            )
                            record_count += 1
                            if record_count % 100 == 0:
                                print(f"  💾 [数据舱 {run_id:03d}] 累计记录 {record_count} 帧自洽数据... "
                                      f"(线速比: {action_v_norm:.2f} | 期望曲率比: {action_kappa_norm:+.2f})")
                                      
                elif ev_type == "STOP":
                    print("\n🛑 [黑匣子 SOTA] 接收到 DORA 停止信号。")
                    break
    except Exception as e:
        print(f"\n❌ ERROR in BlackBox Logger: {e}")
    finally:
        if csv_file is not None:
            csv_file.close()
        print(f"🔌 [黑匣子 SOTA] 数据采集器安全下线。共完成 {run_id} 段高能数据采集！")

if __name__ == "__main__":
    main()
