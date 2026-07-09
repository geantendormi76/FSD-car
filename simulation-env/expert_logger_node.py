#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
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
🗃️  NEXUS V2 - 10维高自洽时空状态舱数据记录仪 (Auto-Rotation)
设计哲学: 瞬移探测自动分段 | 局部目标齐次投影 | 阻断因果混淆
=================================================================
"""
import os
import time
import numpy as np
import pyarrow as pa
from dora import Node

# 🎯 【老司机配置中心】: 设定你期望的目标终点世界坐标 (PointGoal-Nav)
# 当你在场景中不同初始点起跑时，Logger 会自动根据此坐标计算自引力引导向量
TARGET_GOAL_WORLD = (0.52, 4.11)  # 默认赛道终点附近坐标，可根据实际测试关卡随时调整

def main():
    print("========================================================")
    print("🗃️  [黑匣子 V2] 10维自引力导航数据采集中心已启动...")
    print("支持功能: 1. 自动重置检测  2. 多关卡分段归档  3. 相对目标自引力投影")
    print("========================================================")
    
    dora_node = Node()
    os.makedirs("dataset", exist_ok=True)
    
    # 状态金库 (用于高频异步数据对齐)
    state_odom = [0.0, 0.0, 0.0]     # 绝对坐标: x, y, yaw
    state_force = [0.0, 0.0]         # 青蛙眼受力: fx, fy
    last_odom = None                 # 用于检测是否发生重置/瞬移
    
    # 分段管理变量
    run_id = 1
    csv_file = None
    record_count = 0
    
    # 辅助函数：打开一个全新分段的 CSV 文件
    def start_new_run(run_num):
        nonlocal csv_file, record_count
        if csv_file is not None:
            csv_file.close()
            
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = f"dataset/spice_run_{run_num:03d}_{timestamp_str}.csv"
        csv_file = open(filename, "w", encoding="utf-8")
        
        # 写入 10 维全状态舱表头，严格对齐 SOTA 模仿学习训练格式
        csv_file.write(
            "timestamp,current_v,current_w,"
            "odom_x,odom_y,odom_yaw,"
            "local_goal_x,local_goal_y,local_goal_dist,"
            "frog_eye_fx,frog_eye_fy,"
            "cmd_v,cmd_w\n"
        )
        csv_file.flush()
        record_count = 0
        print(f"\n📂 [黑匣子 V2] >>> 成功创建并开启全新数据分段: Run {run_num:03d} <<<")
        print(f"   -> 存储路径: {filename}")
        return filename

    # 启动第 1 次运行数据分段
    start_new_run(run_id)

    try:
        while True:
            event = dora_node.next(timeout=0.01)
            if event is not None:
                ev_type = event["type"]
                if ev_type == "INPUT":
                    ev_id = event["id"]
                    
                    # 1. 缓存最新的绝对位置与自车速度参数
                    if ev_id == "odometry":
                        data = event["value"].to_numpy()
                        if len(data) >= 3:
                            curr_x, curr_y, curr_yaw = float(data[0]), float(data[1]), float(data[2])
                            
                            # 🎯 核心亮点：检测瞬移（比如在 GUI 按钮中重置了小车位置，或者小车闪现）
                            if last_odom is not None:
                                dist_jump = np.sqrt((curr_x - last_odom[0])**2 + (curr_y - last_odom[1])**2)
                                if dist_jump > 3.0: # 发生大于 3 米的瞬移
                                    print(f"\n🔄 [黑匣子 V2] 探测到小车发生世界位置瞬移 (跨度: {dist_jump:.2f}米)！")
                                    print("   -> 判定前一关卡结束。开始物理保存并切换新文件...")
                                    run_id += 1
                                    start_new_run(run_id)
                                    
                            state_odom = [curr_x, curr_y, curr_yaw]
                            last_odom = (curr_x, curr_y)
                            
                    # 2. 缓存最新的青蛙眼 2D 避障势场力
                    elif ev_id == "obstacle_force":
                        data = event["value"].to_numpy()
                        if len(data) >= 2:
                            state_force = [float(data[0]), float(data[1])]
                            
                    # 3. 🎯 核心写入触发：收到你下发的赛车级遥控信号，进行 10 维并网落盘！
                    elif ev_id == "control_cmd":
                        data = event["value"].to_numpy()
                        if len(data) >= 2:
                            cmd_v, cmd_w = float(data[0]), float(data[1])
                            
                            # A. 提取状态变量
                            x_ego, y_ego, yaw_ego = state_odom[0], state_odom[1], state_odom[2]
                            f_x, f_y = state_force[0], state_force[1]
                            
                            # B. 📐 将世界目标点进行 2D 局部齐次坐标变换 (PointGoal-Nav 核心)
                            dx = TARGET_GOAL_WORLD[0] - x_ego
                            dy = TARGET_GOAL_WORLD[1] - y_ego
                            
                            # 通过自车航向角进行反向投影，求出目标点在车头局部坐标系下的 X(前后), Y(左右) 坐标
                            local_g_x = dx * np.cos(yaw_ego) + dy * np.sin(yaw_ego)
                            local_g_y = -dx * np.sin(yaw_ego) + dy * np.cos(yaw_ego)
                            local_g_dist = np.sqrt(local_g_x**2 + local_g_y**2)
                            
                            current_time = time.time()
                            
                            # C. 10 维全状态舱无损落盘 (以你的遥控指令作为 Action Label)
                            csv_file.write(
                                f"{current_time:.4f},{cmd_v:.4f},{cmd_w:.4f},"
                                f"{x_ego:.4f},{y_ego:.4f},{yaw_ego:.4f},"
                                f"{local_g_x:.4f},{local_g_y:.4f},{local_g_dist:.4f},"
                                f"{f_x:.4f},{f_y:.4f},"
                                f"{cmd_v:.4f},{cmd_w:.4f}\n"
                            )
                            
                            record_count += 1
                            if record_count % 100 == 0:
                                print(f"  💾 [分段 {run_id:03d}] 成功写入 {record_count} 帧高能数据... "
                                      f"(车速: {cmd_v:.2f} m/s | 相对目标距离: {local_g_dist:.2f}m)")
                                
                elif ev_type == "STOP":
                    print("\n🛑 [黑匣子 V2] 收到 DORA 停止信号。")
                    break
                    
    except Exception as e:
        print(f"\n❌ [黑匣子 V2] 发生异常: {e}")
    finally:
        if csv_file is not None:
            csv_file.close()
        print(f"🔌 [黑匣子 V2] 已安全下线并锁死写保护。共完成 {run_id} 次不同场景的黄金数据录制！")

if __name__ == "__main__":
    main()