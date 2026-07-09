#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
"""
=================================================================
🗃️ NEXUS - 3.3 阶段：人类黄金数据黑匣子 (Expert Data Logger)
设计哲学: 异步状态对齐 | 零拷贝内存读取 | 高频 CSV 落盘
=================================================================
"""
import os
import time
import pyarrow as pa
from dora import Node

def main():
    print("========================================================")
    print("🗃️ [黑匣子] 人类黄金数据采集节点 (Expert Logger) 已启动")
    print("正在监听 DORA 神经总线，等待人类遥控指令接入...")
    print("========================================================")
    
    dora_node = Node()
    
    # 状态缓存金库 (用于异步数据对齐)
    state_odom = [0.0, 0.0, 0.0]  # x, y, yaw
    state_force = [0.0, 0.0]      # f_x, f_y
    
    # 创建数据集保存目录
    os.makedirs("dataset", exist_ok=True)
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    csv_filename = f"dataset/spice_human_data_{timestamp_str}.csv"
    
    # 初始化 CSV 表头
    with open(csv_filename, "w", encoding="utf-8") as f:
        f.write("timestamp,odom_x,odom_y,odom_yaw,force_x,force_y,cmd_v,cmd_w\n")
    
    print(f"✅ 数据将实时落盘至: {csv_filename}")
    
    record_count = 0
    
    try:
        while True:
            # 极速轮询 DORA 事件
            event = dora_node.next(timeout=0.01)
            if event is not None:
                ev_type = event["type"]
                if ev_type == "INPUT":
                    ev_id = event["id"]
                    
                    # 1. 缓存最新的物理里程计状态
                    if ev_id == "odometry":
                        data = event["value"].to_numpy()
                        if len(data) >= 3:
                            state_odom = [data[0], data[1], data[2]]
                            
                    # 2. 缓存最新的青蛙眼动态斥力状态
                    elif ev_id == "obstacle_force":
                        data = event["value"].to_numpy()
                        if len(data) >= 2:
                            state_force = [data[0], data[1]]
                            
                    # 3. 🎯 核心触发器：监听到人类下发控制指令，立刻打包当前状态落盘！
                    elif ev_id == "control_cmd":
                        data = event["value"].to_numpy()
                        if len(data) >= 2:
                            cmd_v, cmd_w = data[0], data[1]
                            current_time = time.time()
                            
                            # 写入 CSV
                            with open(csv_filename, "a", encoding="utf-8") as f:
                                f.write(f"{current_time:.4f},{state_odom[0]:.4f},{state_odom[1]:.4f},{state_odom[2]:.4f},"
                                        f"{state_force[0]:.4f},{state_force[1]:.4f},{cmd_v:.4f},{cmd_w:.4f}\n")
                            
                            record_count += 1
                            if record_count % 100 == 0:
                                print(f"💾 [黑匣子] 已录制 {record_count} 帧黄金数据... (当前车速: {cmd_v:.2f} m/s)")
                                
                elif ev_type == "STOP":
                    print("\n🛑 [黑匣子] 收到 DORA 停止信号，停止录制。")
                    break
                    
    except Exception as e:
        print(f"\n❌ [黑匣子] 发生异常: {e}")
    finally:
        print(f"🔌 [黑匣子] 录制结束。共采集 {record_count} 帧数据，保存在 {csv_filename}。")

if __name__ == "__main__":
    main()