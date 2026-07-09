#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
import sys
import select
import termios
import tty
import time
import socket
import struct
import os

def get_key(settings):
    # 此处运行于前台物理交互终端，具备合法的物理 TTY 设备权限
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    # 建立 UDP 网络客户端
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest_addr = ("127.0.0.1", 5005)
    
    # 🎯 智驾重塑控制器：目标速度与平滑滤波器状态
    target_v = 0.0  # 目标线速度 (保持型)
    target_w = 0.0  # 目标角速度 (自回中型)
    
    current_v = 0.0  # 滤波后的当前实际线速度
    current_w = 0.0  # 滤波后的当前实际角速度
    
    # 🏎️ 3.3阶段 Spiced RL 专属参数：全包线极限驾驶解锁
    max_v = 1.00  # 解除封印：最高线速度飙升至 1.0 m/s
    min_v = -0.30 # 最大倒车速度
    max_w = 1.00  # 极限打舵角速度 1.0 rad/s
    acc_step = 0.08 # 油门灵敏度
    brake_step = 0.15 # 刹车灵敏度 (刹车比油门猛，符合真实车辆物理)
    coast_decay = 0.03 # 松开油门时的自然滑行阻力 (Coasting)
    
    # 滤波时间系数：调高响应度，让老司机的微操更跟手，同时保留防滑移底线
    alpha_v = 0.25  
    alpha_w = 0.40  
    print("========================================================")
    print("🏎️  NEXUS - 极限驾驶数据采集终端 (Spiced Teleop) 已启动")
    print("设计哲学: 赛车级油门刹车逻辑 | 阻尼滑行 | 毫秒级跟手转向")
    print("--------------------------------------------------------")
    print("控制手势 (请保持当前终端 Focus 状态，长按生效):")
    print("  [长按 W] : 踩油门加速 (最高 1.0 m/s)  [松开 W] : 自然滑行减速")
    print("  [长按 S] : 踩刹车/倒车                [松开 S] : 停止倒车")
    print("  [长按 A/D]: 极限打舵转向              [松开 A/D]: 方向盘自动回正")
    print("  [Space]  : 紧急抱闸 (瞬间锁死)")
    print("  [ Q ]    : 退出采集")
    print("========================================================")
    try:
        while True:
            key = get_key(settings)
            
            # A. 处理线速度 (赛车踏板逻辑：按住加速，松开滑行，S键刹车)
            if key == 'w' or key == 'W':
                target_v = min(max_v, target_v + acc_step)
            elif key == 's' or key == 'S':
                target_v = max(min_v, target_v - brake_step)
            else:
                # 模拟自然滑行阻力 (Coasting)
                if target_v > 0:
                    target_v = max(0.0, target_v - coast_decay)
                elif target_v < 0:
                    target_v = min(0.0, target_v + coast_decay)

            # B. 处理角速度 (方向盘逻辑：按住打死，松开瞬间回正)
            if key == 'a' or key == 'A':
                target_w = max_w
            elif key == 'd' or key == 'D':
                target_w = -max_w
            else:
                target_w = 0.0
            
            # C. 处理紧急刹车
            if key == ' ':
                target_v = 0.0
                target_w = 0.0
                current_v = 0.0 # 强制物理锁死
                current_w = 0.0
                
            elif key == 'q' or key == 'Q':
                print("\n🛑 退出发送端。")
                break
            
            # D. 核心物理自愈：一阶低通滤波器 (Ramp Filter)
            # 消除台阶响应（Step Response）带来的电机瞬间输出冲击，
            # 100% 杜绝轮胎在 PhysX 物理沙盘上原地打滑（Slip）导致里程计漂移！
            current_v += alpha_v * (target_v - current_v)
            current_w += alpha_w * (target_w - current_w)
            
            # 极小值消隐
            if abs(current_v) < 0.005: current_v = 0.0
            if abs(current_w) < 0.005: current_w = 0.0

            # 持续向 DORA 广播当前的运动状态 (维持 DORA 50Hz 物理控制节拍，消除物理滑移)
            print(
                f"\r[NEXUS 遥测] 目标设定 -> v: {target_v:.2f} m/s, w: {target_w:.2f} rad/s | "
                f"实际输出 -> v_cmd: {current_v:.2f} m/s, w_cmd: {current_w:.2f} rad/s    ", 
                end="", flush=True
            )
            
            packed_data = struct.pack('ff', current_v, current_w)
            sock.sendto(packed_data, dest_addr)
            time.sleep(0.02) # 50Hz 物理闭环节拍
            
    except Exception as e:
        print(f"\n❌ [发送端] 发生错误: {e}")
    finally:
        # 退出前向 DORA 发送清零刹车数据
        packed_data = struct.pack('ff', 0.0, 0.0)
        sock.sendto(packed_data, dest_addr)
        sock.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n🔌 [发送端] 控制套接字关闭，退出交互。")

if __name__ == "__main__":
    main()

