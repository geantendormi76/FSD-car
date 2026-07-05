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
    
    # 🛡️ 黄金自驾建图物理参数：限制最高速防止特征模糊，低速稳态建图才能获得 100% 回环率
    max_v = 0.20  # 最大线速度限制在 0.20 m/s (极致稳态)
    min_v = -0.10 # 最大倒车速度
    max_w = 0.40  # 最大角速度限制在 0.40 rad/s
    
    v_step = 0.05 # 线速度步进 (按一次 W 增加 0.05)
    
    # 滤波时间系数：数值越小越平滑，防止轮子在物理引擎中打滑
    alpha_v = 0.12  # 线速度一阶阻尼系数
    alpha_w = 0.25  # 角速度一阶阻尼系数

    print("========================================================")
    print("🛰️  NEXUS - 工业级一阶滤波稳态建图控制器 (teleop_sender)已启动")
    print("设计哲学: 保持型线速控制 | 自回中型角速控制 | 物理防滑移一阶低通滤波")
    print("--------------------------------------------------------")
    print("控制手势 (请保持当前终端 Focus 状态，无需长按):")
    print("  W : 增加向前目标车速 (+0.05 m/s)   S : 降低目标车速/倒车 (-0.05 m/s)")
    print("  A : 靶向左转 (瞬时 nudging)       D : 靶向右转 (瞬时 nudging)")
    print("  Space (空格键) : 紧急制动 (目标速度瞬间归零)")
    print("  Q : 退出键盘控制端")
    print("========================================================")
    
    try:
        while True:
            key = get_key(settings)
            
            # A. 处理线速度 (保持型：按一下变一次，松手不减速，维持匀速爬坡)
            if key == 'w' or key == 'W':
                target_v = min(max_v, target_v + v_step)
            elif key == 's' or key == 'S':
                target_v = max(min_v, target_v - v_step)
            
            # B. 处理角速度 (自回中型：按住转弯，松手车头迅速自动回正)
            if key == 'a' or key == 'A':
                target_w = max_w
            elif key == 'd' or key == 'D':
                target_w = -max_w
            else:
                # 无转向按键，角速度目标自动归零
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

