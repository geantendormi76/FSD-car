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
    
    v = 0.0  # 线速度 (m/s)
    w = 0.0  # 角速度 (rad/s)
    
    v_step = 0.05
    w_step = 0.1
    max_v = 0.3
    max_w = 0.6
    
    print("========================================================")
    print("🛰️  NEXUS - 本地 TTY 键盘遥控发送端已启动")
    print("控制手势 (请保持当前键盘控制台前台置顶 Focus 状态):")
    print("  W : 推进加速          S : 减速/后退")
    print("  A : 左转弯            D : 右转弯")
    print("  Space (空格键) : 紧急停障")
    print("  Q : 退出键盘控制端")
    print("========================================================")
    
    try:
        while True:
            key = get_key(settings)
            
            if key == 'w' or key == 'W':
                v = 0.25  # 按住 W：直接靶向推进航巡速度 (0.25 m/s)
            elif key == 's' or key == 'S':
                v = -0.15 # 按住 S：倒车速度 (-0.15 m/s)
            elif key == 'a' or key == 'A':
                w = 0.45  # 按住 A：靶向左转角速度 (0.45 rad/s)
            elif key == 'd' or key == 'D':
                w = -0.45 # 按住 D：靶向右转角速度 (-0.45 rad/s)
            elif key == ' ':
                v = 0.0
                w = 0.0
            elif key == 'q' or key == 'Q':
                print("\n🛑 退出发送端。")
                break
            else:
                # 🛡️ 物理主动衰减：无任何按键按下，代表手部已松开
                # 摩擦力线性衰减线速度，模拟惯性滑行停止
                if v > 0:
                    v = max(0.0, v - 0.04)
                elif v < 0:
                    v = min(0.0, v + 0.04)
                # 偏航角速度极速衰减，实现“车头自回中”
                if w > 0:
                    w = max(0.0, w - 0.12)
                elif w < 0:
                    w = min(0.0, w + 0.12)
            
            # 持续向 DORA 广播当前的运动状态（维持 DORA 50Hz 物理控制节拍，消除物理滑移）
            print(f"\r[遥控指令] 线速度 cmd_v: {v:.2f} m/s | 角速度 cmd_w: {w:.2f} rad/s    ", end="", flush=True)
            packed_data = struct.pack('ff', v, w)
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

