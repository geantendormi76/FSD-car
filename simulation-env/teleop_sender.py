import sys
import select
import termios
import tty
import time
import socket
import struct

def get_key(settings):
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
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest_addr = ("127.0.0.1", 5005)
    
    target_v = 0.0  
    target_w = 0.0  
    current_v = 0.0  
    current_w = 0.0  
    
    # 🏎️ Spiced 2026 级全包线极限标定参数
    max_v = 1.00  
    min_v = 0.00 
    max_w = 1.00  
    acc_step = 0.08 
    brake_step = 0.15 
    coast_decay = 0.03 
    
    alpha_v = 0.25  
    alpha_w = 0.40  
    
    print("========================================================")
    print("🏎️  NEXUS - Platform-Agnostic Human Teleop Terminal")
    print("设计哲学: 赛车级踏板阻尼 | 自动低通滤波 | 实时曲率映射")
    print("========================================================")
    print("控制指令 (请长按以激活连续指令流):")
    print("  [ W / S ] : 踩油门加速 / 踩刹车减速")
    print("  [ A / D ] : 向左 / 向右打舵")
    print("  [Space]   : 紧急抱闸锁死")
    print("  [ Q ]     : 安全退出")
    print("========================================================")
    
    try:
        while True:
            key = get_key(settings)
            
            if key == 'w' or key == 'W':
                target_v = min(max_v, target_v + acc_step)
            elif key == 's' or key == 'S':
                target_v = max(min_v, target_v - brake_step)
            else:
                if target_v > 0:
                    target_v = max(0.0, target_v - coast_decay)
                elif target_v < 0:
                    target_v = min(0.0, target_v + coast_decay)
                    
            if key == 'a' or key == 'A':
                target_w = max_w
            elif key == 'd' or key == 'D':
                target_w = -max_w
            else:
                target_w = 0.0
                
            if key == ' ':
                target_v = 0.0
                target_w = 0.0
                current_v = 0.0
                current_w = 0.0
            elif key == 'q' or key == 'Q':
                print("\n🛑 Safe Teleop Session Terminated.")
                break
                
            current_v += alpha_v * (target_v - current_v)
            current_w += alpha_w * (target_w - current_w)
            
            if abs(current_v) < 0.005: current_v = 0.0
            if abs(current_w) < 0.005: current_w = 0.0
            
            # 计算符合 Sim2Real-AD 标准的实时期望路径曲率 kappa = w / v
            kappa = current_w / max(abs(current_v), 0.01)
            
            print(
                f"\r[NEXUS 遥测] 目标设定 -> v: {target_v:.2f} m/s | "
                f"实际输出 -> v_cmd: {current_v:.2f} m/s, w_cmd: {current_w:.2f} rad/s | "
                f"期望曲率 kappa: {kappa:+.3f} rad/m    ", 
                end="", flush=True
            )
            
            packed_data = struct.pack('ff', current_v, current_w)
            sock.sendto(packed_data, dest_addr)
            time.sleep(0.02) # 50Hz 物理控制周期
            
    except Exception as e:
        print(f"\n❌ ERROR in Teleop: {e}")
    finally:
        packed_data = struct.pack('ff', 0.0, 0.0)
        sock.sendto(packed_data, dest_addr)
        sock.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n🔌 Socket cleanly closed. Exit.")
if __name__ == "__main__":
    main()
