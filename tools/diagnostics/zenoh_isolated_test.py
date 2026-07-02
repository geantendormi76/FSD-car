# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
import os
import sys
import struct
import time
import subprocess
import socket

# 自动定位并加载 CUDA DLL 避免 Windows 端 Zenoh 报错
cuda_bin_path = r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.1\bin"
if os.path.exists(cuda_bin_path) and os.name == 'nt':
    try: os.add_dll_directory(cuda_bin_path)
    except Exception: pass

import zenoh

def get_wsl_gateway_ip():
    try:
        cmd = "ip route | grep default | awk '{print $3}'"
        ip = subprocess.check_output(cmd, shell=True, text=True).strip()
        return ip if ip else "172.23.176.1"
    except Exception:
        return "172.23.176.1"

def get_windows_wsl_ip():
    try:
        hostname = socket.gethostname()
        ips = socket.gethostbyname_ex(hostname)[2]
        # 🎯 核心自愈：自动抓取 Windows 本地以 172. 开头的 WSL 虚拟网卡真实 IPv4 地址
        wsl_ip = next((ip for ip in ips if ip.startswith("172.")), None)
        return wsl_ip if wsl_ip else "0.0.0.0"
    except Exception:
        return "0.0.0.0"

def run_server():
    print("="*80)
    print("🛰️  NEXUS BIM Portal - Zenoh 纯净隔离诊断服务端 (Windows)")
    print("="*80)
    
    wsl_ip = get_windows_wsl_ip()
    print(f"🖥️  探测到 Windows 侧 WSL 虚拟网卡 IP: {wsl_ip}")
    
    z_config = zenoh.Config()
    z_config.insert_json5("mode", '"peer"')
    
    # 🎯 终极自愈：强行绑定到具体的 IPv4 网卡 IP，绕开 Windows 0.0.0.0 默认解析为 IPv6 [::] 导致的物理拒绝！
    endpoint = f'["tcp/{wsl_ip}:17449"]'
    z_config.insert_json5("listen/endpoints", endpoint)
    z_config.insert_json5("scouting/multicast/enabled", "false")
    
    session = zenoh.open(z_config)
    print(f"✅ [侦听就绪] Windows 端 Zenoh 成功监听: {wsl_ip}:17449。正在全力等待 WSL2 端连接...")

    def listener(sample):
        print(f"\n📥 收到 WSL2 神经指令 [Key: {sample.key_expr}]")
        payload_obj = sample.payload
        
        # 采用 2026 终极多路径自愈解码
        if hasattr(payload_obj, "to_bytes"):
            payload = payload_obj.to_bytes()
        elif hasattr(payload_obj, "contiguous"):
            payload = bytes(payload_obj.contiguous())
        else:
            payload = bytes(payload_obj)

        print(f"  裸字节流长度: {len(payload)} | 十六进制: {payload.hex()}")
        if len(payload) == 8:
            v, w = struct.unpack("<ff", payload)
            print(f"  🎯 运动学解码 -> 线速度 v: {v:.3f} m/s | 角速度 w: {w:.3f} rad/s")
        else:
            print("  ⚠️ 字节流长度错误")

    sub = session.declare_subscriber("fsd/spinal_cord/cmd_vel", listener)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 服务端退出")
    finally:
        session.close()

def run_client():
    print("="*80)
    print("🛰️  NEXUS BIM Portal - Zenoh 纯净隔离诊断客户端 (WSL2)")
    print("="*80)
    
    gateway_ip = get_wsl_gateway_ip()
    print(f"📡 目标 Windows 宿主机网关 IP: {gateway_ip}")

    z_config = zenoh.Config()
    # 🎯 强行配置为 Client 模式以建立确定性 TCP 连接
    z_config.insert_json5("mode", '"client"')
    z_config.insert_json5("connect/endpoints", f'["tcp/{gateway_ip}:17449"]')
    z_config.insert_json5("scouting/multicast/enabled", "false")
    
    session = zenoh.open(z_config)
    pub = session.declare_publisher("fsd/spinal_cord/cmd_vel")
    print("✅ [连接建立] WSL2 客户端连接成功，开始高频下发测试控制量...")

    try:
        tick = 0
        while True:
            tick += 1
            v = 0.2
            w = 0.0
            payload = struct.pack("<ff", v, w)
            pub.put(payload)
            if tick % 10 == 0:
                print(f"🚀 发送测试信号 -> v: {v:.2f} | w: {w:.2f} (已经发送 {tick} 次)")
            time.sleep(0.1) # 10Hz 发送
    except KeyboardInterrupt:
        print("\n🛑 客户端退出")
    finally:
        session.close()

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--server":
        run_server()
    else:
        run_client()