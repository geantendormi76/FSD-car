# -*- coding: utf-8 -*-
import socket
import subprocess
import sys
import time


def run_probe():
    print("=" * 60)
    print("🔬 FSD-car V3.0: 跨 OS 物理边界 TCP 诊断探针")
    print("=" * 60)

    # 1. 动态捕获 Windows 宿主机 IP
    print("[1/3] 正在解析 WSL2 路由表，定位 Windows 宿主机 IP...")
    try:
        cmd = "ip route show | grep default | awk '{print $3}'"
        gateway_ip = subprocess.check_output(cmd, shell=True, text=True).strip()
        if not gateway_ip:
            raise ValueError("无法获取网关 IP")
        print(f"  📍 锁定目标 IP: {gateway_ip}")
    except Exception as e:
        print(f"  ❌ 路由解析失败: {e}")
        sys.exit(1)

    # 2. 构造底层 TCP Socket 探针
    port = 17449
    print(f"\n[2/3] 正在向 {gateway_ip}:{port} 发射 TCP SYN 探测包...")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 设置 3 秒超时，足以判断防火墙是否丢包
    s.settimeout(3.0)

    start_time = time.time()
    try:
        # 尝试建立三次握手
        s.connect((gateway_ip, port))
        elapsed = (time.time() - start_time) * 1000
        print(f"  ✅ 握手成功！耗时: {elapsed:.2f} ms")
        print("\n[3/3] 🟢 最终诊断结论：")
        print(
            "  物理通道完全畅通！Windows 防火墙未拦截，且 isaac_sim_gateway.py 正在正常监听。"
        )
        print("  如果此时 Rust 仍然报错，说明是 Zenoh 协议层面的问题（如版本不一致）。")
        s.close()

    except socket.timeout:
        print("  🔴 握手失败！耗时: > 3000 ms (Timeout)")
        print("\n[3/3] 🔴 最终诊断结论：【元凶二：防火墙物理拦截】")
        print("  -> 症状：数据包发出后如泥牛入海，没有任何回应。")
        print("  -> 药方：Windows Defender 防火墙拦截了来自 WSL2 子网的入站请求。")
        print("  -> 处方：请在 Windows PowerShell (管理员) 中执行放行命令：")
        print(
            '     New-NetFirewallRule -DisplayName "FSD-Zenoh-17449" -Direction Inbound -LocalPort 17449 -Protocol TCP -Action Allow'
        )

    except ConnectionRefusedError:
        print("  🔴 握手失败！连接被瞬间拒绝 (Connection Refused)")
        print("\n[3/3] 🔴 最终诊断结论：【元凶一：服务未就绪 / 时序倒置】")
        print("  -> 症状：防火墙没有拦截，但目标端口上没有任何进程在接客。")
        print(
            "  -> 药方：Windows 端的 isaac_sim_gateway.py 根本没有运行，或者启动报错退出了。"
        )
        print(
            "  -> 处方：请先在 Windows 端成功运行网关脚本，确认看到 [单播通道锁紧] 日志后，再启动 WSL2 侧的程序。"
        )

    except Exception as e:
        print(f"  ⚠️ 发生未知网络异常: {e}")

    print("=" * 60)


if __name__ == "__main__":
    run_probe()
