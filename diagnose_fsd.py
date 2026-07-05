#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import socket
import sys

def check_usd_assets():
    assets_dir = "/home/zhz/fsd-car/assets"
    print(f"📁 [探针] 扫描资产目录: {assets_dir}")
    if os.path.exists(assets_dir):
        files = os.listdir(assets_dir)
        usd_files = [f for f in files if f.endswith(".usd")]
        for f in usd_files:
            print(f"  -> Found USD Stage: {f}")
        return usd_files
    else:
        print("  -> ❌ 资产目录不存在")
        return []

def check_udp_port(port):
    print(f"🔌 [探针] 测试本地端口 {port} 绑定状态...")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", port))
        print(f"  -> 🟢 UDP 端口 {port} 未被占用，可正常进行网络通信。")
    except Exception as e:
        print(f"  -> ❌ UDP 端口 {port} 绑定失败 (可能被占用或残留): {e}")
    finally:
        s.close()

def audit_isaac_dora_node_code():
    node_path = "/home/zhz/fsd-car/simulation-env/isaac_dora_node.py"
    print(f"🧠 [探针] 审计物理代理路径: {node_path}")
    if os.path.exists(node_path):
        with open(node_path, "r", encoding="utf-8") as f:
            content = f.read()
            # 检查硬编码的 USD 路径
            if "fsd_car_racetrack.usd" in content:
                print("  -> ⚠️ 发现病灶：`isaac_dora_node.py` 内部锁死了硬编码路径 'fsd_car_racetrack.usd'，未进行 clean 变体的自适应检测！")
            else:
                print("  -> 🟢 未发现硬编码赛道。")
    else:
        print("  -> ❌ 找不到物理代理节点代码")

print("========================================================")
print("🛰️  NEXUS - FSD-car 自动化诊断探针已启动...")
print("========================================================")
check_usd_assets()
check_udp_port(5005)
audit_isaac_dora_node_code()
print("========================================================")
