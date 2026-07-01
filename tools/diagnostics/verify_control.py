# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
import struct
import subprocess
import time

import zenoh


def main():
    print("=" * 80)
    print("🛰️  FSD-car V3.0: WSL2 运动学极性物理隔离测试探针 (NAT 动态版)")
    print("=" * 80)

    # 1. 动态获取 Windows 宿主机 IP [cite: 1.1.3]
    try:
        cmd = "ip route show | grep default | awk '{print $3}'"
        gateway_ip = subprocess.check_output(cmd, shell=True, text=True).strip()
        if not gateway_ip:
            gateway_ip = "127.0.0.1"
    except Exception:
        gateway_ip = "127.0.0.1"

    print(f"📡 [并网连接] 正在单播连接 Windows 宿主机: {gateway_ip}:17449")

    # 2. 初始化极速 Zenoh 客户端
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["tcp/{gateway_ip}:17449"]')
    conf.insert_json5("scouting/multicast/enabled", "false")

    session = zenoh.open(conf)
    pub = session.declare_publisher("fsd/spinal_cord/cmd_vel")

    print(
        "\n🟢 [极性隔离序列启动] 请在 Windows 3D 仿真界面紧密观察两个轮子的滚动方向："
    )
    try:
        # 阶段一：静止 (3秒)
        print("\n⏱️ [0-3s] 阶段一：发送停止指令 (v = 0.0, w = 0.0) -> 小车应当完全静止")
        for _ in range(30):
            payload = struct.pack("<ff", 0.0, 0.0)
            pub.put(payload)
            time.sleep(0.1)

        # 阶段二：直行 (5秒)
        print(
            "\n⏱️ [3-8s] 阶段二：发送直行指令 (v = 0.2, w = 0.0) -> 左右车轮必须同时【向前滚】"
        )
        for _ in range(50):
            payload = struct.pack("<ff", 0.2, 0.0)
            pub.put(payload)
            time.sleep(0.1)

        # 阶段三：原地自转 (5秒)
        print(
            "\n⏱️ [8-13s] 阶段三：发送原地左转指令 (v = 0.0, w = 0.5) -> 左轮应当【后滚】，右轮应当【前滚】"
        )
        for _ in range(50):
            payload = struct.pack("<ff", 0.0, 0.5)
            pub.put(payload)
            time.sleep(0.1)

        # 阶段四：静止 (3秒)
        print(
            "\n⏱️ [13-16s] 阶段四：发送停止指令 (v = 0.0, w = 0.0) -> 小车应当完全静止"
        )
        for _ in range(30):
            payload = struct.pack("<ff", 0.0, 0.0)
            pub.put(payload)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n🛑 测试被用户手动中断")
    finally:
        # 强制发送安全停止
        pub.put(struct.pack("<ff", 0.0, 0.0))
        session.close()
        print("\n🏁 [测试序列结束] 物理通道已安全释放。")
        print("=" * 80)


if __name__ == "__main__":
    main()
