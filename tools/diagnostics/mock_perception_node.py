import time
import struct
import numpy as np
import pyarrow as pa
from dora import Node

node = Node()
print("🟢 [MOCK感知] 零拷贝模拟通道已并网！开始发送 100Hz 避障力...")

# 🎯 物理温启动：在进入阻塞循环前主动发射第一次避障力，打破时序死锁！
fe_payload = struct.pack("<ff", -0.05, 0.1)
node.send_output("obstacle_force", pa.array(np.frombuffer(fe_payload, dtype=np.uint8)))

tick = 0
for event in node:
    if event["type"] == "INPUT" and event["id"] == "control_cmd":
        v, w = struct.unpack("<ff", bytes(event["value"]))
        print(f"📥 [收] 快脑反馈指令 -> 线速度: {v:.3f} m/s | 角速度: {w:.3f} rad/s")
        tick += 1
        if tick >= 20:
            print("✅ [SHM测试] 控制环路完美闭环，正在安全退出...")
            break

        # 闭环状态下，高频持续发送避障力信号
        node.send_output("obstacle_force", pa.array(np.frombuffer(fe_payload, dtype=np.uint8)))
        time.sleep(0.01)
