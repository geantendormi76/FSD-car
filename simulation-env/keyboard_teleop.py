#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "dora-rs==0.3.13",
#     "pyarrow>=14.0.0"
# ]
# ///
import socket
import struct
import time
import pyarrow as pa
from dora import Node

def main():
    # 1. 接入 DORA 拓扑网关
    print("💎 [DORA 接收端] DORA 零拷贝键盘遥控并网通道激活...")
    dora_node = Node()
    
    # 2. 物理绑定非阻塞式 UDP 监听套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 5005))
    sock.setblocking(False)  # 🛡️ 架构师自愈：设为完全非阻塞，避免 socket 锁死 DORA 主线程
    
    print("========================================================")
    print("🛰️  NEXUS - 键盘遥控桥接接收节点 (DORA Node) 已启动")
    print("正在监听网络套接字: 127.0.0.1:5005 (非阻塞高速通道)")
    print("========================================================")
    
    try:
        while True:
            # A. 尝试从 UDP 缓冲区非阻塞式读取遥控数据
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) == 8:
                    # 极速解析 IEEE 754 32位连续浮点数 [v, w]
                    v, w = struct.unpack('ff', data)
                    # 严格按照 FSD 运动指令契约打包为 Arrow Float32 数组广播
                    arrow_cmd = pa.array([v, w], type=pa.float32())
                    dora_node.send_output("control_cmd", arrow_cmd)
            except BlockingIOError:
                pass  # 缓冲区无数据，平滑过渡
            
            # B. 🛡️ 架构师自愈：使用 DORA 官方标准的非阻塞 try_recv 轮询系统生命周期事件
            event = dora_node.next(0.002)
            if event is not None:
                if event["type"] == "STOP":
                    print("\n🛑 [DORA 接收端] 接收到 DORA 停止信号，安全下线。")
                    break
            
            # 维持 500Hz 的超高频轮询，保护 CPU 不发生空转
            time.sleep(0.002)
    except Exception as e:
        print(f"\n❌ [DORA 接收端] 发生异常崩溃: {e}")
    finally:
        sock.close()
        print("🔌 [DORA 接收端] UDP 套接字已关闭，节点安全卸载。")

if __name__ == "__main__":
    main()