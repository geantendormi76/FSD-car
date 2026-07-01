#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

"""
=================================================================
🛡️ FSD-car V3.0: 虚拟物理界代理 (自适应路由自愈版)
架构哲学: CPU SHM 零拷贝 | Zenoh 1.0+ 确定性单播 | 路由网关动态对齐
=================================================================
"""

import queue
import subprocess
import time

import numpy as np
import pyarrow as pa
import zenoh
from dora import Node

# ---------------------------------------------------------------------------
# 🛡️ 架构师自愈设计：自动启动 SimulationApp (WSL2 无 Isaac Sim 时安全降级)
# ---------------------------------------------------------------------------
ISAAC_SIM_AVAILABLE = True
simulation_app = None

try:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})
    from isaacsim.core.api.world import World
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.sensors.camera import Camera

    print("✅ [物理主权] NVIDIA Isaac Sim Standalone 引擎启动成功！")
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print(
        "⚠️ [降级自愈] WSL2 环境未搭载本地 Isaac Sim 物理界，已切换为高性能 Windows 组网中枢！"
    )


class 虚拟物理界代理:
    def __init__(self, use_gpu_pipeline: bool = False):
        # 1. 初始化 DORA 节点契约
        self.node = Node()

        # 2. 对齐物理相机分辨率
        self.宽度 = 640
        self.高度 = 480
        self.通道数 = 3

        # 3. 仿真环境初始化
        if ISAAC_SIM_AVAILABLE:
            usd_path = r"D:\isaac_assets\fsd_car_racetrack.usd"
            self.world = World(
                stage_units_in_meters=1.0,
                usd_path=usd_path,
                physics_prim_path="/PhysicsScene",
            )

            self.car_path = "/Root/jetbot"
            self.car = Articulation(self.car_path)
            self.world.scene.add(self.car)

            camera_path = f"{self.car_path}/rgb_camera"
            self.camera = Camera(
                prim_path=camera_path,
                name="bionic_retina",
                resolution=(self.宽度, self.高度),
            )

            self.world.reset()
            self.world.play()
            self.world.step(render=True)
            self.camera.initialize()
        else:
            self.帧计数器 = 0
            self.mock_canvas = np.zeros(
                (self.高度, self.宽度, self.通道数), dtype=np.uint8
            )
            self.zenoh_queue = queue.Queue(maxsize=1)

            # 🎯 2026 自愈核心：直接从 Linux 路由表动态提取当前的 Windows 宿主机网关 IP
            try:
                cmd = "ip route show | grep default | awk '{print $3}'"
                gateway_ip = subprocess.check_output(cmd, shell=True, text=True).strip()
                if not gateway_ip:
                    gateway_ip = "127.0.0.1"
            except Exception:
                gateway_ip = "127.0.0.1"

            # 配置 Zenoh 客户端，锁定对齐后的 17449 端口与动态 IP
            self.z_config = zenoh.Config()
            endpoint = f'["tcp/{gateway_ip}:17449"]'
            self.z_config.insert_json5("connect/endpoints", endpoint)
            self.z_config.insert_json5(
                "scouting/multicast/enabled", "false"
            )  # 禁用多播自发现

            self.z_session = zenoh.open(self.z_config)

            # 异步监听来自 Windows 端发送的真实相机视网膜图像
            def zenoh_camera_listener(sample):
                if self.zenoh_queue.full():
                    try:
                        self.zenoh_queue.get_nowait()
                    except queue.Empty:
                        pass
                self.zenoh_queue.put(sample.payload)

            self.camera_sub = self.z_session.declare_subscriber(
                "fsd/perception/camera_rgb", zenoh_camera_listener
            )
            print(
                f"🔗 [WSL2 代理中枢] 动态探测到并网通道! 正在单播连接 Windows -> [tcp/{gateway_ip}:17449]..."
            )

    def 生成动态Mock画布(self) -> np.ndarray:
        self.帧计数器 += 1
        偏移量 = (self.帧计数器 * 5) % self.宽度
        self.mock_canvas.fill(0)
        self.mock_canvas[:, 偏移量 : 偏移量 + 50] = [255, 0, 0]  # 红色站牌 (R)
        self.mock_canvas[:, (偏移量 + 200) % self.宽度 : (偏移量 + 250) % self.宽度] = [
            0,
            255,
            0,
        ]  # 绿色站牌 (G)
        self.mock_canvas[:, (偏移量 + 400) % self.宽度 : (偏移量 + 450) % self.宽度] = [
            0,
            0,
            255,
        ]  # 蓝色站牌 (B)
        time.sleep(0.033)  # 30Hz 节奏补偿
        return self.mock_canvas

    def 获取物理视网膜帧(self) -> np.ndarray:
        if ISAAC_SIM_AVAILABLE:
            self.world.step(render=True)
            return self.camera.get_rgb()
        else:
            try:
                # 33ms 超时，确保不阻塞 DORA 时间片
                payload = self.zenoh_queue.get(timeout=0.033)
                rgb_frame = np.frombuffer(bytes(payload), dtype=np.uint8).reshape(
                    (self.高度, self.宽度, self.通道数)
                )
                return rgb_frame
            except queue.Empty:
                return self.生成动态Mock画布()

    def 驱动物理底盘(self, v: float, w: float):
        if ISAAC_SIM_AVAILABLE:
            L = 0.1125  # 轮距 (meters)
            R = 0.03  # 轮半径 (meters)

            v_left = (v - w * L / 2.0) / R
            v_right = -(v + w * L / 2.0) / R  # 右轮极性反向自愈

            self.car.set_dof_velocity_targets(np.array([v_left, v_right]))
        else:
            if self.帧计数器 % 30 == 0:
                print(
                    f"🏎️ [WSL2 代理反馈] 转发指令 -> 线速度: {v:.3f} m/s, 角速度: {w:.3f} rad/s"
                )

    def 启动生命循环(self):
        print("🚀 虚拟物理界代理已启动，正在向 DORA 共享内存注入物理法则...")
        print("🚀 [管道状态] 开启: Apache Arrow CPU Shared Memory (SHM) 零拷贝")

        try:
            while True:
                # 1. 视网膜捕获与零拷贝注入
                rgb_frame = self.获取物理视网膜帧()

                if rgb_frame is None:
                    continue

                flat_frame = rgb_frame.reshape(-1)  # 共享相同底层内存视图
                arrow_array = pa.Array.from_buffers(
                    pa.uint8(), len(flat_frame), [None, pa.py_buffer(flat_frame)]
                )

                self.node.send_output(
                    output_id="camera_rgb",
                    data=arrow_array,
                    metadata={
                        "width": self.宽度,
                        "height": self.高度,
                        "channels": self.通道数,
                        "encoding": "rgb8",
                    },
                )

                # 2. 神经反射弧监听
                event = self.node.next(timeout=0.001)

                if event is not None:
                    if event["type"] == "INPUT" and event["id"] == "control_cmd":
                        cmd_array = event["value"].to_numpy()
                        # 🛡️ 架构师 2026 物理主权自愈：
                        # 由于 Rust 端发来的是 8 字节裸内存 [f32; 2]（线速度与角速度），
                        # DORA 传输时将其封装为了 uint8 的 Arrow 数组。
                        # 我们必须使用 np.frombuffer 进行零拷贝二进制还原，彻底解决将原始字节误当做速度值而导致小车原地疯狂打转的 Bug！
                        if len(cmd_array) == 8:
                            v, w = np.frombuffer(cmd_array, dtype=np.float32)
                            self.驱动物理底盘(v, w)

                    elif event["type"] == "STOP":
                        print("🛑 接收到 DORA 全局停止指令，安全关闭物理引擎...")
                        break

        except KeyboardInterrupt:
            print("\n🛑 用户手动中断，安全退出...")
        finally:
            if simulation_app is not None:
                print("🔌 正在安全释放 Isaac Sim 物理界进程...")
                simulation_app.close()


if __name__ == "__main__":
    代理 = 虚拟物理界代理(use_gpu_pipeline=False)
    代理.启动生命循环()
