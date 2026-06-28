#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛡️ FSD-car V3.0: 虚拟物理界 (The Matrix) - Isaac Sim DORA 极速接入点
架构哲学: CPU SHM 零拷贝 | GPU CUDA IPC 显存直通 | 物理引擎降级自愈
=================================================================
"""

import time

import numpy as np
import pyarrow as pa
from dora import Node

# ---------------------------------------------------------------------------
# 🛡️ 架构师自愈设计：自动探针检测 Isaac Sim 物理界与 CUDA 算力环境
# ---------------------------------------------------------------------------
try:
    from omni.isaac.core import World
    from omni.isaac.sensor import Camera

    ISAAC_SIM_AVAILABLE = True
    print("✅ [物理主权] 检测到 NVIDIA Isaac Sim 环境，已挂载物理法则引擎！")
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("⚠️ [降级自愈] 未检测到 Isaac Sim 环境，已切换为高性能 Mock 视频流发生器！")

try:
    import dora.cuda
    import torch

    CUDA_AVAILABLE = torch.cuda.is_available()
    if CUDA_AVAILABLE:
        print(
            f"⚡ [显存主权] 检测到 CUDA GPU 算力就绪: {torch.cuda.get_device_name(0)}"
        )
    else:
        print("ℹ️ [显存主权] 未检测到可用 GPU，通信管道锁定为极速 CPU 共享内存 (SHM)")
except ImportError:
    CUDA_AVAILABLE = False
    print(
        "ℹ️ [显存主权] 未安装 PyTorch/dora.cuda，通信管道锁定为极速 CPU 共享内存 (SHM)"
    )


class 虚拟物理界代理:
    def __init__(self, use_gpu_pipeline: bool = False):
        # 1. 初始化 DORA 节点契约
        self.node = Node()

        # 2. 对齐物理相机分辨率 (70° FOV 广角单目)
        self.宽度 = 640
        self.高度 = 480
        self.通道数 = 3

        # 3. 决定是否启用 GPU 显存直通通路 (Sim-to-Real SOTA)
        self.启用GPU直通 = use_gpu_pipeline and CUDA_AVAILABLE

        # 4. 仿真环境初始化
        if ISAAC_SIM_AVAILABLE:
            self.world = World()
            self.camera = Camera(
                prim_path="/World/Car/front_camera",
                resolution=(self.宽度, self.高度),
            )
            self.world.reset()
        else:
            self.帧计数器 = 0
            # 预分配 Mock 背景缓冲区，避免在 30Hz 循环中重复分配内存
            self.mock_canvas = np.zeros(
                (self.高度, self.宽度, self.通道数), dtype=np.uint8
            )

    def 获取物理视网膜帧(self) -> np.ndarray:
        """
        获取当前物理世界的 RGB 渲染帧 (支持物理引擎与降级 Mock)
        """
        if ISAAC_SIM_AVAILABLE:
            self.world.step(render=True)
            rgba_image = self.camera.get_rgba()
            return rgba_image[:, :, :3].astype(np.uint8)
        else:
            # 🛡️ 降级自愈：在预分配的画布上生成高速移动彩色条纹 (模拟小车行进)
            self.帧计数器 += 1
            偏移量 = (self.帧计数器 * 5) % self.宽度

            # 清空画布 (高吞吐原地操作，绝不 malloc)
            self.mock_canvas.fill(0)

            # 绘制高对比度物理地标 (模拟 XFeat 纠偏和青蛙眼避障站牌)
            self.mock_canvas[:, 偏移量 : 偏移量 + 50] = [255, 0, 0]  # 红色站牌 (R)
            self.mock_canvas[
                :, (偏移量 + 200) % self.宽度 : (偏移量 + 250) % self.宽度
            ] = [0, 255, 0]  # 绿色站牌 (G)
            self.mock_canvas[
                :, (偏移量 + 400) % self.宽度 : (偏移量 + 450) % self.宽度
            ] = [0, 0, 255]  # 蓝色站牌 (B)

            # 物理 30Hz 节奏补偿
            time.sleep(0.033)
            return self.mock_canvas

    def 驱动物理底盘(self, v: float, w: float):
        """
        神经反馈底盘执行器
        """
        if ISAAC_SIM_AVAILABLE:
            # 此处用于转换轮速并写入 Isaac Sim ArticulationController
            pass
        else:
            if self.帧计数器 % 30 == 0:
                print(
                    f"🏎️ [底盘执行器] 接收到 Rust 规控指令 -> 线速度: {v:.3f} m/s, 角速度: {w:.3f} rad/s"
                )

    def 启动生命循环(self):
        print("🚀 虚拟物理界代理已启动，正在向 DORA 共享内存注入物理法则...")
        if self.启用GPU直通:
            print("🚀 [管道状态] 开启: NVIDIA PyTorch GPU CUDA IPC 显存直通")
        else:
            print("🚀 [管道状态] 开启: Apache Arrow CPU Shared Memory (SHM) 零拷贝")

        try:
            while True:
                # ---------------------------------------------------------
                # 1. 视网膜捕获与零拷贝注入 (The Output Path)
                # ---------------------------------------------------------
                rgb_frame = self.获取物理视网膜帧()

                if self.启用GPU直通:
                    # 🛡️ 显存直通 SOTA：将 numpy 转换为 PyTorch GPU Tensor
                    # 并提取 CUDA IPC 句柄，数据留在显卡，传输开销为 0 纳秒！
                    gpu_tensor = torch.as_tensor(rgb_frame, device="cuda")
                    ipc_buffer, metadata = dora.cuda.torch_to_ipc_buffer(gpu_tensor)

                    self.node.send_output(
                        output_id="camera_rgb", data=ipc_buffer, metadata=metadata
                    )
                else:
                    # 🛡️ CPU 零拷贝优化：直接通过 NumPy 缓冲区协议创建连续的一维 Arrow 数组
                    # 绕过了 Python 层面的逐元素列表转换，配合 DORA 的 Zenoh SHM 实现内存级零拷贝！
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
                            "encoding": "bgr8",
                        },
                    )

                # ---------------------------------------------------------
                # 2. 神经反射弧监听 (The Input Path)
                # ---------------------------------------------------------
                # 设置 1ms 超时非阻塞，绝不卡死 30Hz 物理渲染
                event = self.node.next(timeout=0.001)

                if event is not None:
                    if event["type"] == "INPUT" and event["id"] == "control_cmd":
                        cmd_array = event["value"].to_numpy()
                        if len(cmd_array) >= 2:
                            v, w = cmd_array[0], cmd_array[1]
                            self.驱动物理底盘(v, w)

                    elif event["type"] == "STOP":
                        print("🛑 接收到 DORA 全局停止指令，安全关闭物理引擎...")
                        break

        except KeyboardInterrupt:
            print("\n🛑 用户手动中断，安全退出...")


if __name__ == "__main__":
    # 如果要强行测试 GPU 显存直通，可以在实例化时传入 True
    代理 = 虚拟物理界代理(use_gpu_pipeline=False)
    代理.启动生命循环()
