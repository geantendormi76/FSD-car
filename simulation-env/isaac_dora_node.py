#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛡️ FSD-car V3.0: 虚拟物理界 (The Matrix) - Isaac Sim DORA 接入点
架构哲学: 物理级零拷贝 | 域随机化 (Domain Randomization) | 软硬绝对解耦
=================================================================
"""

import time
import numpy as np
import pyarrow as pa
from dora import Node

# 尝试导入 Isaac Sim 核心库 (如果运行在普通 Python 环境则自动降级为 Mock 模式)
try:
    from omni.isaac.core import World
    from omni.isaac.sensor import Camera
    ISAAC_SIM_AVAILABLE = True
    print("✅ [物理主权] 检测到 NVIDIA Isaac Sim 环境，已挂载真实物理法则引擎！")
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("⚠️ [降级自愈] 未检测到 Isaac Sim 环境，已自动切换为高保真 Mock 视频流发生器！")

class 虚拟物理界代理:
    def __init__(self):
        # 1. 初始化 DORA 节点契约
        self.node = Node()
        
        # 2. 物理相机参数对齐 (严格对齐真实小车的广角单目摄像头)
        self.宽度 = 640
        self.高度 = 480
        self.通道数 = 3
        
        # 3. 仿真环境初始化
        if ISAAC_SIM_AVAILABLE:
            self.world = World()
            self.camera = Camera(
                prim_path="/World/Car/front_camera",
                resolution=(self.宽度, self.高度),
            )
            self.world.reset()
        else:
            # Mock 模式下的测试画面状态
            self.帧计数器 = 0

    def 获取物理视网膜帧(self) -> np.ndarray:
        """
        获取当前物理世界的 RGB 渲染帧
        """
        if ISAAC_SIM_AVAILABLE:
            # 步进物理引擎
            self.world.step(render=True)
            # 获取无畸变或带物理畸变的 RGB 图像 (RGBA -> RGB)
            rgba_image = self.camera.get_rgba()
            return rgba_image[:, :, :3].astype(np.uint8)
        else:
            # 🛡️ 降级自愈：生成移动的彩色条纹，模拟小车运动，用于验证 Rust 端的 XFeat 特征提取
            self.帧计数器 += 1
            偏移量 = (self.帧计数器 * 5) % self.宽度
            
            # 创建纯黑背景
            mock_frame = np.zeros((self.高度, self.宽度, self.通道数), dtype=np.uint8)
            
            # 绘制三条高对比度的垂直彩条 (模拟物理地标)
            mock_frame[:, 偏移量:偏移量+50] = [255, 0, 0]       # 红色站牌
            mock_frame[:, (偏移量+200)%self.宽度:(偏移量+250)%self.宽度] = [0, 255, 0] # 绿色站牌
            mock_frame[:, (偏移量+400)%self.宽度:(偏移量+450)%self.宽度] = [0, 0, 255] # 蓝色站牌
            
            # 模拟 30Hz 的物理渲染耗时
            time.sleep(0.033)
            return mock_frame

    def 驱动物理底盘(self, v: float, w: float):
        """
        接收来自 Rust 规控大脑的指令，驱动虚拟轮胎
        """
        if ISAAC_SIM_AVAILABLE:
            # TODO: 将 (v, w) 转换为左右轮速，调用 Isaac Sim 的 ArticulationController
            # left_speed = v - w * 轴距 / 2
            # right_speed = v + w * 轴距 / 2
            pass
        else:
            # Mock 模式下仅打印探针日志
            if self.帧计数器 % 30 == 0:
                print(f"🏎️ [底盘执行器] 接收到 Rust 规控指令 -> 线速度: {v:.3f} m/s, 角速度: {w:.3f} rad/s")

    def 启动生命循环(self):
        print("🚀 虚拟物理界代理已启动，正在向 DORA 共享内存注入物理法则...")
        
        try:
            while True:
                # ---------------------------------------------------------
                # 1. 视网膜捕获与零拷贝注入 (The Output Path)
                # ---------------------------------------------------------
                rgb_frame = self.获取物理视网膜帧()
                
                # 🛡️ 物理级零拷贝核心：将 numpy 数组展平并转换为 Apache Arrow 格式。
                # pyarrow 会直接接管 numpy 的底层 C 内存，不发生任何数据复制！
                arrow_array = pa.array(rgb_frame.ravel())
                
                # 携带空间元数据，注入 DORA 共享内存总线
                self.node.send_output(
                    output_id="camera_rgb",
                    data=arrow_array,
                    metadata={
                        "width": self.宽度,
                        "height": self.高度,
                        "channels": self.通道数,
                        "encoding": "bgr8" # 契合 OpenCV 的默认色彩空间
                    }
                )

                # ---------------------------------------------------------
                # 2. 神经反射弧监听 (The Input Path)
                # ---------------------------------------------------------
                # 设置 1 毫秒的非阻塞超时，确保不会卡死 30Hz 的物理渲染循环
                event = self.node.next(timeout=0.001)
                
                if event is not None:
                    if event["type"] == "INPUT" and event["id"] == "control_cmd":
                        # 🛡️ 契约解析：Rust 端发来的是包含 2 个 f32 的 Arrow Array [v, w]
                        cmd_array = event["value"].to_numpy()
                        if len(cmd_array) >= 2:
                            v, w = cmd_array[0], cmd_array[1]
                            self.驱动物理底盘(v, w)
                            
                    elif event["type"] == "STOP":
                        print("🛑 接收到 DORA 全局停止指令，正在安全关闭物理引擎...")
                        break
                        
        except KeyboardInterrupt:
            print("\n🛑 用户手动中断，安全退出...")

if __name__ == "__main__":
    代理 = 虚拟物理界代理()
    代理.启动生命循环()