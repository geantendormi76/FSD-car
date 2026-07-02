#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

"""
=================================================================
🛡️ FSD-car V3.0: 仿真环境物理代理节点 (原生 Linux 零拷贝并网版)
设计哲学: 去网关化 | 双视觉算法下沉 | 共享内存直通 | 物理与算法时钟对齐
=================================================================
"""

import os
import sys
import struct
import numpy as np
import cv2
import torch
import torch.nn.functional as F
import onnxruntime as ort
import pyarrow as pa
from dora import Node

# ---------------------------------------------------------------------------
# 🛡️ 环境变量自愈：自动加载项目根目录的 .env 文件
# ---------------------------------------------------------------------------
def load_env_manually():
    # 向上自适应溯源两层，找到项目根目录下的 .env
    env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".env")
    if os.path.exists(env_path):
        with open(env_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    parts = line.split("=", 1)
                    if len(parts) == 2:
                        os.environ[parts[0].strip()] = parts[1].strip()
load_env_manually()

# ---------------------------------------------------------------------------
# 🛡️ NVIDIA Isaac Sim 启动哨兵 (LTS 级强依赖约束)
# ---------------------------------------------------------------------------
try:
    from isaacsim import SimulationApp
    # 本地原生运行，启动 headed 模式以便实时在大屏监测小车运动
    simulation_app = SimulationApp({"headless": False})
    
    import omni
    from pxr import UsdPhysics
    from isaacsim.core.api.world import World
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.sensors.camera import Camera
    from isaacsim.core.utils.stage import open_stage
    print("✅ [物理代理] NVIDIA Isaac Sim Standalone 引擎启动成功！")
except ImportError as e:
    print(f"❌ 致命错误：无法在本地导入 Isaac Sim 模块！请确保运行在 python.sh 环境下。报错: {e}")
    sys.exit(1)


# ===========================================================================
# 🐸 仿生青蛙眼不对称时空感受野（ERF/IRF）算法引擎
# ===========================================================================
class BionicFrogEye:
    def __init__(self, width=640, height=480):
        self.w = width
        self.h = height
        self.prev_gray = None
        
        # 预分配感受野显存金库，杜绝运行期动态申请
        self.erf = np.zeros((height, width), dtype=np.float32)
        self.irf = np.zeros((height, width), dtype=np.float32)
        
        self.alpha_erf = 0.4        # 兴奋感受野衰减率（极快，捕捉瞬态）
        self.alpha_irf = 0.85       # 抑制感受野衰减率（较慢，存储记忆）
        self.beta = 0.5             # 抑制强度权重
        self.event_threshold = 15.0 # 时间帧差事件门限
        
        # 预计算二次方物理距离权重矩阵，靠近图像底部（越近）权重越高
        y_indices, x_indices = np.indices((self.h, self.w))
        self.x_coords = x_indices.astype(np.float32)
        self.closeness_weight = (y_indices / float(self.h)) ** 2
        
    def process_frame(self, frame_rgb):
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            return 0.0, 0.0, np.zeros((self.h, self.w), dtype=np.uint8)
            
        diff = cv2.absdiff(self.prev_gray, gray)
        _, events = cv2.threshold(diff, self.event_threshold, 1.0, cv2.THRESH_BINARY)
        
        events_rf = cv2.GaussianBlur(events, (21, 21), 0)
        
        self.erf = self.erf * self.alpha_erf + events_rf
        self.irf = self.irf * self.alpha_irf + events_rf
        
        net_energy = np.maximum(0.0, self.erf - self.beta * self.irf)
        self.prev_gray = gray
        
        weighted_energy = net_energy * self.closeness_weight
        total_energy = np.sum(weighted_energy)
        
        if total_energy > 15.0:  # 避障激活门限
            x_c = np.sum(self.x_coords * weighted_energy) / total_energy
            dx = (x_c - self.w / 2.0) / (self.w / 2.0)  # 归一化至 [-1.0, 1.0]
            F_y = -dx * 1.5  # 逃逸方向与障碍物重心相反
            F_x = - (total_energy / (self.w * self.h)) * 5.0
        else:
            F_x, F_y = 0.0, 0.0
            
        heatmap = np.clip(net_energy * 255.0, 0, 255).astype(np.uint8)
        return F_x, F_y, heatmap


# ====================================================
#  📸 XFeat 局部高精度特征提取引擎 (本地 GPU/CPU 自适应版)
# ====================================================
class XFeatEngine:
    def __init__(self, model_path):
        # 原生 Linux 下直接启用 GPU 推理加速
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        print(f"✓ [XFeat] 神经网络推理引擎装载完毕，物理计算提供商: {self.session.get_providers()}")

    def extract(self, frame_rgb, top_k=200):
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        pad_y = (640 - 480) // 2
        padded = cv2.copyMakeBorder(gray, pad_y, 640 - 480 - pad_y, 0, 0, cv2.BORDER_CONSTANT, value=0)
        
        tensor = torch.from_numpy(padded).float().unsqueeze(0).unsqueeze(0) # [1, 1, 640, 640]
        mean, std = tensor.mean(), tensor.std()
        tensor = (tensor - mean) / (std + 1e-6)

        outs = self.session.run(None, {self.input_name: tensor.numpy()})
        desc, scores, rel = [torch.from_numpy(x) for x in outs]

        # 极致高精度极性校正
        if desc.shape[1] == 80 and desc.shape[2] == 80:
            desc = desc.permute(0, 3, 1, 2)
        if scores.shape[1] == 80 and scores.shape[2] == 80:
            scores = scores.permute(0, 3, 1, 2)
        if len(rel.shape) == 4 and rel.shape[3] == 1 and rel.shape[1] == 80 and rel.shape[2] == 80:
            rel = rel.permute(0, 3, 1, 2)

        scores = F.softmax(scores, dim=1)[:, :-1, :, :]
        scores = F.pixel_shuffle(scores, 8)
        rel = F.interpolate(rel, size=(640, 640), mode='bilinear', align_corners=False)
        conf = scores * rel

        conf_nms = F.max_pool2d(conf, kernel_size=5, stride=1, padding=2)
        mask = (conf == conf_nms) & (conf > 0.05)

        conf_flat = conf[mask]
        if len(conf_flat) == 0:
            return b''
        
        topk = min(top_k, len(conf_flat))
        scores_k, indices = torch.topk(conf_flat, topk)
        
        y, x = torch.where(mask[0, 0])
        y, x = y[indices], x[indices]

        grid_x = (x.float() / 639.0) * 2.0 - 1.0
        grid_y = (y.float() / 639.0) * 2.0 - 1.0
        grid = torch.stack([grid_x, grid_y], dim=1).unsqueeze(0).unsqueeze(2) # [1, N, 1, 2]
        
        # 🛡️ 架构师 2026 修复：使用标准的 grid_sample 提取特征描述子
        desc_sampled = F.grid_sample(desc, grid, mode='bilinear', align_corners=False)
        desc_sampled = desc_sampled.squeeze(0).squeeze(2).t() # [N, 64]
        desc_sampled = F.normalize(desc_sampled, p=2, dim=1)

        y = y.float() - pad_y
        x = x.float()

        valid = (y >= 0) & (y < 480)
        x, y, scores_k, desc_sampled = x[valid], y[valid], scores_k[valid], desc_sampled[valid]

        N = len(x)
        buffer = bytearray(struct.pack("<I", N))
        
        x_np, y_np, s_np, d_np = x.numpy(), y.numpy(), scores_k.numpy(), desc_sampled.numpy()
        for i in range(N):
            buffer.extend(struct.pack("<fff", x_np[i], y_np[i], s_np[i]))
            buffer.extend(d_np[i].tobytes())
            
        return bytes(buffer)


# ===========================================================================
#  🚀 DORA 原生节点主循环与仿真执行器
# ===========================================================================
def main():
    print("💎 [物理主权] 正在构建 FSD 本地零拷贝并网通道...")
    
    # 1. 初始化本地 DORA 节点契约
    dora_node = Node()

    # 2. 读取环境变量路径，加载物理场景
    fsd_assets_dir = os.environ.get("FSD_ASSETS_DIR", "/home/zhz/fsd_assets")
    usd_path = os.path.join(fsd_assets_dir, "Collected_fsd_car_racetrack", "fsd_car_racetrack.usd")
    
    if not os.path.exists(usd_path):
        print(f"❌ 致命错误：无法在指定路径找到物理场景文件 -> {usd_path}")
        sys.exit(1)
        
    open_stage(usd_path=usd_path)
    
    # 3. 初始化仿真世界
    world = World(stage_units_in_meters=1.0, physics_prim_path="/PhysicsScene")
    car_path = "/Root/jetbot"
    stage = omni.usd.get_context().get_stage()
    
    if not stage.GetPrimAtPath(car_path):
        print(f"❌ 致命错误：仿真场景中找不到小车模型，期望路径 -> {car_path}")
        sys.exit(1)

    # 🛡️ 物理关节属性重置：清除冗余阻尼，驯化为纯速度伺服关节
    for prim in stage.Traverse():
        if prim.GetPath().HasPrefix(car_path) and prim.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(1e5)

    # 4. 绑定物理实体
    car = Articulation(car_path)
    camera_path = f"{car_path}/rgb_camera"
    camera = Camera(prim_path=camera_path, name="bionic_retina", resolution=(640, 480))

    # 5. 加载算法引擎
    xfeat_model_path = os.path.join(fsd_assets_dir, "model", "xfeat_640x640.onnx")
    if not os.path.exists(xfeat_model_path):
        print(f"❌ 致命错误：找不到 XFeat ONNX 权重文件 -> {xfeat_model_path}")
        sys.exit(1)
        
    xfeat_engine = XFeatEngine(xfeat_model_path)
    frog_eye = BionicFrogEye(640, 480)

    # 6. 物理世界预热起跑
    world.reset()
    world.play()
    world.step(render=True)
    camera.initialize()
    
    print("🏆 [物理代理] 本地物理界仿真节点已成功激活，正在向 DORA 共享内存灌注高频流...")

    L = 0.1125  # 轮距
    R = 0.03    # 轮半径
    tick = 0

    v_cmd = 0.0
    w_cmd = 0.0

    try:
        while simulation_app.is_running():
            # A. 步进物理和渲染帧
            world.step(render=True)
            tick += 1

            # B. 极速显存抓取与前置过滤
            rgb_raw = camera.get_rgb()
            if rgb_raw is not None:
                if hasattr(rgb_raw, "cpu"):
                    rgb_frame = rgb_raw.cpu().numpy()
                else:
                    rgb_frame = rgb_raw
                    
                # 剥离 Alpha 通道并重整像素值
                if len(rgb_frame.shape) == 3 and rgb_frame.shape[2] == 4:
                    rgb_frame = rgb_frame[:, :, :3]
                if rgb_frame.dtype == np.float32 or rgb_frame.dtype == np.float64:
                    rgb_frame = (rgb_frame * 255.0).astype(np.uint8)

                if rgb_frame.size > 0:
                    # 1. 🐸 100Hz 仿生感受野势场解算并实时并网 (直接转换为 Arrow Uint8 数组写入共享内存)
                    F_x, F_y, heatmap = frog_eye.process_frame(rgb_frame)
                    fe_payload = struct.pack("<ff", F_x, F_y)
                    
                    arrow_obstacle_force = pa.array(np.frombuffer(fe_payload, dtype=np.uint8))
                    dora_node.send_output("obstacle_force", arrow_obstacle_force)

                    # 2. 📸 1Hz 慢系统 XFeat 骨干特征提取并并网
                    if tick % 100 == 0:
                        xfeat_payload = xfeat_engine.extract(rgb_frame, top_k=200)
                        if xfeat_payload:
                            arrow_xfeat_features = pa.array(np.frombuffer(xfeat_payload, dtype=np.uint8))
                            dora_node.send_output("xfeat_features", arrow_xfeat_features)

            # C. 🏎️ 非阻塞监听快系统大脑下发的运动规控反馈 (0 延迟)
            event = dora_node.next(timeout=0.001)
            if event is not None:
                if event["type"] == "INPUT" and event["id"] == "control_cmd":
                    cmd_bytes = bytes(event["value"])
                    if len(cmd_bytes) == 8:
                        v_cmd, w_cmd = struct.unpack("<ff", cmd_bytes)
                elif event["type"] == "STOP":
                    print("🛑 [物理代理] 收到 DORA 全局停止指令，退出仿真。")
                    break

            # D. 执行物理控制
            v_left = (v_cmd - w_cmd * L / 2.0) / R
            v_right = -(v_cmd + w_cmd * L / 2.0) / R  # 右轮极性反向自愈
            car.set_dof_velocity_targets(np.array([v_left, v_right]))

    except KeyboardInterrupt:
        print("\n🛑 用户手动中断，优雅下线。")
    finally:
        # 安全制动
        try:
            car.set_dof_velocity_targets(np.array([0.0, 0.0]))
            world.step(render=True)
        except Exception:
            pass
        simulation_app.close()
        print("🔌 物理仿真界代理已安全卸载。")


if __name__ == "__main__":
    main()