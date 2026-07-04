#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛡️ FSD-car V4.2: 仿真环境物理代理节点 (Ubuntu 26.04 级联退避版)
设计哲学: Headless 离屏 | 100Hz 绝对物理时钟锁定 | 共享内存直通
核心规范: Isaac Sim 2026 Core API (SimulationContext)
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
# 🛡️ 物理资产与环境变量自愈
# ---------------------------------------------------------------------------
os.environ["ENABLE_CAMERAS"] = "1"
os.environ["ISAAC_ASSET_ROOT"] = "/run/media/zhz/数据/isaac_assets"

def load_env_manually():
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
# 🛡️ NVIDIA Isaac Sim 2026 启动哨兵
# ---------------------------------------------------------------------------
try:
    sys.argv.extend(["--/log/level=error", "--/log/fileLogLevel=error"])
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({
        "headless": False,
    })
    
    import omni
    from pxr import UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import open_stage
    import omni.replicator.core as rep
    print("✅ [物理代理] NVIDIA Isaac Sim 2026 核心引擎启动成功！")
except ImportError as e:
    print(f"❌ 致命错误：无法在本地导入 Isaac Sim 模块！报错: {e}")
    sys.exit(1)

# ===========================================================================
# 🐸 仿生青蛙眼不对称时空感受野（ERF/IRF）算法引擎
# ===========================================================================
class BionicFrogEye:
    def __init__(self, width=640, height=480):
        self.w = width
        self.h = height
        self.frame_buffer = []
        self.buffer_size = 5
        self.erf = np.zeros((height, width), dtype=np.float32)
        self.irf = np.zeros((height, width), dtype=np.float32)
        self.alpha_erf = 0.4
        self.alpha_irf = 0.85
        self.beta = 0.5
        self.event_threshold = 10.0
        
        y_indices, x_indices = np.indices((self.h, self.w))
        self.x_coords = x_indices.astype(np.float32)
        self.closeness_weight = (y_indices / float(self.h)) ** 2
        
    def process_frame(self, frame_rgb):
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        self.frame_buffer.append(gray)
        if len(self.frame_buffer) < self.buffer_size:
            return 0.0, 0.0, np.zeros((self.h, self.w), dtype=np.uint8)
            
        history_frame = self.frame_buffer.pop(0)
        diff = cv2.absdiff(history_frame, gray)
        _, events = cv2.threshold(diff, self.event_threshold, 1.0, cv2.THRESH_BINARY)
        events_rf = cv2.GaussianBlur(events, (21, 21), 0)
        self.erf = self.erf * self.alpha_erf + events_rf
        self.irf = self.irf * self.alpha_irf + events_rf
        net_energy = np.maximum(0.0, self.erf - self.beta * self.irf)
        self.prev_gray = gray
        weighted_energy = net_energy * self.closeness_weight
        total_energy = np.sum(weighted_energy)
        if total_energy > 15.0:
            x_c = np.sum(self.x_coords * weighted_energy) / total_energy
            dx = (x_c - self.w / 2.0) / (self.w / 2.0)
            F_y = -dx * 1.5
            F_x = - (total_energy / (self.w * self.h)) * 5.0
        else:
            F_x, F_y = 0.0, 0.0
        heatmap = np.clip(net_energy * 255.0, 0, 255).astype(np.uint8)
        return F_x, F_y, heatmap

# ====================================================
#  📸 XFeat 局部高精度特征提取引擎
# ====================================================
class XFeatEngine:
    def __init__(self, model_path):
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        print(f"✓ [XFeat] 神经网络推理引擎装载完毕")

    def extract(self, frame_rgb, top_k=200):
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        pad_y = (640 - 480) // 2
        padded = cv2.copyMakeBorder(gray, pad_y, 640 - 480 - pad_y, 0, 0, cv2.BORDER_CONSTANT, value=0)
        tensor = torch.from_numpy(padded).float().unsqueeze(0).unsqueeze(0)
        mean, std = tensor.mean(), tensor.std()
        tensor = (tensor - mean) / (std + 1e-6)
        outs = self.session.run(None, {self.input_name: tensor.numpy()})
        desc, scores, rel = [torch.from_numpy(x) for x in outs]
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
        grid = torch.stack([grid_x, grid_y], dim=1).unsqueeze(0).unsqueeze(2)
        desc_sampled = F.grid_sample(desc, grid, mode='bilinear', align_corners=False)
        desc_sampled = desc_sampled.squeeze(0).squeeze(2).t()
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
#  🚀 DORA 原生节点主循环与 2026 确定性仿真执行器
# ===========================================================================
def main():
    print("💎 [物理主权] 正在构建 FSD 本地零拷贝并网通道...")
    dora_node = Node()

    fsd_assets_dir = "/home/zhz/fsd-car/assets"
    usd_path = os.path.join(fsd_assets_dir, "fsd_car_racetrack.usd")
    if not os.path.exists(usd_path):
        print(f"❌ 致命错误：找不到物理场景文件 -> {usd_path}")
        sys.exit(1)
    open_stage(usd_path=usd_path)
    
    world = World(physics_dt=0.01, rendering_dt=0.01, backend="numpy")
    car_path = "/Root/jetbot"
    stage = omni.usd.get_context().get_stage()

    for prim in stage.Traverse():
        if prim.GetPath().HasPrefix(car_path) and prim.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(1e5)

    car = Articulation(prim_paths_expr=car_path, name="jetbot")
    world.scene.add(car)

    camera_path = f"{car_path}/chassis/rgb_camera/jetbot_camera"
    render_product = rep.create.render_product(camera_path, (640, 480))
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([render_product])

    world.reset()
    world.play()
    
    # 预热
    print("⏳ [物理代理] 正在进行 RTX 渲染管道硬件预热...")
    for _ in range(60):
        world.step(render=True)
    print("✅ [物理代理] RTX 渲染管道预热完毕！")

    L = 0.1125
    R = 0.03
    tick = 0
    v_cmd = 0.0
    w_cmd = 0.0

    try:
        while simulation_app.is_running():
            # 🛡️ 始终保持 RENDER=TRUE，维系底层 syntheticdata 管线生命力，防止 C++ 段错误
            world.step(render=True)
            tick += 1

            # 🛠️ 时序测试：发送 Dummy 控制力，绕过慢速 get_data() 拷贝
            dummy_f_x, dummy_f_y = 0.1, 0.2
            fe_payload = struct.pack("<ff", dummy_f_x, dummy_f_y)
            arrow_obstacle_force = pa.array(np.frombuffer(fe_payload, dtype=np.uint8))
            dora_node.send_output("obstacle_force", arrow_obstacle_force)

            # 🛡️ 架构师 2026 级自愈：级联事件流控制锁
            global_stop = False

            while True:
                # 极速非阻塞事件流榨干
                event = dora_node.next(timeout=0.0001)
                if event is None:
                    break
                
                ev_type = event["type"]
                if ev_type == "INPUT":
                    if event["id"] == "control_cmd":
                        cmd_bytes = bytes(event["value"])
                        if len(cmd_bytes) == 8:
                            v_cmd, w_cmd = struct.unpack("<ff", cmd_bytes)
                elif ev_type == "STOP":
                    print("🛑 [物理代理] 收到 DORA 全局停止指令，触发二级退避。")
                    global_stop = True
                    break

            # 🛡️ 终极自愈：彻底打破防盗门，优雅退出主程序，拒绝无响应挂起！
            if global_stop:
                break

            # 差速驱动逻辑
            v_left = (v_cmd - w_cmd * L / 2.0) / R
            v_right = (v_cmd + w_cmd * L / 2.0) / R
            car.set_joint_velocity_targets(np.array([[v_left, v_right]]))

            # 高频日志打印
            if tick % 100 == 0:
                print(
                    f"📊 [Python 100Hz 纯净度] 步数: {tick:<6} | "
                    f"Dora反馈指令: v={v_cmd:.3f}, w={w_cmd:.3f} | "
                    f"电调靶向速度: Left={v_left:.2f}, Right={v_right:.2f}"
                )

    except KeyboardInterrupt:
        print("\n🛑 用户手动中断")
    finally:
        try:
            car.set_joint_velocity_targets(np.array([[0.0, 0.0]]))
            world.step(render=True)
            world.stop()
        except Exception:
            pass
        simulation_app.close()
        print("🔌 物理仿真界代理已安全卸载。")

if __name__ == "__main__":
    main()