#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛰️  NEXUS - 视觉与物理集成自愈测试探针 (Standalone Integration Probe)
设计哲学: 1:1 还原生产级算法 | 彻底剥离 DORA 依赖 | 定位 GUI 假死瓶颈
使用规范: 必须通过 /home/zhz/isaacsim/python.sh 启动
=================================================================
"""

import os
import sys
import time
import struct
import numpy as np
import cv2
import torch
import torch.nn.functional as F
import onnxruntime as ort

# 强行注入环境变量
os.environ["ENABLE_CAMERAS"] = "1"
os.environ["ISAAC_ASSET_ROOT"] = "/run/media/zhz/数据/isaac_assets"
sys.argv.extend(["--/log/level=error", "--/log/fileLogLevel=error"])

# 强制将 cuDNN 9 的路径在 Python 内部注入链接器，确保运行时 100% 成功加载 CUDA EP
try:
    import nvidia.cudnn
    cudnn_lib = os.path.join(nvidia.cudnn.__path__[0], "lib")
    os.environ["LD_LIBRARY_PATH"] = f"{cudnn_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"
    print(f"🟢 [探针自愈] 成功注入 cuDNN 9 链接通路: {cudnn_lib}")
except Exception as e:
    print(f"⚠️ [探针警告] 无法自动定位 nvidia.cudnn 包: {e}")

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # 开启窗口进行直观审计

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import open_stage
import omni.replicator.core as rep

# 1. 1:1 还原生产级算法类 (直接从物理代理复制)
class BionicFrogEye:
    def __init__(self, width=640, height=480):
        self.w, self.h = width, height
        self.frame_buffer = []
        self.buffer_size = 5
        self.erf = np.zeros((height, width), dtype=np.float32)
        self.irf = np.zeros((height, width), dtype=np.float32)
        self.alpha_erf, self.alpha_irf, self.beta = 0.4, 0.85, 0.5
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

class XFeatEngine:
    def __init__(self, model_path):
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        print(f"✓ [XFeat] 神经网络引擎成功初始化，当前活跃 Execution Provider: {self.session.get_providers()}")

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

def main():
    print("🎯 [NEXUS 探针] 正在载入物理赛道 USD...")
    open_stage(usd_path="/home/zhz/fsd-car/assets/fsd_car_racetrack.usd")
    
    world = World(physics_dt=0.01, rendering_dt=0.01, backend="numpy")
    car = Articulation(prim_paths_expr="/Root/jetbot", name="jetbot")
    world.scene.add(car)

    # 离屏图像绑定
    camera_path = "/Root/jetbot/chassis/rgb_camera/jetbot_camera"
    render_product = rep.create.render_product(camera_path, (640, 480))
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([render_product])

    # 实例化双感知器
    frog_eye = BionicFrogEye(640, 480)
    xfeat_engine = XFeatEngine("/home/zhz/fsd-car/model/xfeat_640x640.onnx")

    world.reset()
    world.play()

    print("⏳ [NEXUS 探针] 正在进行 RTX 渲染管道 GPU 预热...")
    for _ in range(30):
        world.step(render=True)
    print("✅ [NEXUS 探针] 预热完毕，进入 Mock 并网闭环测试。")

    tick = 0
    try:
        for step in range(100):
            t_start = time.perf_counter()
            
            # 1. 物理步进
            world.step(render=True)
            tick += 1
            
            # 2. 抓取图像
            rgb_raw = rgb_annotator.get_data()
            if rgb_raw is not None:
                rgb_frame = rgb_raw
                if len(rgb_frame.shape) == 3 and rgb_frame.shape[2] == 4:
                    rgb_frame = rgb_frame[:, :, :3]
                if rgb_frame.dtype == np.float32 or rgb_frame.dtype == np.float64:
                    rgb_frame = (rgb_frame * 255.0).astype(np.uint8)

                # 3. 运行仿生眼避障 (测试是否在此处假死)
                F_x, F_y, _ = frog_eye.process_frame(rgb_frame)
                
                # 4. 运行 XFeat 特征提取 (测试每 10 帧触发一次是否发生卡死)
                if tick % 10 == 0:
                    _ = xfeat_engine.extract(rgb_frame, top_k=200)

            elapsed = (time.perf_counter() - t_start) * 1000.0
            print(f"Frame: {tick:<4} | 单帧耗时: {elapsed:.2f} ms | 避障力: Fx={F_x:.3f}, Fy={F_y:.3f}")
            
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
        print("🔌 探针安全卸载完毕。")

if __name__ == "__main__":
    main()
