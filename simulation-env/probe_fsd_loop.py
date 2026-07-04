
# -*- coding: utf-8 -*-

"""
=================================================================
🛰️ NEXUS - FSD-car 纯净物理与渲染时序诊断探针 (Zero-Dora 级联版)
设计哲学: 彻底脱离 DORA 依赖 | 独占式物理时钟审计 | 排除通信噪点
使用规范: 必须通过 /home/zhz/isaacsim/python.sh 启动
=================================================================
"""

import os
import sys
import time
import struct
import numpy as np

# 1. 强行激活 RTX 离屏相机渲染配置
os.environ["ENABLE_CAMERAS"] = "1"
os.environ["ISAAC_ASSET_ROOT"] = "/run/media/zhz/数据/isaac_assets"

# 强制静默冗余日志
sys.argv.extend(["--/log/level=error", "--/log/fileLogLevel=error"])

# 2. 从官方规范引入 2026 核心 Application 启动器
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # 开启 GUI 窗口以便观测

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import open_stage
import omni.replicator.core as rep

print("\n" + "="*80)
print("🎯 [NEXUS 探针] 正在载入 75G 级物理赛道 USD 资产...")
open_stage(usd_path="/home/zhz/fsd-car/assets/fsd_car_racetrack.usd")

# 3. 锁定 100Hz 物理时钟
world = World(physics_dt=0.01, rendering_dt=0.01, backend="numpy")
car = Articulation(prim_paths_expr="/Root/jetbot", name="jetbot")
world.scene.add(car)

# ===========================================================================
# ⚙️ 诊断参数配置 (🛡️ 拒绝死码与临时注释，重构为优雅的功能开关)
# ===========================================================================
# True  -> 开启相机渲染与 GPU-to-CPU 显存拷贝 (用于测试包含 Replicator 的完整物理性能)
# False -> 关闭相机拷贝 (测试纯物理推演极限速度)
TEST_GPU_COPY_OVERHEAD = False

# 4. 根据配置动态挂载离屏渲染管线
if TEST_GPU_COPY_OVERHEAD:
    camera_path = "/Root/jetbot/chassis/rgb_camera/jetbot_camera"
    render_product = rep.create.render_product(camera_path, (640, 480))
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([render_product])

world.reset()
world.play()

print("⏳ [NEXUS 探针] 正在对 GPU 渲染管线进行 60 帧预热...")
for _ in range(60):
    world.step(render=True)
print("✅ [NEXUS 探针] GPU 预热完毕，进入 100 帧物理与渲染时序压测...\n")

print(f"{'Tick':<6} | {'Step_Time(ms)':<14} | {'Data_Fetch_Time(ms)':<18} | {'Status'}")
print("-" * 65)

tick = 0
time_stats = []

try:
    for _ in range(100):
        tick += 1
        t_start = time.perf_counter()
        
        # A. 物理与图形步进
        world.step(render=True)
        t_after_step = time.perf_counter()
        
        # B. 【关键测试点】不执行 get_data() 时的空转表现
        # 我们模拟获取数据并打包成 8 字节的计算延迟
        dummy_f_x, dummy_f_y = 0.1, 0.2
        _ = struct.pack("<ff", dummy_f_x, dummy_f_y)
        t_after_fetch = time.perf_counter()
        
        # 计算各阶段物理延迟
        step_duration = (t_after_step - t_start) * 1000.0
        fetch_duration = (t_after_fetch - t_after_step) * 1000.0
        total_duration = step_duration + fetch_duration
        time_stats.append(total_duration)
        
        print(f"{tick:<6} | {step_duration:<14.2f} | {fetch_duration:<18.4f} | 🟢 RunLoop Active")
        
        # 频率守卫：模拟 100Hz 节拍器的强制等待
        time.sleep(max(0, 0.01 - (total_duration / 1000.0)))

    # 计算统计指标
    avg_time = sum(time_stats) / len(time_stats)
    print("\n" + "="*80)
    print("📊 [NEXUS 探针报告] 物理时序审计完成：")
    print(f"  -> 单步平均总时间（含 GPU 渲染）: {avg_time:.2f} ms")
    print(f"  -> 极限物理推演频率（理想状态）: {1000.0 / avg_time:.1f} Hz")
    print("  -> 诊断结论：")
    if avg_time <= 11.0:
        print("     🟢 [绿通] 仿真物理与渲染管线耗时极低，系统具备运行 100Hz 控制环的硬实时能力！")
    else:
        print("     ⚠️  [阻塞] 纯物理渲染耗时依然超标，请检查显卡驱动性能或降低仿真窗口分辨率。")
    print("="*80 + "\n")

except Exception as e:
    import traceback
    print(f"❌ 探针运行中捕获到异常: {e}")
    traceback.print_exc()
finally:
    simulation_app.close()
    print("🔌 探针已安全退出。")
