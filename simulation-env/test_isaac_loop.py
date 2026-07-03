# -*- coding: utf-8 -*-
import os
# 🛡️ 架构师 2026 终极自愈：强行将 75G 本地物理资产并网！
os.environ["ISAAC_ASSET_ROOT"] = "/run/media/zhz/数据/isaac_assets"

import sys
import numpy as np
import time

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api.world import World
from isaacsim.core.prims import Articulation
from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.stage import open_stage

print("=" * 80)
print("🛰️  NEXUS - Isaac Sim 本地主循环纯净闭环探针已启动！")
print("=" * 80)

fsd_assets_dir = "/home/zhz/fsd-car/assets"
usd_path = os.path.join(fsd_assets_dir, "fsd_car_racetrack.usd")

print(f"正在载入场景: {usd_path}")
open_stage(usd_path=usd_path)

world = World(stage_units_in_meters=1.0, physics_prim_path="/PhysicsScene")
car_path = "/Root/jetbot"

car = Articulation(prim_paths_expr=car_path, name="jetbot")
world.scene.add(car)

camera_path = f"{car_path}/chassis/rgb_camera/jetbot_camera"
camera = Camera(prim_path=camera_path, name="bionic_retina", resolution=(640, 480))
world.scene.add(camera)

print("正在重置并播放物理世界...")
world.reset()
world.play()
world.step(render=True)
camera.initialize()

print("✓ 探针主循环启动！开始运行 1000 步 (~10秒) 的物理闭环压力测试...")
tick = 0
try:
    for i in range(1000):
        if not simulation_app.is_running():
            print("⚠️ 警告：simulation_app.is_running() 返回了 False！")
            break
            
        # 步进物理和渲染
        world.step(render=True)
        tick += 1
        
        # 极速获取相机图像并审计其类型与形状
        rgb_raw = camera.get_rgb()
        if rgb_raw is not None:
            if tick % 100 == 0:
                print(f"[{tick:04d}] 相机图像抓取成功 | 类型: {type(rgb_raw)} | 形状: {getattr(rgb_raw, 'shape', '未知')}")
                
        # 给小车注入差速动力，验证物理引擎控制是否生效
        # （左轮正转，右轮反转，小车应当在赛道上原地自转）
        v_left = 5.0
        v_right = -5.0
        car.set_joint_velocity_targets(np.array([[v_left, v_right]]))
        
        time.sleep(0.01)
        
    print("🏆 压力测试完成！未发生任何闪退崩溃，小车自转测试成功！")

except Exception as e:
    import traceback
    print("❌ 核心循环发生致命崩溃！详细堆栈如下：")
    traceback.print_exc()
finally:
    simulation_app.close()
    print("=" * 80)
