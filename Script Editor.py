# -*- coding: utf-8 -*-
# 🛡️ FSD-Car / Isaac Sim 6.0 Jetbot 终极纯净闭环起跑脚本
import asyncio
import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot

# 🛡️ 核心引入：动作令牌与原生相机 [cite: 1.4.1]
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
import carb

def kill_zombie_async_tasks():
    """
    🛡️ 内存自愈守卫：强行杀死残留的僵尸协程，释放被锁死的 UsdStage 引用
    """
    try:
        loop = asyncio.get_running_loop()
        tasks = [t for t in asyncio.all_tasks(loop) if not t.done()]
        for t in tasks:
            coro_name = str(t.get_coro())
            if "run_fsd_bionic_loop" in coro_name or "script_" in coro_name:
                t.cancel()
                carb.log_warn(f"[FSD MEMORY] Successfully terminated zombie coroutine: {coro_name}")
    except RuntimeError:
        pass

async def run_fsd_bionic_loop():
    # 1. 强行停止时间线以释放物理场景锁 [cite: 2.1.5]
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        carb.log_warn("[FSD LIFECYCLE] Active timeline detected, stopping to release physics lock...")
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    # 2. 彻底清除上一次残留的 World 脏单例
    if World.instance():
        World.instance().clear_instance()

    # 3. 探针对齐：强行指定物理场路径为最根部的 /PhysicsScene (100% 对齐探针 v3.0 绝对路径)
    world = World(stage_units_in_meters=1.0, physics_prim_path="/PhysicsScene")

    # 4. 接管 /Root/jetbot (必须在 reset 之前实例化，以对齐物理引擎初始化契约)
    car_path = "/Root/jetbot"
    stage = omni.usd.get_context().get_stage()
    
    if not stage.GetPrimAtPath(car_path):
        carb.log_error(f"[FSD ERROR] Vehicle prim not found at: {car_path}")
        return

    # 将小车注册到场景中，参与物理引擎的 reset 初始化
    car = Robot(prim_path=car_path, name="fsd_car")
    world.scene.add(car)

    # 5. 直接接管原生自带相机 /Root/jetbot/rgb_camera
    camera_path = f"{car_path}/rgb_camera"
    camera = Camera(prim_path=camera_path, name="bionic_retina", resolution=(640, 480))
    camera.initialize()

    # 6. 使用 6.0 终极异步初始化函数，彻底将 _physics_context 唤醒！ [cite: 3.3.2]
    await world.initialize_simulation_context_async()

    # 7. 此时小车已安全注册在场景中，重置操作将自动激活并分配其底层 ArticulationController C++ 句柄
    await world.reset_async()

    # 8. 物理时间线起跑 (此时物理上下文和小车控制器均已安全初始化)
    await world.play_async()
    
    # 手动激活小车的 C++ 控制器 [cite: 1.2.2]
    car.initialize()
    carb.log_warn("[FSD] Jetbot and Camera initialized successfully! Entering 30Hz loop...")

    # Jetbot 真实差速物理几何参数
    L = 0.1125  # 轮距 (meters)
    R = 0.03    # 轮半径 (meters)

    # 9. 异步仿真循环
    for step in range(500):
        # -----------------------------------------------------------------
        # 🛡️ 规控指令下发：前进速度 0.15 m/s，转弯角速度 0.3 rad/s (优雅地向左转弯)
        # -----------------------------------------------------------------
        target_v = 0.15
        target_w = 0.3

        # 差速逆解 (🛡️ 极性标定对齐：两侧均保持正极性，对齐 Jetbot 官方物理契约)
        v_left = (target_v - target_w * L / 2.0) / R
        v_right = (target_v + target_w * L / 2.0) / R

        # 写入车轮
        car.apply_action(ArticulationAction(joint_velocities=[v_left, v_right]))

        # 抓取相机图像
        rgb_frame = camera.get_rgb()
        
        if step % 50 == 0:
            if rgb_frame is not None:
                carb.log_warn(f"[FSD] Camera capture ok - Shape: {rgb_frame.shape}, Step: {step}/500, Wheel Speeds: [{v_left:.2f}, {v_right:.2f}]")
            else:
                carb.log_warn(f"[FSD] Camera warming up... Step: {step}/500")

        # 步进一帧，让出控制权
        await omni.kit.app.get_app().next_update_async()

    carb.log_warn("[FSD] Simulation finished. Safe braking applied.")
    # 刹车
    car.apply_action(ArticulationAction(joint_velocities=[0.0, 0.0]))
    world.pause()

# 🛡️ 架构师守卫：启动前强制清理一次内存中的残留协程
kill_zombie_async_tasks()

# 10. 派发异步任务
asyncio.ensure_future(run_fsd_bionic_loop())