# -*- coding: utf-8 -*-
import sys
import os

# 🛡️ 架构师 2026 终极自愈：在 SimulationApp 启动前，强行将 75G 本地物理资产并网！
os.environ["ISAAC_ASSET_ROOT"] = "/run/media/zhz/数据/isaac_assets"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics
import omni.usd
from isaacsim.core.utils.stage import open_stage

print("=" * 80)
print("🛰️  NEXUS - USD 空间场景与物理骨骼逆向探针 [本地资产并网版] 已启动！")
print("=" * 80)

fsd_assets_dir = "/home/zhz/fsd-car/assets"
# 🛡️ 对齐至能完整显示资产的根级 USD 路径
usd_path = os.path.join(fsd_assets_dir, "fsd_car_racetrack.usd")
print(f"🎯 正在解析目标 USD 场景: {usd_path}")

if not os.path.exists(usd_path):
    print(f"❌ 致命错误：找不到物理场景文件 -> {usd_path}")
    simulation_app.close()
    sys.exit(1)

open_stage(usd_path=usd_path)
stage = omni.usd.get_context().get_stage()

if not stage:
    print("❌ 致命错误：无法获取 USD 舞台句柄！")
    simulation_app.close()
    sys.exit(1)

print("✓ 场景载入成功！开始遍历整个空间结构树...")

all_articulation_roots = []
potential_cars = []

for prim in stage.Traverse():
    prim_path = str(prim.GetPath())
    typename = prim.GetTypeName()
    applied_schemas = prim.GetAppliedSchemas()
    
    has_physx_articulation = any("Articulation" in schema for schema in applied_schemas)
    
    if has_physx_articulation:
        all_articulation_roots.append((prim_path, typename, applied_schemas))
        
    lower_path = prim_path.lower()
    if "jetbot" in lower_path or "car" in lower_path or "vehicle" in lower_path:
        potential_cars.append((prim_path, typename, applied_schemas))

print("\n📈 【诊断结果 1：当前场景中所有被物理引擎识别的 Articulation 骨骼】:")
if all_articulation_roots:
    for p, t, s in all_articulation_roots:
        print(f"  -> 📍 路径: {p:<30} | 类型: {t:<15} | 物理 API: {s}")
else:
    print("  ⚠️ 警告：当前场景树中没有找到任何被物理引擎激活的 Articulation 骨骼！")

print("\n📈 【诊断结果 2：当前场景中所有匹配机器人关键字 (jetbot/car) 的原始 Prim】:")
if potential_cars:
    for p, t, s in potential_cars:
        print(f"  -> 📍 路径: {p:<30} | 类型: {t:<15} | 物理 API: {s}")

target_path = "/Root/jetbot"
target_prim = stage.GetPrimAtPath(target_path)
print(f"\n🔍 【诊断结果 3：对 '{target_path}' 的物理状态进行深度显微镜审计】:")
if target_prim.IsValid():
    print(f"  - 是否存在: ✅ 是")
    print(f"  - 图元类型: {target_prim.GetTypeName()}")
    print(f"  - 已应用 API 面板: {target_prim.GetAppliedSchemas()}")
    print(f"  - 子节点级联树:")
    for child in target_prim.GetChildren():
        print(f"    -> 🔹 子节点: {child.GetName():<15} | 类型: {child.GetTypeName()}")
else:
    print(f"  - 是否存在: ❌ 否")

simulation_app.close()
print("=" * 80)
