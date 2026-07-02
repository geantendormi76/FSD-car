# -*- coding: utf-8 -*-
import sys
import os
import importlib

print("=" * 80)
print("🛰️  NEXUS - NVIDIA Isaac Sim Python 命名空间深度逆向探针")
print("=" * 80)

# 1. 打印当前的运行环境
print(f"📌 [Python 解释器]: {sys.executable}")
print(f"📌 [当前工作目录]: {os.getcwd()}")

# 2. 核心诊断：审计 sys.path 优先级，看看是谁污染了搜索链
print("\n🔍 [审计 sys.path 搜索链优先级]:")
for idx, path in enumerate(sys.path):
    print(f"  [{idx:02d}] -> {path}")

# 3. 核心诊断：审计环境变量污染
print("\n🔍 [审计系统环境变量]:")
for var in ["PYTHONPATH", "LD_LIBRARY_PATH", "PATH"]:
    print(f"  {var} = {os.environ.get(var, '❌ 未设置')}")

# 4. 逆向核心：探测 omni.kit.asset_converter 的物理加载源头
def probe_module(mod_name):
    print(f"\n⚡ [深度探测模块]: {mod_name}")
    try:
        # 尝试导入
        mod = importlib.import_module(mod_name)
        # 获取其物理路径（如果是 Namespace 包，通常会返回多个路径或 None）
        mod_path = getattr(mod, "__path__", "Namespace / Unknown Location")
        print(f"  ✅ 成功加载！")
        print(f"  📍 物理加载源头 -> {mod_path}")
        
        # 打印其暴露的公开属性，检查是否存在 AssetConverterContext
        attrs = [attr for attr in dir(mod) if not attr.startswith("_")]
        print(f"  📦 暴露的公开属性数: {len(attrs)}")
        print(f"  📋 公开属性列表: {attrs[:15]} ...")
    except Exception as e:
        print(f"  ❌ 加载失败！报错信息: {e}")

probe_module("omni.kit.asset_converter")
probe_module("omni.kit.tool.asset_importer")

print("\n" + "=" * 80)
