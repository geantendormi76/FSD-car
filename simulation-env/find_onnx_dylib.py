#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛰️  NEXUS - ONNX Runtime 物理库主动装载审计探针 (Active Runtime Probe)
设计哲学: 物理扫盘 + 运行时逆向逼问双重机制 | 拒绝任何路径猜测
使用规范: 必须通过 /home/zhz/isaacsim/python.sh 启动
=================================================================
"""

import os
import sys

print("=" * 80)
print("🛰️  NEXUS - ONNX Runtime 物理库主动装载审计探针已启动")
print("========================================================")

# --- 核心机制 1：主动调用 Python 运行时进行逆向逼问 ---
print("🔍 正在审讯 Python 运行时以检索 onnxruntime 库...")
try:
    import onnxruntime
    # 逆向追踪其 site-packages 安装物理路径
    pkg_file = onnxruntime.__file__
    pkg_dir = os.path.dirname(pkg_file)
    
    # 根据标准 Python Wheel 规范，动态库通常存放在 capi 目录下
    candidate_so = os.path.join(pkg_dir, "capi", "libonnxruntime.so")
    
    if os.path.exists(candidate_so):
        size_mb = os.path.getsize(candidate_so) / (1024 * 1024)
        print(f"  -> 🟢 [运行时逆向成功] 发现 Python 自带的高保真库！")
        print(f"     绝对路径: {candidate_so}")
        print(f"     文件大小: {size_mb:.2f} MB")
        print("-" * 80)
        print("🏆 诊断自愈命令：")
        print(f"  export ORT_DYLIB_PATH=\"{candidate_so}\"")
        print("  cargo run -p showcase --release --bin perception_sandbox")
        print("=" * 80 + "\n")
        sys.exit(0)
except ImportError:
    print("  -> ⚠️  [运行时空白] 当前 python.sh 的物理环境中未安装 onnxruntime 包。")

# --- 核心机制 2：全盘物理树扫盘 (备份防线) ---
print("🔍 正在对物理磁盘进行剪枝遍历扫描...")
target_file = "libonnxruntime.so"
search_dirs = ["/home/zhz/fsd-car", "/home/zhz/isaacsim", "/usr/lib", "/usr/local/lib"]
found_paths = []

for root_dir in search_dirs:
    if not os.path.exists(root_dir):
        continue
    for root, dirs, files in os.walk(root_dir):
        dirs[:] = [d for d in dirs if d not in ['.git', 'target', '.venv', '__pycache__', 'build']]
        if target_file in files:
            full_path = os.path.join(root, target_file)
            try:
                found_paths.append(full_path)
            except:
                pass

if found_paths:
    print(f"  -> 🟢 [磁盘扫描成功] 发现库：{found_paths[0]}")
    print("-" * 80)
    print("🏆 诊断自愈命令：")
    print(f"  export ORT_DYLIB_PATH=\"{found_paths[0]}\"")
    print("  cargo run -p showcase --release --bin perception_sandbox")
    sys.exit(0)

# --- 核心机制 3：两线全部落空，给出最科学的自愈药方 ---
print("-" * 80)
print("❌ 物理审计结论：你的系统及仿真环境目前处于【动态库绝对真空】状态。")
print("💡 终极自愈药方（对症下药）：")
print("  请使用 Isaac Sim 官方指定的 Python 环境一键下载该动态库包体：")
print("\n  /home/zhz/isaacsim/python.sh -m pip install onnxruntime")
print("\n  下载完成后，再次运行此探针，它将 100% 自动捕获并输出正确的 export 绑定路径！")
print("=" * 80 + "\n")
