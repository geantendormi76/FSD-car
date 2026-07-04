#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛰️  NEXUS - ONNX Runtime 物理包内胆剖析探针 (Active Package Inspector)
设计哲学: 直接审讯 Python 包内结构 | 检索所有 .so 动态库 | 对症下药
使用规范: 必须通过 /home/zhz/isaacsim/python.sh 启动
=================================================================
"""

import os
import sys

print("=" * 80)
print("🛰️  NEXUS - ONNX Runtime 物理包内胆剖析探针已激活")
print("========================================================")

try:
    import onnxruntime
    # 1. 抓取 onnxruntime 模块在 Python 下的真实物理原点
    pkg_file = onnxruntime.__file__
    pkg_dir = os.path.dirname(pkg_file)
    print(f"🟢 [审讯成功] 已定位 onnxruntime 物理安装包根目录：")
    print(f"   -> {pkg_dir}\n")
    
    print("🔍 开始遍历该包体根目录下所有的动态链接库 (.so) 文件...")
    print("-" * 75)
    
    found_any = False
    # 2. 递归遍历包体，强制寻找所有 .so 文件
    for root, dirs, files in os.walk(pkg_dir):
        for f in files:
            if f.endswith(".so") or "onnxruntime" in f.lower():
                found_any = True
                full_path = os.path.join(root, f)
                try:
                    size_mb = os.path.getsize(full_path) / (1024 * 1024)
                    print(f"  -> 🟢 发现库文件: {f:<30} | 大小: {size_mb:>6.2f} MB | 路径: {full_path}")
                except Exception as e:
                    print(f"  -> ⚠️  发现库文件: {f:<30} (读取失败: {e})")
                    
    print("-" * 75)
    if not found_any:
        print("❌ 警告：在该 Python 包体内竟然没有发现任何 .so 动态链接库！这极不正常。")
    else:
        print("🏆 剖析完成！请观察上述输出中哪个是核心动态链接库，直接将其路径 export 即可！")

except ImportError as e:
    print(f"❌ 审讯失败：当前 python.sh 环境无法 import onnxruntime。报错: {e}")
    print("💡 诊断说明：请确认 /home/zhz/isaacsim/python.sh 与你执行 pip install 的环境是同一个。")

print("=" * 80 + "\n")
