#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛰️  NEXUS - D_CUDA_EP 运行时异常阻断拦截探针 (Interrogation Probe)
设计哲学: 强行拦截 CUDA 异常 | 打印最底层链接报错 | 揪出真正缺失的库
使用规范: 必须通过 /home/zhz/isaacsim/python.sh 启动
=================================================================
"""

import os
import sys
import traceback

print("=" * 80)
print("🛰️  NEXUS - CUDA 运行时异常审计探针已激活")
print("========================================================")

# 打印当前动态链接器能看到的 RPATH/LD 变量，进行首轮审计
print(f"📋 当前进程 LD_LIBRARY_PATH:\n   -> {os.environ.get('LD_LIBRARY_PATH', 'None')}\n")

try:
    import onnxruntime as ort
    print(f"📋 当前 ONNX Runtime 版本: {ort.__version__}")
    print(f"📋 当前可用 Providers 队列: {ort.get_available_providers()}\n")
    
    print("🔥 正在强行初始化 CUDA 独占会话以拦截底层 C++ 异常...")
    print("-" * 75)
    
    # 强制单向加载 CUDA 阻断测试，拦截其 silent fallback 行为
    session = ort.InferenceSession(
        "/home/zhz/fsd-car/model/xfeat_640x640.onnx", 
        providers=['CUDAExecutionProvider']
    )
    
    print("-" * 75)
    print("🟢 [惊人结论] CUDA 运行时实际上已经完全初始化成功！")
    print(f"   当前激活提供商: {session.get_providers()}")
    
except Exception as e:
    print("-" * 75)
    print("❌ [阻断捕获成功] CUDA 运行时初始化失败！")
    print(f"   错误代码: {e}\n")
    print("🚨 【最底层 C++ 链接器崩溃堆栈】:")
    traceback.print_exc()
    print("-" * 75)
    print("💡 诊断说明：")
    print("   请仔细检查上述堆栈。通常情况下，这会暴露出：")
    print("   1. 缺失某一个具体的 CUDA 库（如 libcublasLt.so.12 或 libcudnn.so.8）")
    print("   2. 或者是你下载的 onnxruntime-gpu 1.27.0 编译时使用了过新的 CUDA 版本（如 CUDA 13.x），与当前系统/Isaac Sim 的 CUDA 12.x 发生版本撕裂！")

print("=" * 80 + "\n")
