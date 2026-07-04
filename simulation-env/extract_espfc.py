#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛰️  NEXUS - esp-fc 飞控核心控制代码抽取器 (Heredoc SOTA 版)
设计哲学: 拿来主义精准抽取 | 剥离冗余驱动 | 格式化并网
=================================================================
"""

import os
import sys

print("========================================================")
print("🛰️  NEXUS - esp-fc 飞控核心控制代码抽取器已启动")
print("========================================================")

# 1. 定义源路径与输出路径 (对齐最新 Ubuntu 26.04 系统路径)
src_root = "/run/media/zhz/数据/开发积累/1、github项目/自动驾驶相关/esp-fc"
output_path = "/run/media/zhz/数据/开发积累/1、github项目/自动驾驶相关/esp_fc_core_extracted.md"

if not os.path.exists(src_root):
    print(f"❌ 致命错误：找不到 esp-fc 物理路径 -> {src_root}")
    sys.exit(1)

# 2. 精准定义我们需要的核心算法及数学库资产 (高内聚，去噪点)
target_files = [
    "lib/AHRS/src/helper_3dmath.h",
    "lib/AHRS/src/Mahony.h",
    "lib/AHRS/src/Mahony.cpp",
    "lib/Espfc/src/Control/Pid.h",
    "lib/Espfc/src/Control/Pid.cpp",
    "lib/Espfc/src/Utils/Filter.h",
    "lib/Espfc/src/Utils/Filter.cpp",
    "lib/Espfc/src/Utils/Timer.h",
    "lib/Espfc/src/Utils/Timer.cpp",
]

markdown_buffer = []
markdown_buffer.append("# 🛰️ NEXUS - esp-fc 飞控核心控制算法提取白皮书\n")
markdown_buffer.append("> 本文档由 Python 物理探测器自动生成。提取了最适合 FSD-car 地面小车复用的硬实时控制、数学与滤波核心。\n\n")

print(f"🔍 正在扫盘并提取核心源文件...")
extracted_count = 0

for target in target_files:
    full_path = os.path.join(src_root, target)
    if os.path.exists(full_path):
        extracted_count += 1
        print(f"  -> 🟢 [提取成功] {target}")
        
        # 写入 Markdown 标题
        markdown_buffer.append(f"## 📁 核心源文件: `{target}`\n")
        markdown_buffer.append(f"- **物理路径**: `{full_path}`\n\n")
        
        # 判断代码语言类型进行语法高亮
        lang = "cpp"
        if target.endswith(".h") or target.endswith(".hpp"):
            lang = "cpp"
            
        markdown_buffer.append(f"```{lang}\n")
        
        # 读取内容并无损写入
        with open(full_path, "r", encoding="utf-8", errors="ignore") as f:
            markdown_buffer.append(f.read())
            
        markdown_buffer.append("\n```\n\n")
    else:
        print(f"  -> ⚠️  [文件缺失] 找不到 {target}，跳过")

# 3. 写入硬盘
try:
    with open(output_path, "w", encoding="utf-8") as out:
        out.writelines(markdown_buffer)
    print("-" * 80)
    print(f"🏆 提取圆满完成！共提取了 {extracted_count} 个核心控制算法源文件。")
    print(f"💾 输出文件已写入: {output_path}")
    print("💡 下一步：请将该 md 文件上传给我，我们开始对控制器进行“毫米级”细节打磨！")
    print("=" * 80 + "\n")
except Exception as e:
    print(f"❌ 写入失败: {e}")

