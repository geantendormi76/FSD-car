#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛰️  NEXUS - DORA 机器人中间件核心算法抽取器 (Heredoc SOTA 版)
设计哲学: 精准抓取零拷贝 SHM 核心 | 提取 Arrow 转换边界 | 纯净并网
=================================================================
"""

import os
import sys

print("========================================================")
print("🛰️  NEXUS - DORA 核心算法提取器已启动")
print("========================================================")

# 1. 定义源路径与输出路径 (对齐最新 Ubuntu 26.04 路径)
src_root = "/run/media/zhz/数据/开发积累/1、github项目/自动驾驶相关/DORA（面向数据流的机器人架构）/dora"
output_path = "/run/media/zhz/数据/开发积累/1、github项目/自动驾驶相关/dora_core_extracted.md"

if not os.path.exists(src_root):
    print(f"❌ 致命错误：找不到 DORA 物理路径 -> {src_root}")
    sys.exit(1)

# 2. 精准定义高内聚、最值得 FSD 小车并网借鉴的 6 大核心文件
target_files = [
    "apis/rust/node/src/lib.rs",
    "apis/rust/node/src/node/mod.rs",
    "apis/rust/node/src/node/arrow_utils.rs",
    "apis/rust/node/src/event_stream/event.rs",
    "libraries/arrow-convert/src/lib.rs",
    "apis/python/node/dora/cuda.py",
]

markdown_buffer = []
markdown_buffer.append("# 🛰️ NEXUS - DORA 机器人中间件核心算法白皮书\n")
markdown_buffer.append("> 本文档由 Python 物理探测器自动生成。提取了 DORA 最核心的零拷贝、共享内存及 Arrow 对齐通信资产。\n\n")

print(f"🔍 正在扫盘并提取 DORA 核心源文件...")
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
        lang = "rust"
        if target.endswith(".py"):
            lang = "python"
            
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
    print(f"🏆 提取圆满完成！共提取了 {extracted_count} 个核心通信算法源文件。")
    print(f"💾 输出文件已写入: {output_path}")
    print("💡 下一步：请将该 md 文件上传给我，我们开始对 DORA 分布式神经通路进行无损细节审计！")
    print("=" * 80 + "\n")
except Exception as e:
    print(f"❌ 写入失败: {e}")

