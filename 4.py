#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=================================================================
🛡️ FSD-car: WSL2 + Zsh + Rust 终端与编译性能深度诊断探针
=================================================================
"""

import os
import sys
import time
import subprocess
import socket

def print_header(title):
    print("\n" + "="*60)
    print(f" 🔍 {title}")
    print("="*60)

def test_filesystem():
    print_header("1. 文件系统与虚拟化检测")
    cwd = os.getcwd()
    print(f"当前工作目录: {cwd}")
    
    # 检测是否在 /mnt/ 下
    if cwd.startswith("/mnt/"):
        print("🔴 [致命红线] 检测到项目存储在 Windows 盘区 (如 /mnt/c/ 或 /mnt/d/)！")
        print("   👉 危害：WSL2 访问 Windows 文件需要经过 9P 虚拟文件系统，速度比原生 ext4 慢 10-20 倍。")
        print("   💡 修复建议：请立即将项目移至 Linux 原生家目录下（如：/home/zhz/FSD-car）进行编译！")
    else:
        print("🟢 [安全] 检测到项目已存储在 Linux 原生 ext4 盘区，文件系统路径正常。")

    # 检测 WSL 版本
    try:
        release = os.uname().release
        if "microsoft-standard" in release:
            print("🟢 [确认] 当前运行环境为 WSL2。")
        else:
            print("🟡 [警告] 无法确认是否为 WSL2，可能是 WSL1 或原生 Linux。")
    except Exception:
        pass

def test_io_performance():
    print_header("2. 磁盘 I/O 压力测试 (排查 Windows Defender 拦截)")
    temp_file = "diag_temp_test_file.bin"
    chunk_size = 1024 * 1024 # 1MB
    chunks = 50 # 50MB
    
    data = b"0" * chunk_size
    
    print(f"正在向磁盘高速写入 {chunks}MB 临时数据...")
    start_time = time.time()
    try:
        with open(temp_file, "wb") as f:
            for _ in range(chunks):
                f.write(data)
                f.flush()
                os.fsync(f.fileno())
        elapsed = time.time() - start_time
        speed = chunks / elapsed
        
        print(f"磁盘写入完成！耗时: {elapsed:.2f} 秒 | 平均速度: {speed:.2f} MB/s")
        
        if speed < 80.0:
            print("🔴 [致命红线] 磁盘写入速度极慢（低于 80MB/s）！")
            print("   👉 危害：这几乎 100% 意味着 Windows Defender 在后台强力拦截扫描你的 ext4.vhdx 或临时编译文件。")
            print("   💡 修复建议：请在 Windows 安全中心中，将 `ext4.vhdx` (WSL磁盘文件) 所在目录和整个 Rust 编译 Target 目录加入【排除项】！")
        elif speed < 250.0:
            print("🟡 [中度警告] 磁盘速度一般（低于 250MB/s），Windows Defender 可能存在轻度扫描干扰。")
        else:
            print("🟢 [健康] 磁盘写入速度正常，Windows Defender 干扰较小。")
            
    except Exception as e:
        print(f"❌ 写入测试失败: {e}")
    finally:
        if os.path.exists(temp_file):
            os.remove(temp_file)

def test_git_latency():
    print_header("3. Git 状态延迟测试 (排查 Zsh 回车粘手卡顿)")
    print("正在评估 Zsh 终端回车触发 Git 查询的渲染延迟...")
    
    start_time = time.time()
    try:
        # 执行 git status 并丢弃输出
        subprocess.run(["git", "status", "--porcelain"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elapsed = (time.time() - start_time) * 1000 # 毫秒
        
        print(f"Git 状态查询耗时: {elapsed:.2f} 毫秒")
        
        if elapsed > 100.0:
            print("🔴 [中度红线] `git status` 耗时超过 100 毫秒！")
            print("   👉 危害：这会导致你在 Zsh 中每次按下回车或输入命令时，都会感到明显的粘手和卡顿。")
            print("   💡 修复建议：在你的 `~/.zshrc` 中加入 `DISABLE_UNTRACKED_FILES_DIRTY=\"true\"`，")
            print("              或者使用 `git config --global core.preloadindex true` 来提速。")
        else:
            print("🟢 [健康] Git 查询延迟极低，终端“粘手”与此无关。")
    except Exception as e:
        print(f"❌ Git 测试失败 (可能未初始化 Git 仓): {e}")

def test_network_and_proxy():
    print_header("4. 神经通路与代理延迟测试")
    print("正在排查环境变量中是否存在失效的 Windows 宿主机代理...")
    
    # 检查环境变量
    proxy_vars = {k: v for k, v in os.environ.items() if 'proxy' in k.lower()}
    if proxy_vars:
        print("检测到当前活动的代理设置:")
        for k, v in proxy_vars.items():
            print(f"  {k} = {v}")
    else:
        print("🟢 [安全] 未检测到任何全局代理环境变量。")
        
    # 测试连接中科大 Rust 镜像源
    target_host = "mirrors.ustc.edu.cn"
    print(f"正在测试与中科大 Rust 镜像源 ({target_host}) 的物理通路连接...")
    start_time = time.time()
    try:
        socket.setdefaulttimeout(3)
        socket.gethostbyname(target_host)
        elapsed = (time.time() - start_time) * 1000
        print(f"🟢 [连通] 解析成功！物理延迟: {elapsed:.2f} 毫秒")
    except Exception as e:
        print(f"🔴 [致命红线] 无法解析或连通 Rust 镜像源！")
        print(f"   👉 危害：这意味着你的网络连接超时（可能是代理配置指向了死 IP），Cargo 每次编译前尝试更新索引时都会卡死数分钟。")
        print("   💡 修复建议：检查代理并关闭，或者重置 `/etc/resolv.conf`。")

def test_system_resources():
    print_header("5. WSL 宿主机系统资源评估")
    try:
        # 读取 Load Average
        with open("/proc/loadavg", "r") as f:
            load = f.read().strip()
        print(f"WSL 当前系统负载平均值 (1, 5, 15分钟): {load}")
        
        # 读取内存
        with open("/proc/meminfo", "r") as f:
            mem_lines = f.readlines()
        mem_total = 0
        mem_free = 0
        for line in mem_lines:
            if "MemTotal" in line:
                mem_total = int(line.split()[1]) // 1024 # MB
            if "MemAvailable" in line:
                mem_free = int(line.split()[1]) // 1024 # MB
                
        print(f"WSL 内存分配状态: 剩余可用 {mem_free}MB / 共分配 {mem_total}MB")
        
        if mem_free < 1024:
            print("🔴 [致命红线] WSL 剩余内存严重不足（低于 1GB）！")
            print("   👉 危害：Rust 编译器（尤其是 Release 编译）是内存吞噬者，内存不足会导致严重的 OOM 换页，使电脑卡死。")
            print("   💡 修复建议：在 Windows 的用户目录下创建 `.wslconfig` 文件，为 WSL 增加内存配额（如 `memory=12GB`），并执行 `wsl --shutdown` 重启。")
        else:
            print("🟢 [健康] 物理可用内存充足。")
            
    except Exception as e:
        print(f"❌ 系统资源读取失败: {e}")

if __name__ == "__main__":
    print("=" * 60)
    print("🕵️‍♂️ FSD-car WSL2 物理性能诊断探针开始工作...")
    print("=" * 60)
    
    test_filesystem()
    test_io_performance()
    test_git_latency()
    test_network_and_proxy()
    test_system_resources()
    
    print("\n" + "="*60)
    print("🕵️‍♂️ 诊断结束！请根据上方带有 [🔴 致命红线] 的提示进行针对性修复。")
    print("="*60)