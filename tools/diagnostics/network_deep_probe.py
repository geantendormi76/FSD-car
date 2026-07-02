# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
import os
import socket
import subprocess
import sys
import time


def get_wsl_gateway_ip():
    try:
        cmd = "ip route | grep default | awk '{print $3}'"
        ip = subprocess.check_output(cmd, shell=True, text=True).strip()
        return ip if ip else None
    except Exception:
        return None


def get_mdns_hostname():
    try:
        cmd = "hostname"
        hostname = subprocess.check_output(cmd, shell=True, text=True).strip()
        # Windows mDNS 主机名通常匹配 WSL 实例的 hostname + .local
        return f"{hostname}.local"
    except Exception:
        return None


def run_server():
    print("=" * 80)
    print("🛰️  NEXUS BIM Portal - 跨 OS 物理连通性极速诊断服务端 (Windows)")
    print("=" * 80)

    # 打印 Windows 所有本地网卡 IP 矩阵，揪出真实的局域网和虚拟网卡 IP
    hostname = socket.gethostname()
    print(f"🖥️  Windows 宿主机名称: {hostname}")
    try:
        ips = socket.gethostbyname_ex(hostname)[2]
        print("🌐 Windows 当前活跃网卡 IP 矩阵:")
        for ip in ips:
            print(f"  -> {ip}")
    except Exception as e:
        print(f"⚠️ 无法获取网卡列表: {e}")

    port = 17449
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        s.bind(("0.0.0.0", port))
        s.listen(5)
        print(
            f"\n🟢 [监听中] 已成功绑定 0.0.0.0:{port}。正在全力侦听来自 WSL2 的 TCP SYN 信号..."
        )
    except Exception as e:
        print(f"❌ 绑定失败 (端口已被占用或无权限): {e}")
        return

    s.settimeout(1.0)
    try:
        while True:
            try:
                conn, addr = s.accept()
                print("\n🎉 【连接成功建立!!!】 捕获到来自 WSL2 的物理握手信号:")
                print(
                    f"  -> 源 IP: {addr[0]} | 源端口: {idx_port if 'idx_port' in locals() else addr[1]}"
                )
                conn.sendall(b"NEXUS_TCP_CONNECTED_OK\n")
                conn.close()
            except socket.timeout:
                continue
    except KeyboardInterrupt:
        print("\n🛑 服务端退出")
    finally:
        s.close()


def run_client():
    print("=" * 80)
    print("🛰️  NEXUS BIM Portal - 跨 OS 物理连通性极速诊断客户端 (WSL2)")
    print("=" * 80)

    # 1. 自动提取候选探测目标
    gateway_ip = get_wsl_gateway_ip()
    mdns_host = get_mdns_hostname()

    candidates = []
    if gateway_ip:
        candidates.append(("WSL 默认网关 IP", gateway_ip))
    if mdns_host:
        candidates.append(("Windows mDNS 主机名", mdns_host))
    candidates.append(("常见默认网关", "172.23.176.1"))

    port = 17449
    print("🕵️‍♂️ 规划探测矩阵 (多路径探测，防止 NAT 路由失效):")
    for name, addr in candidates:
        print(f"  -> [{name}]: {addr}:{port}")

    print("\n⚡ [开始发射高频 TCP SYN 探测包...]")
    for name, addr in candidates:
        print(f"\n🛰️ 正在测试路径 [{name}] -> {addr}...")

        # 尝试进行 DNS/mDNS 解析
        try:
            target_ip = socket.gethostbyname(addr)
            print(f"  -> 解析成功! 映射真实 IP 为: {target_ip}")
        except Exception as e:
            print(f"  ❌ 域名解析失败: {e}")
            target_ip = addr

        # 执行原生 TCP 握手
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)  # 2秒判定

        start_time = time.time()
        try:
            s.connect((target_ip, port))
            elapsed = (time.time() - start_time) * 1000
            print(f"  ✅ 【连通大获成功!!!】耗时: {elapsed:.2f} ms")

            # 接收服务端确认
            data = s.recv(1024)
            print(f"  💬 收到 Windows 探针确认: {data.decode().strip()}")
            s.close()
            print(f"\n💡 终极自愈药方：请立刻将你的配置文件中的连接 IP 修改为: {addr}")
            print("=" * 80)
            return  # 一旦有通的，直接结束
        except socket.timeout:
            print("  ❌ 【连接超时 (Timeout)】耗时: > 2000 ms")
            print(
                "     -> 诊断：物理丢包。防火墙仍在使用隐藏策略拦截（比如 Windows 有第三方安全软件火绒、360、腾讯管家等）。"
            )
        except ConnectionRefusedError:
            print("  ❌ 【连接被拒绝 (Connection Refused)】")
            print(
                "     -> 诊断：物理通路已通，但 Windows 端的 Server 探针没运行，或没绑定 17449 端口。"
            )
        except Exception as e:
            print(f"  ❌ 探测异常: {e}")
        finally:
            s.close()

    print("\n❌ 探测矩阵全部挂盘，物理通道被底层彻底封锁！")
    print("💡 终极自愈诊断建议：")
    print(
        "  1. 请检查你的 Windows 是否开启了代理（如 Clash/V2Ray），尤其是“TUN 模式”或“系统代理”。代理软件会强行将 127.0.0.1 之外的虚拟路由全部吃掉！请彻底关闭它们再测。"
    )
    print(
        "  2. 请在 Windows 控制面板中，临时将 Windows 防火墙整体关闭（公用与专用网络），排除是否属于高级组策略强行拦截。"
    )
    print("=" * 80)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--server":
        # 针对 Windows 端载入 DLL 搜索链以防止 OMNI 报错
        cuda_bin_path = r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.1\bin"
        if os.path.exists(cuda_bin_path):
            try:
                os.add_dll_directory(cuda_bin_path)
            except Exception:
                pass
        run_server()
    else:
        run_client()
