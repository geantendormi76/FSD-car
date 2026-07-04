import socket
import sys

print("="*80)
print("🔍 [NEXUS 探针] 正在审计 127.0.0.1:53290 物理端口状态...")
print("="*80)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    # 尝试绑定端口，如果被 TIME_WAIT 锁定，这里会抛出 OSError
    s.bind(("127.0.0.1", 53290))
    print("  -> 🟢 [自愈成功] 端口 53290 已经完全释放，可以安全执行 `dora up`！")
    s.close()
    sys.exit(0)
except OSError as e:
    print(f"  -> ❌ [物理闭锁] 端口 53290 依然被 Linux 内核锁定在 TIME_WAIT 状态中！")
    print(f"     错误详情: {e}")
    print("  -> 💡 对策: 请在终端静静等待 20~30 秒，让内核释放端口，然后重新测试。")
    s.close()
    sys.exit(1)
