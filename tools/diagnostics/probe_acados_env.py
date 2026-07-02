# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
import ctypes
import os


def probe():
    print("=" * 70)
    print("🛰️  FSD-Car & Acados 动态库装载链深度检测探针")
    print("=" * 70)

    # 1. 检查 ACADOS_SOURCE_DIR 变量
    acados_source = os.environ.get(
        "ACADOS_SOURCE_DIR", "/home/zhz/fsd-car/simulation-env/acados"
    )
    print(f"📌 [环境变量] ACADOS_SOURCE_DIR -> {acados_source}")
    if not os.path.exists(acados_source):
        print("❌ [路径失效] 找不到指定的 acados 物理目录！")
        return

    # 2. 检测动态链接库完整性
    lib_dir = os.path.join(acados_source, "lib")
    print(f"📌 [动态库目录] 正在扫描: {lib_dir}")

    required_libs = ["libacados.so", "libqpOASES_e.so", "libblasfeo.so", "libhpipm.so"]
    missing_libs = []

    if os.path.exists(lib_dir):
        files_in_lib = os.listdir(lib_dir)
        for lib in required_libs:
            match = [f for f in files_in_lib if f.startswith(lib)]
            if match:
                print(f"  ✅ 发现物理库: {match[0]}")
            else:
                print(f"  ❌ 缺失核心依赖库: {lib}")
                missing_libs.append(lib)
    else:
        print(f"❌ [致命] 找不到 lib 文件夹: {lib_dir}")
        return

    # 3. 检查 LD_LIBRARY_PATH 环境变量并网状态
    ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    print(f"📌 [环境变量] LD_LIBRARY_PATH -> {ld_path}")
    is_joined = lib_dir in ld_path
    print(
        f"  💡 并网评估: {'✅ acados 路径已并网' if is_joined else '⚠️ 未检测到并网 (会导致 ld-linux 加载失败)'}"
    )

    # 4. 执行 ctypes 装载实验
    print("\n⚡ [符号沙盘演练] 正在尝试通过操作系统 C-Linker 装载动态库...")
    if not missing_libs:
        # 尝试直接装载依赖库以对全局符号表进行内存预热
        for lib in ["libqpOASES_e.so", "libblasfeo.so", "libhpipm.so"]:
            lib_path = os.path.join(lib_dir, lib)
            try:
                ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
                print(f"  💎 预装载 {lib} -> ✅ 成功入驻进程空间")
            except Exception as e:
                print(f"  ⚠️ 预装载 {lib} -> ❌ 失败: {e}")

        # 核心加载测试
        libacados_path = os.path.join(lib_dir, "libacados.so")
        try:
            ctypes.CDLL(libacados_path)
            print("  🏆 [终极载入测试] libacados.so -> 🎉 成功！系统已完全自愈！")
        except Exception as e:
            print(f"  🔥 [终极载入测试] libacados.so -> ❌ 失败，底层链接断链: {e}")
    else:
        print("❌ 核心动态库文件残缺，请在 simulation-env 目录下重新编译 acados 源码。")

    print("=" * 70)


if __name__ == "__main__":
    probe()
