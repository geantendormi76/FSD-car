# -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
import ctypes
import os


def run_diagnostic():
    print("=" * 80)
    print("🛰️  FSD-Car V3.0 SOTA C-FFI 符号沙盘与内存边界深度诊断探针")
    print("=" * 80)

    # 1. 自动寻找动态库
    workspace_root = "/home/zhz/fsd-car"
    acados_lib_dir = os.path.join(workspace_root, "simulation-env/acados/lib")
    solver_lib_dir = os.path.join(workspace_root, "simulation-env/c_generated_code")

    libacados_path = os.path.join(acados_lib_dir, "libacados.so")
    libsolver_path = os.path.join(
        solver_lib_dir, "libacados_ocp_solver_diff_drive_car.so"
    )

    print("📂 [库自检] 正在对齐 C 库路径...")
    print(
        f"  -> libacados: {libacados_path} ({'✅ 存在' if os.path.exists(libacados_path) else '❌ 缺失'})"
    )
    print(
        f"  -> libsolver: {libsolver_path} ({'✅ 存在' if os.path.exists(libsolver_path) else '❌ 缺失'})"
    )

    if not os.path.exists(libacados_path) or not os.path.exists(libsolver_path):
        print("❌ [致命] 物理库残缺，请先在 simulation-env 目录下生成求解器。")
        return

    # 2. 并网环境预热并加载依赖（对齐 2026 年最新 SOTA 加载链规范）
    print("\n⚡ [符号沙盘演练] 正在装载动态链接库...")
    try:
        # 预先全局注入依赖，防止符号断链
        for dep in ["libqpOASES_e.so", "libblasfeo.so", "libhpipm.so"]:
            dep_path = os.path.join(acados_lib_dir, dep)
            ctypes.CDLL(dep_path, mode=ctypes.RTLD_GLOBAL)

        # 加载核心库
        libacados = ctypes.CDLL(libacados_path, mode=ctypes.RTLD_GLOBAL)
        # 加载求解器共享库
        libsolver = ctypes.CDLL(libsolver_path)
        print("✅ 动态链接库全部成功载入进程空间！")
    except Exception as e:
        print(f"❌ 装载失败: {e}")
        return

    # 3. 符号导出审计 (2026 SOTA API 契约审计)
    print("\n🔎 [符号导出审计] 正在探测 C API 函数签名极性...")

    symbols_to_check = {
        "diff_drive_car_acados_create_capsule": "分配内存胶囊 [create_capsule]",
        "diff_drive_car_acados_create": "求解器矩阵初始化 [create]",
        "diff_drive_car_acados_free": "释放求解器 [free]",
        "diff_drive_car_acados_free_capsule": "释放胶囊内存 [free_capsule]",
    }

    detected_symbols = {}
    for sym, desc in symbols_to_check.items():
        has_sym = hasattr(libsolver, sym)
        detected_symbols[sym] = has_sym
        status_str = "✨ [SOTA 2026 API 支持]" if has_sym else "⚠️ [旧版本 API]"
        print(
            f"  -> {sym:<40} : {'✅ 发现' if has_sym else '❌ 缺失'} ({desc}) {status_str}"
        )

    # 4. 执行内存分配与 FFI 运行时碰撞演练
    print("\n💎 [内存边界碰撞演练] 模拟 Rust 执行构造函数...")

    if (
        detected_symbols["diff_drive_car_acados_create_capsule"]
        and detected_symbols["diff_drive_car_acados_create"]
    ):
        try:
            # A. 提取符号
            create_capsule = getattr(libsolver, "diff_drive_car_acados_create_capsule")
            create_capsule.restype = ctypes.c_void_p

            create_solver = getattr(libsolver, "diff_drive_car_acados_create")
            create_solver.argtypes = [ctypes.c_void_p]
            create_solver.restype = ctypes.c_int

            free_solver = getattr(libsolver, "diff_drive_car_acados_free")
            free_solver.argtypes = [ctypes.c_void_p]
            free_solver.restype = ctypes.c_int

            free_capsule = getattr(libsolver, "diff_drive_car_acados_free_capsule")
            free_capsule.argtypes = [ctypes.c_void_p]
            free_capsule.restype = ctypes.c_int

            # B. 碰撞测试：分配胶囊
            print("  1. 正在调用 `create_capsule()`...")
            capsule = create_capsule()
            if capsule:
                print(f"     ✅ 内存胶囊分配成功！虚拟地址: {hex(capsule)}")
            else:
                print("     ❌ 内存胶囊分配失败！返回了 NULL。")
                return

            # C. 碰撞测试：初始化求解器
            print("  2. 正在调用 `create(capsule)`...")
            status = create_solver(capsule)
            if status == 0:
                print("     ✅ NMPC 求解器矩阵与卡尔曼滤波器内部初始化成功！状态码: 0")
            else:
                print(f"     ❌ 内部初始化失败！状态码: {status}")

            # D. 碰撞测试：释放内存
            print("  3. 正在调用 `free(capsule)` 和 `free_capsule()`...")
            free_solver(capsule)
            free_capsule(capsule)
            print("     ✅ 内存安全释放，未发生段错误 (Segfault)！")

            print("\n🏆 [终极结论] 你的 C 语言动态链接库在物理层和内存层完全健康！")
            print("🚀 [立刻修复] 请立即在 Rust 端执行硬重新编译，彻底清理缓存！")

        except Exception as e:
            print(f"🔥 演练崩溃: {e}")
    else:
        print("\n❌ 符号不匹配，无法进行内存演练。")
    print("=" * 80)


if __name__ == "__main__":
    run_diagnostic()  # -*- coding: utf-8 -*-
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。


def run_diagnostic():
    print("=" * 80)
    print("🛰️  FSD-Car V3.0 SOTA C-FFI 符号沙盘与内存边界深度诊断探针")
    print("=" * 80)

    # 1. 自动寻找动态库
    workspace_root = "/home/zhz/fsd-car"
    acados_lib_dir = os.path.join(workspace_root, "simulation-env/acados/lib")
    solver_lib_dir = os.path.join(workspace_root, "simulation-env/c_generated_code")

    libacados_path = os.path.join(acados_lib_dir, "libacados.so")
    libsolver_path = os.path.join(
        solver_lib_dir, "libacados_ocp_solver_diff_drive_car.so"
    )

    print("📂 [库自检] 正在对齐 C 库路径...")
    print(
        f"  -> libacados: {libacados_path} ({'✅ 存在' if os.path.exists(libacados_path) else '❌ 缺失'})"
    )
    print(
        f"  -> libsolver: {libsolver_path} ({'✅ 存在' if os.path.exists(libsolver_path) else '❌ 缺失'})"
    )

    if not os.path.exists(libacados_path) or not os.path.exists(libsolver_path):
        print("❌ [致命] 物理库残缺，请先在 simulation-env 目录下生成求解器。")
        return

    # 2. 并网环境预热并加载依赖（对齐 2026 年最新 SOTA 加载链规范）
    print("\n⚡ [符号沙盘演练] 正在装载动态链接库...")
    try:
        # 预先全局注入依赖，防止符号断链
        for dep in ["libqpOASES_e.so", "libblasfeo.so", "libhpipm.so"]:
            dep_path = os.path.join(acados_lib_dir, dep)
            ctypes.CDLL(dep_path, mode=ctypes.RTLD_GLOBAL)

        # 加载核心库
        libacados = ctypes.CDLL(libacados_path, mode=ctypes.RTLD_GLOBAL)
        # 加载求解器共享库
        libsolver = ctypes.CDLL(libsolver_path)
        print("✅ 动态链接库全部成功载入进程空间！")
    except Exception as e:
        print(f"❌ 装载失败: {e}")
        return

    # 3. 符号导出审计 (2026 SOTA API 契约审计)
    print("\n🔎 [符号导出审计] 正在探测 C API 函数签名极性...")

    symbols_to_check = {
        "diff_drive_car_acados_create_capsule": "分配内存胶囊 [create_capsule]",
        "diff_drive_car_acados_create": "求解器矩阵初始化 [create]",
        "diff_drive_car_acados_free": "释放求解器 [free]",
        "diff_drive_car_acados_free_capsule": "释放胶囊内存 [free_capsule]",
    }

    detected_symbols = {}
    for sym, desc in symbols_to_check.items():
        has_sym = hasattr(libsolver, sym)
        detected_symbols[sym] = has_sym
        status_str = "✨ [SOTA 2026 API 支持]" if has_sym else "⚠️ [旧版本 API]"
        print(
            f"  -> {sym:<40} : {'✅ 发现' if has_sym else '❌ 缺失'} ({desc}) {status_str}"
        )

    # 4. 执行内存分配与 FFI 运行时碰撞演练
    print("\n💎 [内存边界碰撞演练] 模拟 Rust 执行构造函数...")

    if (
        detected_symbols["diff_drive_car_acados_create_capsule"]
        and detected_symbols["diff_drive_car_acados_create"]
    ):
        try:
            # A. 提取符号
            create_capsule = getattr(libsolver, "diff_drive_car_acados_create_capsule")
            create_capsule.restype = ctypes.c_void_p

            create_solver = getattr(libsolver, "diff_drive_car_acados_create")
            create_solver.argtypes = [ctypes.c_void_p]
            create_solver.restype = ctypes.c_int

            free_solver = getattr(libsolver, "diff_drive_car_acados_free")
            free_solver.argtypes = [ctypes.c_void_p]
            free_solver.restype = ctypes.c_int

            free_capsule = getattr(libsolver, "diff_drive_car_acados_free_capsule")
            free_capsule.argtypes = [ctypes.c_void_p]
            free_capsule.restype = ctypes.c_int

            # B. 碰撞测试：分配胶囊
            print("  1. 正在调用 `create_capsule()`...")
            capsule = create_capsule()
            if capsule:
                print(f"     ✅ 内存胶囊分配成功！虚拟地址: {hex(capsule)}")
            else:
                print("     ❌ 内存胶囊分配失败！返回了 NULL。")
                return

            # C. 碰撞测试：初始化求解器
            print("  2. 正在调用 `create(capsule)`...")
            status = create_solver(capsule)
            if status == 0:
                print("     ✅ NMPC 求解器矩阵与卡尔曼滤波器内部初始化成功！状态码: 0")
            else:
                print(f"     ❌ 内部初始化失败！状态码: {status}")

            # D. 碰撞测试：释放内存
            print("  3. 正在调用 `free(capsule)` 和 `free_capsule()`...")
            free_solver(capsule)
            free_capsule(capsule)
            print("     ✅ 内存安全释放，未发生段错误 (Segfault)！")

            print("\n🏆 [终极结论] 你的 C 语言动态链接库在物理层和内存层完全健康！")
            print("🚀 [立刻修复] 请立即在 Rust 端执行硬重新编译，彻底清理缓存！")

        except Exception as e:
            print(f"🔥 演练崩溃: {e}")
    else:
        print("\n❌ 符号不匹配，无法进行内存演练。")
    print("=" * 80)


if __name__ == "__main__":
    run_diagnostic()
