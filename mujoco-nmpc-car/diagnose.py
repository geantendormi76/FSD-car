# diagnose.py
import os
import sys

def run_diagnostics():
    print("=" * 60)
    print("🔍 FSD-Car & Acados 环境自动诊断程序")
    print("=" * 60)

    # 1. 检查环境变量
    acados_env = os.environ.get('ACADOS_SOURCE_DIR', '')
    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    print(f"[环境变量] ACADOS_SOURCE_DIR: {acados_env or '❌ 未设置'}")
    print(f"[环境变量] LD_LIBRARY_PATH: {ld_path or '❌ 未设置'}")

    # 2. 检查环境变量路径是否真实存在
    env_dir_exists = False
    if acados_env:
        env_dir_exists = os.path.isdir(acados_env)
        print(f"[路径检查] 环境变量指向的目录是否存在: {'✅ 存在' if env_dir_exists else '❌ 不存在 (路径失效)'}")
    else:
        print("[路径检查] ❌ 环境变量未设置，无法校验")

    # 3. 检查 Python 环境中 acados_template 的导入状态
    import_ok = False
    try:
        import acados_template
        import_ok = True
        print(f"[Python导入] acados_template 导入状态: ✅ 成功 (位置: {acados_template.__file__})")
    except ImportError as e:
        print(f"[Python导入] acados_template 导入状态: ❌ 失败 (原因: {e})")

    # 4. 全局搜索 acados 源码目录的真实物理路径
    print("\n[磁盘扫描] 正在寻找真实的 acados 源码目录位置...")
    possible_paths = []
    search_root = "/home/zhz/FSD-car"
    
    if os.path.exists(search_root):
        for root, dirs, files in os.walk(search_root):
            if 'acados' in dirs:
                candidate = os.path.join(root, 'acados')
                # 校验是否为真实的 acados 仓库目录（通过特有子文件检验）
                if os.path.exists(os.path.join(candidate, 'interfaces', 'acados_template')):
                    possible_paths.append(candidate)
                    # 防止深入扫描子目录中的 acados
                    dirs.remove('acados')
    
    if possible_paths:
        print("✅ 找到以下有效的 acados 目录：")
        for p in possible_paths:
            print(f"  📍 {p}")
    else:
        print("❌ 未在 /home/zhz/FSD-car 目录下找到有效的 acados 目录。请确认它是否被误删。")

    # 5. 提供诊断结论与一键修复建议
    print("\n" + "=" * 60)
    print("💡 诊断结论与一键修复建议:")
    print("=" * 60)

    if not possible_paths:
        print("由于未找到 acados 源码目录，请先确认 acados 文件夹是否被不小心删除了。")
        return

    # 选取找到的第一个路径作为基准
    correct_path = possible_paths[0]
    
    needs_env_update = (acados_env != correct_path)
    needs_reinstall = not import_ok or needs_env_update

    if needs_env_update:
        print(f"⚠️ 检测到环境变量失效或路径不匹配！")
        print(f"  当前环境变量值: {acados_env}")
        print(f"  实际物理文件值: {correct_path}")
        print("\n👉 修复步骤 1：请在终端运行以下命令更新当前环境变量（并同步修改 ~/.bashrc）：")
        print(f"export ACADOS_SOURCE_DIR=\"{correct_path}\"")
        print(f"export LD_LIBRARY_PATH=\"$LD_LIBRARY_PATH:{correct_path}/lib\"")

    if needs_reinstall:
        print("\n👉 修复步骤 2：请复制并运行以下命令重新关联 Python 库：")
        print(f"pip install -e {correct_path}/interfaces/acados_template")

    if not needs_env_update and import_ok:
        print("您的环境配置参数看起来正常。")
        print("如果代码依然报错，可能是您在运行代码时没有激活当前的 Python 虚拟环境，导致指向了错误的 Python 解释器。")
        print(f"当前诊断程序所运行的 Python 路径为: {sys.executable}")

if __name__ == "__main__":
    run_diagnostics()
