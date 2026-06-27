import os
import sys

print("==================================================")
print("      WSL2 代理设置深度排查探针 (2026)")
print("==================================================")

# 1. 检查当前活跃的代理环境变量
print("\n[1] 当前生效的代理环境变量:")
proxy_vars = {k: v for k, v in os.environ.items() if 'proxy' in k.lower()}
if proxy_vars:
    for k, v in proxy_vars.items():
        print(f"  {k} = {v}")
else:
    print("  没有检测到活跃的代理环境变量。")

# 2. 扫描常见的系统和用户启动配置文件
print("\n[2] 开始扫描常见启动配置文件...")
targets = [
    '~/.bashrc', '~/.zshrc', '~/.profile', '~/.bash_profile', 
    '~/.bash_login', '~/.zshenv', '~/.zprofile', '~/.zlogin',
    '/etc/profile', '/etc/environment', '/etc/bash.bashrc',
    '/etc/resolv.conf'
]

found_any = False
for t in targets:
    path = os.path.expanduser(t)
    if os.path.exists(path):
        try:
            with open(path, 'r', errors='ignore') as f:
                content = f.read()
                if "192.168.3.11" in content or "代理已开启" in content:
                    print(f"  🚩 发现可疑配置在: {t}")
                    lines = content.split('\n')
                    for i, line in enumerate(lines):
                        if "192.168.3.11" in line or "代理已开启" in line:
                            print(f"    第 {i+1} 行: {line.strip()}")
                    found_any = True
        except Exception as e:
            pass

# 3. 深度递归扫描家目录下的隐藏脚本（过滤掉大文件目录如 conda）
print("\n[3] 深度递归扫描家目录下的隐藏脚本与插件...")
home_dir = os.path.expanduser('~')
# 忽略掉不可能有代理脚本的巨大目录，防止卡死
ignore_dirs = ['miniconda3', '.cache', '.git', '.conda', 'node_modules', 'workspace', 'FSD-car', 'nlp_TC_Encoder', 'lejepa-mvp']

for root, dirs, files in os.walk(home_dir):
    # 动态裁剪忽略目录
    dirs[:] = [d for d in dirs if d not in ignore_dirs]
    
    for file in files:
        # 只扫描脚本文件、隐藏文件、以及无后缀文件
        if file.endswith('.sh') or file.startswith('.') or '.' not in file:
            file_path = os.path.join(root, file)
            try:
                with open(file_path, 'r', errors='ignore') as f:
                    content = f.read()
                    if "192.168.3.11" in content or "代理已开启" in content:
                        rel_path = os.path.relpath(file_path, home_dir)
                        print(f"  🚩 发现可疑脚本在: ~/{rel_path}")
                        lines = content.split('\n')
                        for i, line in enumerate(lines):
                            if "192.168.3.11" in line or "代理已开启" in line:
                                print(f"    第 {i+1} 行: {line.strip()}")
                        found_any = True
            except:
                pass

if not found_any:
    print("  ❌ 未能在任何配置文件中找到包含 '192.168.3.11' 或 '代理已开启' 的代码。")
    print("  提示：请检查你是否在 Windows 端的控制台或全局系统变量里设置了代理穿透。")
print("\n==================================================")
