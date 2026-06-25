import socket
import urllib.request
import urllib.error
import subprocess

print("==============================================")
print("     WSL2 物理网络 & DNS 域名解析 诊断工具")
print("==============================================")

# 1. 检查底层网络（直接用IP连接，绕过DNS）
print("【第一步】测试物理网络连接 (Ping 微软公共DNS)...", end="")
try:
    # 8.8.8.8 是谷歌的公共DNS，114.114.114.114 是国内公共DNS
    result = subprocess.run(["ping", "-c", "2", "-W", "2", "114.114.114.114"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode == 0:
        print(" 成功 (物理网络是通的)")
    else:
        print(" 失败 (物理网络不通，可能是Windows防火墙或网卡问题)")
except Exception as e:
    print(f" 失败 ({e})")

# 2. 检查 DNS 域名解析
print("【第二步】测试域名解析 (百度)...", end="")
try:
    socket.gethostbyname("www.baidu.com")
    print(" 成功 (DNS工作正常)")
except Exception as e:
    print(f" 失败 ({e}) -> 说明WSL2不认识网址，通常需要修改 /etc/resolv.conf")

# 3. 检查微软更新服务器（code . 报错的关键原因）
print("【第三步】测试连接 VS Code 服务器...", end="")
try:
    urllib.request.urlopen("https://update.code.visualstudio.com", timeout=4)
    print(" 成功 (可以正常连接微软，应能打开 code)")
except Exception as e:
    print(f" 失败 ({e}) -> 无法下载VS Code Server")

# 4. 检查 GitHub 连接（git clone 报错的原因）
print("【第四步】测试连接 GitHub...", end="")
try:
    urllib.request.urlopen("https://github.com", timeout=4)
    print(" 成功 (可以正常拉取 acados)")
except Exception as e:
    print(f" 失败 ({e}) -> 连接GitHub超时，国内网络环境常见，需要配置代理")

print("==============================================")
