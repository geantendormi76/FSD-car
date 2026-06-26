import cv2
import numpy as np

# 绑定您的物理主权视频流
VIDEO_URL = "http://192.168.5.19:8080/video"
cap = cv2.VideoCapture(VIDEO_URL)

if not cap.isOpened():
    print("❌ 无法连接到手机视频流，请检查 WSL2 网络！")
    exit(1)

print("🛡️ 物理主权视频流并网成功！按 'q' 键退出。")

prev_gray = None

while True:
    ret, frame = cap.read()
    if not ret: break
    
    # 1. 视网膜初级过滤：转灰度 + 高斯模糊去噪
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    
    if prev_gray is None:
        prev_gray = gray
        continue
        
    # 2. 青蛙眼核心算子：帧差法（抑制静态背景 IRF，兴奋动态物体 ERF）
    frame_diff = cv2.absdiff(prev_gray, gray)
    
    # 3. 提取高能级运动斑块
    _, thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)
    thresh = cv2.dilate(thresh, None, iterations=2)
    
    # 4. 生成 2D 人工势场（强模糊模拟势场梯度排斥力）
    potential_field = cv2.GaussianBlur(thresh, (101, 101), 0)
    
    # 伪彩色热力图映射 (越红代表斥力越大)
    heatmap = cv2.applyColorMap(potential_field, cv2.COLORMAP_JET)
    result = cv2.addWeighted(frame, 0.6, heatmap, 0.4, 0)
    
    cv2.imshow("WSL2 - Pseudo Frog-Eye Potential Field", result)
    
    # 关键点：更新抑制感受野背景
    prev_gray = gray 
    
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
