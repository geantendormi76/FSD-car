# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "dora-rs==0.3.13",
#     "numpy>=1.26.0",
#     "opencv-python>=4.8.0",
#     "pyarrow>=14.0.0"
# ]
# ///
import cv2
import numpy as np
import pyarrow as pa
from dora import Node

def main():
    print("========================================================")
    print("💎 [NEXUS AR HUD] SOTA 级增强现实遥测大屏并网成功...")
    print("设计哲学: 仿真视讯零延迟解码 | AR特征增强叠加 | 多模态并构")
    print("========================================================")
    
    dora_node = Node()

    # 状态金库
    robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0
    prior_x, prior_y, prior_yaw = 0.0, 0.0, 0.0
    force_x, force_y = 0.0, 0.0
    v_cmd, w_cmd = 0.0, 0.0
    features = []
    current_frame = None

    # 画布与地图参数
    map_size = 600
    map_scale = 35.0  # 适应更宽广的建图轨迹
    map_center = (map_size // 2, map_size // 2)
    trajectory_history = []

    window_name = "NEXUS AR HUD Telemetry (SOTA 2026)"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    while True:
        event = dora_node.next(timeout=0.01) # 100Hz 级高速轮询
        if event is not None:
            ev_type = event["type"]
            if ev_type == "INPUT":
                ev_id = event["id"]
                if ev_id == "jpeg_image":
                    # 🎯 零拷贝解码 JPEG 图像流
                    jpeg_bytes = event["value"].to_numpy()
                    current_frame = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)
                elif ev_id == "odometry":
                    data = event["value"].to_numpy()
                    if len(data) >= 3:
                        robot_x, robot_y, robot_yaw = data[0], data[1], data[2]
                        
                        # 🎯 🌟 SOTA 级大屏自愈：如果检测到瞬移（坐标大幅度跳变），自动清空历史画线
                        if len(trajectory_history) > 0:
                            last_x, last_y = trajectory_history[-1]
                            dist_jump = np.sqrt((robot_x - last_x)**2 + (robot_y - last_y)**2)
                            if dist_jump > 2.0:  # 瞬移跨度大于 2 米，判定为重置
                                trajectory_history.clear()
                                print("🔄 [NEXUS 大屏] 检测到物理重置，历史轨迹画布已清空！")
                                
                        # 记录历史行驶轨迹
                        trajectory_history.append((robot_x, robot_y))
                        if len(trajectory_history) > 400:
                            trajectory_history.pop(0)
                elif ev_id == "human_prior":
                    data = event["value"].to_numpy()
                    if len(data) >= 3:
                        prior_x, prior_y, prior_yaw = data[0], data[1], data[2]
                elif ev_id == "obstacle_force":
                    data = event["value"].to_numpy()
                    if len(data) >= 2:
                        force_x, force_y = data[0], data[1]
                elif ev_id == "control_cmd":
                    data = event["value"].to_numpy()
                    if len(data) >= 2:
                        v_cmd, w_cmd = data[0], data[1]
                elif ev_id == "xfeat_features":
                    struct_arr = event["value"]
                    x_arr = struct_arr.field("x").to_numpy()
                    y_arr = struct_arr.field("y").to_numpy()
                    features = list(zip(x_arr, y_arr))
            elif ev_type == "STOP":
                break

        # ==========================================
        # 🖥️ 渲染引擎：AR 双屏拼接 (1240 x 620)
        # ==========================================
        dashboard = np.zeros((620, 1240, 3), dtype=np.uint8)

        # 1. 如果没有接收到视讯帧，展示温启动加载界面
        if current_frame is None:
            hud_view = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(hud_view, "WAITING FOR SIMULATION STREAM...", (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
        else:
            # 拷贝当前帧，准备绘制 AR HUD
            hud_view = current_frame.copy()

            # 🎯 🌟 AR HUD 核心实现：将特征点直接渲染在相机画面上
            for (fx, fy) in features:
                # 在画面对应的像素位置，绘制高亮半透明准心
                cv2.circle(hud_view, (int(fx), int(fy)), 2, (0, 255, 0), -1)
                
            # 🎯 🌟 AR HUD 核心实现：将排斥力向量绘制在视野中央下方 (直观反映推力)
            f_center_x = 320
            f_center_y = 400
            cv2.circle(hud_view, (f_center_x, f_center_y), 6, (255, 255, 255), -1)
            f_scale = 120.0
            f_end_x = int(f_center_x + force_x * f_scale)
            f_end_y = int(f_center_y - force_y * f_scale)
            # 绘制醒目的排斥力红色箭矢
            cv2.arrowedLine(hud_view, (f_center_x, f_center_y), (f_end_x, f_end_y), (0, 0, 255), 3, tipLength=0.25)
            cv2.putText(hud_view, f"Bionic Repulse: ({force_x:.2f}, {force_y:.2f})", (20, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        # 调整 HUD 尺寸适应大屏
        hud_resized = cv2.resize(hud_view, (620, 620))
        dashboard[0:620, 620:1240] = hud_resized

        # --- 左半屏：全局拓扑与轨迹跟踪 (620x620) ---
        map_view = np.zeros((620, 620, 3), dtype=np.uint8)
        # 绘制科幻暗格背景
        for i in range(0, 620, 40):
            cv2.line(map_view, (i, 0), (i, 620), (25, 25, 25), 1)
            cv2.line(map_view, (0, i), (620, i), (25, 25, 25), 1)

        def to_map_coords(px, py):
            mx = int(310 + px * map_scale)
            my = int(310 - py * map_scale)
            return (mx, my)

        # 绘制历史行驶轨迹线 (弹性橡皮筋面包屑)
        if len(trajectory_history) > 1:
            for i in range(len(trajectory_history) - 1):
                pt1 = to_map_coords(trajectory_history[i][0], trajectory_history[i][1])
                pt2 = to_map_coords(trajectory_history[i+1][0], trajectory_history[i+1][1])
                cv2.line(map_view, pt1, pt2, (120, 255, 120), 1, cv2.LINE_AA)

        # 绘制目标引力点 (Human Prior)
        prior_pt = to_map_coords(prior_x, prior_y)
        cv2.drawMarker(map_view, prior_pt, (0, 255, 255), cv2.MARKER_STAR, 22, 2)
        cv2.putText(map_view, "Goal", (prior_pt[0]+12, prior_pt[1]-12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 绘制小车
        robot_pt = to_map_coords(robot_x, robot_y)
        cv2.circle(map_view, robot_pt, 9, (0, 255, 0), -1)
        r_end_x = int(robot_pt[0] + 28 * np.cos(robot_yaw))
        r_end_y = int(robot_pt[1] - 28 * np.sin(robot_yaw))
        cv2.arrowedLine(map_view, robot_pt, (r_end_x, r_end_y), (0, 100, 255), 2, tipLength=0.3)

        # 叠加 UI 状态栏
        cv2.putText(map_view, "FSD Global Trajectory", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(map_view, f"CMD -> v: {v_cmd:.2f} m/s | w: {w_cmd:.2f} rad/s", (20, 580), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 150, 50), 2)

        dashboard[0:620, 0:620] = map_view

        # 刷新屏幕
        cv2.imshow(window_name, dashboard)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()