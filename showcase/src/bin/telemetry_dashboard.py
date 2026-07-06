# /// script
# requires-python = ">=3.12"
# dependencies = [
#     # 🛡️ 协议对准：强制锁死 0.3.13 版本，与宿主 dora-cli 0.3.13 通信消息格式 v0.6.0 完美重合！
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
    print("💎 [NEXUS 探针] DORA 旁路可视化遥测大屏已启动...")
    print("设计哲学: 零拷贝旁路监听 | 双屏异构渲染 | 绝对不阻塞主控环路")
    print("========================================================")
    
    # 接入 DORA 拓扑网关
    dora_node = Node()

    # 状态金库缓存
    robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0
    prior_x, prior_y, prior_yaw = 0.0, 0.0, 0.0
    force_x, force_y = 0.0, 0.0
    v_cmd, w_cmd = 0.0, 0.0
    features = []

    # 画布物理参数
    map_size = 600
    map_scale = 40.0  # 1 meter = 40 pixels (适应 10m 级别的室内/室外场景)
    map_center = (map_size // 2, map_size // 2)

    window_name = "NEXUS Telemetry Dashboard (Ubuntu 26.04 LTS)"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    while True:
        # 🛡️ 架构师自愈：使用 0.02s (50Hz) 的非阻塞超时，确保渲染帧率平滑
        event = dora_node.next(timeout=0.02)
        if event is not None:
            ev_type = event["type"]
            if ev_type == "INPUT":
                ev_id = event["id"]
                if ev_id == "odometry":
                    data = event["value"].to_numpy()
                    if len(data) >= 3:
                        robot_x, robot_y, robot_yaw = data[0], data[1], data[2]
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
                    # 🎯 零拷贝解析 Arrow StructArray
                    struct_arr = event["value"]
                    x_arr = struct_arr.field("x").to_numpy()
                    y_arr = struct_arr.field("y").to_numpy()
                    features = list(zip(x_arr, y_arr))
            elif ev_type == "STOP":
                print("🛑 [NEXUS 探针] 收到 DORA 停止信号，安全卸载大屏。")
                break

        # ==========================================
        # 🖥️ 渲染引擎：双屏拼接 (1200 x 600)
        # ==========================================
        dashboard = np.zeros((600, 1200, 3), dtype=np.uint8)

        # --- 左半屏：全局拓扑与轨迹 (600x600) ---
        # 绘制物理网格 (每 1 米一格)
        grid_step = int(map_scale)
        for i in range(0, 600, grid_step):
            cv2.line(dashboard, (i, 0), (i, 600), (30, 30, 30), 1)
            cv2.line(dashboard, (0, i), (600, i), (30, 30, 30), 1)

        # 坐标系转换闭包 (物理坐标 -> 像素系，Y轴向上为正)
        def to_map_coords(px, py):
            mx = int(map_center[0] + px * map_scale)
            my = int(map_center[1] - py * map_scale) 
            return (mx, my)

        # 1. 绘制人类先验引力点 (Target)
        prior_pt = to_map_coords(prior_x, prior_y)
        cv2.drawMarker(dashboard, prior_pt, (0, 255, 255), cv2.MARKER_STAR, 20, 2)
        cv2.putText(dashboard, "Target (Human Prior)", (prior_pt[0]+10, prior_pt[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 2. 绘制小车当前位姿 (Odometry)
        robot_pt = to_map_coords(robot_x, robot_y)
        cv2.circle(dashboard, robot_pt, 8, (0, 255, 0), -1)
        # 绘制车头朝向向量
        end_x = int(robot_pt[0] + 25 * np.cos(robot_yaw))
        end_y = int(robot_pt[1] - 25 * np.sin(robot_yaw))
        cv2.arrowedLine(dashboard, robot_pt, (end_x, end_y), (0, 0, 255), 2, tipLength=0.3)
        cv2.putText(dashboard, "FSD-car", (robot_pt[0]+10, robot_pt[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # --- 右半屏：感知与势场 (600x600 区域内嵌 560x420 的相机视野) ---
        offset_x = 600
        cv2.rectangle(dashboard, (offset_x, 0), (1200, 600), (15, 15, 15), -1)
        cv2.putText(dashboard, "Perception & Force Field", (offset_x + 20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 绘制相机视野边界 (假设原图 640x480，按比例缩放至 560x420)
        cam_offset_x = offset_x + 20
        cam_offset_y = 80
        scale_p = 560 / 640.0
        cv2.rectangle(dashboard, (cam_offset_x, cam_offset_y), (cam_offset_x + 560, cam_offset_y + 420), (50, 50, 50), 1)

        # 3. 绘制 CLIDD 稀疏特征点
        for (fx, fy) in features:
            px = int(cam_offset_x + fx * scale_p)
            py = int(cam_offset_y + fy * scale_p)
            cv2.circle(dashboard, (px, py), 2, (0, 255, 0), -1)
        cv2.putText(dashboard, f"CLIDD Features: {len(features)}", (cam_offset_x, cam_offset_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 4. 绘制青蛙眼避障势场力 (在视野中心绘制)
        center_px = cam_offset_x + 280
        center_py = cam_offset_y + 210
        cv2.circle(dashboard, (center_px, center_py), 5, (255, 255, 255), -1)
        force_scale = 100.0  # 放大力向量以便肉眼观测
        f_end_x = int(center_px + force_x * force_scale)
        f_end_y = int(center_py - force_y * force_scale) 
        cv2.arrowedLine(dashboard, (center_px, center_py), (f_end_x, f_end_y), (0, 0, 255), 3, tipLength=0.2)
        cv2.putText(dashboard, f"Obstacle Force: ({force_x:.2f}, {force_y:.2f})", (cam_offset_x, cam_offset_y + 445), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 5. 绘制 NMPC 规控指令遥测
        cv2.putText(dashboard, f"NMPC CMD -> v: {v_cmd:.2f} m/s, w: {w_cmd:.2f} rad/s", (offset_x + 20, 560), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 50), 2)

        # 刷新屏幕
        cv2.imshow(window_name, dashboard)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()