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
import time
from dora import Node

def main():
    print("========================================================")
    print("👁️  NEXUS - Optical Flow Reflex Node Activated")
    print("Features: Epipolar Rotational Blanking | Semantic Masking")
    print("========================================================")
    
    dora_node = Node()
    
    prev_gray = None
    prev_pts = None
    
    # Vehicle motion tracker states
    prev_yaw = None
    prev_time = None
    omega_z = 0.0 # Instantaneous yaw rate
    
    # Initialize IPM homography matrix (aligned with Rust's IpmProjector)
    src_pts = np.float32([
        [0, 479],
        [639, 479],
        [224, 264],
        [416, 264]
    ])
    dst_pts = np.float32([
        [28.8, 191],
        [163.2, 191],
        [28.8, 0],
        [163.2, 0]
    ])
    H = cv2.getPerspectiveTransform(src_pts, dst_pts)
    
    # 192x192 occupancy grid initialized as all-zero (free space)
    bev_grid = np.zeros((192, 192), dtype=np.uint8)
    
    # Camera intrinsic properties (VGA resolution)
    fx = 500.0
    cx = 320.0
    cy = 240.0
    
    lk_params = dict(winSize=(15, 15), maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    feature_params = dict(maxCorners=120, qualityLevel=0.05, minDistance=10, blockSize=7)
    
    try:
        while True:
            event = dora_node.next(timeout=0.01)
            if event is None:
                continue
                
            ev_type = event["type"]
            if ev_type == "INPUT":
                ev_id = event["id"]
                
                # 1. Update vehicle yaw rate from odometry
                if ev_id == "odometry":
                    odom_data = event["value"].to_numpy()
                    if len(odom_data) >= 3:
                        curr_yaw = float(odom_data[2])
                        curr_time = time.time()
                        
                        if prev_yaw is not None and prev_time is not None:
                            dt = curr_time - prev_time
                            if dt > 0.001:
                                dyaw = curr_yaw - prev_yaw
                                # normalize to [-pi, pi]
                                dyaw = (dyaw + np.pi) % (2.0 * np.pi) - np.pi
                                omega_z = dyaw / dt
                                
                        prev_yaw = curr_yaw
                        prev_time = curr_time
                        
                # 2. Update semantic occupancy grid from perception_node
                elif ev_id == "bev_grid":
                    grid_flat = event["value"].to_numpy()
                    if len(grid_flat) == 192 * 192:
                        bev_grid = grid_flat.reshape((192, 192))
                        
                # 3. Main processing loop triggered by new visual frames
                elif ev_id == "jpeg_image":
                    jpeg_bytes = event["value"].to_numpy()
                    frame = cv2.imdecode(jpeg_bytes, cv2.IMREAD_GRAYSCALE)
                    if frame is None:
                        continue
                        
                    h, w = frame.shape
                    ttc = 10.0 # default safe TTC
                    
                    if prev_gray is None or prev_pts is None or len(prev_pts) < 15:
                        prev_gray = frame
                        prev_pts = cv2.goodFeaturesToTrack(frame, mask=None, **feature_params)
                    else:
                        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, frame, prev_pts, None, **lk_params)
                        
                        good_new = next_pts[status == 1]
                        good_old = prev_pts[status == 1]
                        
                        if len(good_new) > 10:
                            divergence_sum = 0.0
                            valid_points = 0
                            
                            for i, (new, old) in enumerate(zip(good_new, good_old)):
                                u_old, v_old = old[0], old[1]
                                u_new, v_new = new[0], new[1]
                                
                                dx = u_new - u_old
                                dy = v_new - v_old
                                
                                # A. Epipolar Rotational Blanking (Yaw Rate Cancellation)
                                x_rel = u_old - cx
                                y_rel = v_old - cy
                                
                                u_rot = -fx * omega_z - (x_rel**2 / fx) * omega_z
                                v_rot = -(x_rel * y_rel / fx) * omega_z
                                
                                dx_comp = dx - u_rot
                                dy_comp = dy - v_rot
                                
                                # B. Semantic Masking (Verify if point is on road via inverse-homography projection)
                                pt_homo = np.array([u_old, v_old, 1.0]).reshape((3, 1))
                                projected = np.dot(H, pt_homo)
                                col = int(projected[0, 0] / projected[2, 0])
                                row = int(projected[1, 0] / projected[2, 0])
                                
                                is_obstacle = True
                                if 0 <= col < 192 and 0 <= row < 192:
                                    if bev_grid[row, col] == 0: # 0 means Road (Free Space)
                                        is_obstacle = False
                                        
                                # C. Cumulative Translation Divergence (Only evaluate true obstacles)
                                if is_obstacle:
                                    r_sq = x_rel**2 + y_rel**2
                                    if r_sq > 400: # filter central singularity zone
                                        div = (dx_comp * x_rel + dy_comp * y_rel) / r_sq
                                        divergence_sum += div
                                        valid_points += 1
                                        
                            if valid_points > 0:
                                mean_divergence = divergence_sum / valid_points
                                if mean_divergence > 0.01: # looming detection threshold
                                    ttc = 1.0 / mean_divergence
                                    
                        prev_gray = frame
                        prev_pts = cv2.goodFeaturesToTrack(frame, mask=None, **feature_params)
                        
                    # Smooth clamp safety zone
                    ttc = max(0.1, min(10.0, ttc))
                    ttc_arrow = pa.array([ttc], type=pa.float32())
                    dora_node.send_output("ttc", ttc_arrow)
                    
            elif ev_type == "STOP":
                print("\n🛑 Optical reflex node unmounted cleanly.")
                break
                
    except Exception as e:
        print(f"Optical Reflex Node Error: {e}")

if __name__ == "__main__":
    main()
