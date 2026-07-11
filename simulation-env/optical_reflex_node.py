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
    print("👁️  NEXUS - Optical Flow Reflex Node Activated")
    print("Architecture: Lucas-Kanade Flow | FOE Divergence | TTC Solver")
    print("========================================================")
    
    dora_node = Node()
    
    prev_gray = None
    prev_pts = None
    
    lk_params = dict(winSize=(15, 15), maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    feature_params = dict(maxCorners=100, qualityLevel=0.1, minDistance=10, blockSize=7)
    
    try:
        while True:
            event = dora_node.next(timeout=0.05)
            if event is None:
                continue
                
            if event["type"] == "INPUT":
                if event["id"] == "jpeg_image":
                    jpeg_bytes = event["value"].to_numpy()
                    frame = cv2.imdecode(jpeg_bytes, cv2.IMREAD_GRAYSCALE)
                    
                    if frame is None:
                        continue
                        
                    h, w = frame.shape
                    center_x, center_y = w / 2.0, h / 2.0
                    
                    ttc = 10.0 
                    
                    if prev_gray is None or prev_pts is None or len(prev_pts) < 10:
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
                                dx = new[0] - old[0]
                                dy = new[1] - old[1]
                                
                                rx = old[0] - center_x
                                ry = old[1] - center_y
                                r_sq = rx**2 + ry**2
                                
                                if r_sq > 1000: 
                                    div = (dx * rx + dy * ry) / r_sq
                                    divergence_sum += div
                                    valid_points += 1
                                    
                            if valid_points > 0:
                                mean_divergence = divergence_sum / valid_points
                                if mean_divergence > 0.02: 
                                    ttc = 1.0 / mean_divergence
                                    
                        prev_gray = frame
                        prev_pts = cv2.goodFeaturesToTrack(frame, mask=None, **feature_params)
                    
                    ttc = max(0.1, min(10.0, ttc))
                    
                    ttc_arrow = pa.array([ttc], type=pa.float32())
                    dora_node.send_output("ttc", ttc_arrow)
                    
            elif event["type"] == "STOP":
                print("\n🛑 Optical reflex node unmounted cleanly.")
                break
                
    except Exception as e:
        print(f"Optical Reflex Node Error: {e}")

if __name__ == "__main__":
    main()
