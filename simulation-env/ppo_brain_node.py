import os
import sys
import numpy as np
import onnxruntime as ort
import pyarrow as pa
from collections import deque
from dora import Node
def main():
    print("=========================================================================")
    print("🧠  NEXUS - 15D Cleaned PPO Neural Brain Deployment Node (SOTA 2026)")
    print("=========================================================================")
    model_path = "/home/zhz/fsd-car/model/spiced_brain.onnx"
    if not os.path.exists(model_path):
        print(f"[-] Error: Target PPO model spiced_brain.onnx not found at {model_path}")
        sys.exit(1)
    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    dora_node = Node()
    state_odom = [0.0, 0.0, 0.0]     
    goal_pose = [0.52, 4.11]         
    history_queue = deque(maxlen=5)
    print("[+] 15D PPO Neural Brain successfully mounted. Entering 100Hz inference loop...")
    print("-" * 73)
    try:
        while True:
            event = dora_node.next(timeout=0.01)
            if event is None:
                continue
            ev_type = event["type"]
            if ev_type == "INPUT":
                ev_id = event["id"]
                if ev_id == "human_prior":
                    prior_arr = event["value"].to_numpy()
                    if len(prior_arr) >= 2:
                        goal_pose = [float(prior_arr[0]), float(prior_arr[1])]
                elif ev_id == "odometry":
                    odom_arr = event["value"].to_numpy()
                    if len(odom_arr) >= 3:
                        state_odom = [float(odom_arr[0]), float(odom_arr[1]), float(odom_arr[2])]
                    dx = goal_pose[0] - state_odom[0]
                    dy = goal_pose[1] - state_odom[1]
                    yaw = state_odom[2]
                    local_goal_x = dx * np.cos(yaw) + dy * np.sin(yaw)
                    local_goal_y = -dx * np.sin(yaw) + dy * np.cos(yaw)
                    local_goal_dist = np.sqrt(local_goal_x**2 + local_goal_y**2)
                    current_frame = [
                        local_goal_x * 0.20,
                        local_goal_y * 0.20,
                        local_goal_dist * 0.20
                    ]
                    if len(history_queue) == 0:
                        for _ in range(5):
                            history_queue.append(current_frame)
                    else:
                        history_queue.append(current_frame)
                    state_15 = np.array(list(history_queue), dtype=np.float32).flatten().reshape(1, 15)
                    run_outs = session.run(None, {input_name: state_15})
                    action = run_outs[0][0] 
                    a_vel = float(action[0])
                    a_kappa = float(action[1])
                    v_max = 0.80
                    kappa_max = 1.25
                    w_max = 1.00
                    v_des = np.clip(a_vel, 0.0, 1.0) * v_max
                    kappa = np.clip(a_kappa, -1.0, 1.0) * kappa_max
                    if abs(v_des) < 0.05:
                        w_ref = np.clip(a_kappa, -1.0, 1.0) * w_max
                    else:
                        w_ref = kappa * v_des
                    control_arrow = pa.array([v_des, w_ref], type=pa.float32())
                    dora_node.send_output("control_cmd", control_arrow)
            elif ev_type == "STOP":
                print("\n🛑 [PPO Brain Node] Unmounted cleanly.")
                break
    except Exception as e:
        print(f"[-] PPO Brain Node runtime error: {e}")
if __name__ == "__main__":
    main()
