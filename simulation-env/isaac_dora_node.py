import os
import sys
import struct
import numpy as np
import cv2
import torch
import torch.nn.functional as F
import onnxruntime as ort
import pyarrow as pa
from dora import Node
os.environ["ENABLE_CAMERAS"] = "1"
os.environ["ISAAC_ASSET_ROOT"] = "/run/media/zhz/数据/isaac_assets"
def load_env_manually():
    env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".env")
    if os.path.exists(env_path):
        with open(env_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    parts = line.split("=", 1)
                    if len(parts) == 2:
                        os.environ[parts[0].strip()] = parts[1].strip()
load_env_manually()
try:
    sys.argv.extend(["--/log/level=error", "--/log/fileLogLevel=error"])
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({
        "headless": False,  
    })
    import omni
    from pxr import Usd, UsdPhysics, PhysxSchema, UsdGeom
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import open_stage
    import omni.replicator.core as rep
    print("NVIDIA Isaac Sim 2026 Core Engine successfully initialized.")
except ImportError as e:
    print(f"ImportError: {e}")
    sys.exit(1)
class CLIDDEngine:
    def __init__(self, model_path):
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        print("CLIDD Local Neural Localization Engine successfully loaded.")
    def extract(self, frame_rgb, top_k=200):
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        pad_y = (640 - 480) // 2
        padded = cv2.copyMakeBorder(gray, pad_y, 640 - 480 - pad_y, 0, 0, cv2.BORDER_CONSTANT, value=0)
        tensor = torch.from_numpy(padded).float().unsqueeze(0).unsqueeze(0)
        mean, std = tensor.mean(), tensor.std()
        tensor = (tensor - mean) / (std + 1e-6)
        outs = self.session.run(None, {self.input_name: tensor.numpy()})
        desc, scores, rel = [torch.from_numpy(x) for x in outs]
        if desc.shape[1] == 80 and desc.shape[2] == 80:
            desc = desc.permute(0, 3, 1, 2)
        if scores.shape[1] == 80 and scores.shape[2] == 80:
            scores = scores.permute(0, 3, 1, 2)
        if len(rel.shape) == 4 and rel.shape[3] == 1 and rel.shape[1] == 80 and rel.shape[2] == 80:
            rel = rel.permute(0, 3, 1, 2)
        scores = F.softmax(scores, dim=1)[:, :-1, :, :]
        scores = F.pixel_shuffle(scores, 8)
        rel = F.interpolate(rel, size=(640, 640), mode='bilinear', align_corners=False)
        conf = scores * rel
        conf_nms = F.max_pool2d(conf, kernel_size=5, stride=1, padding=2)
        mask = (conf == conf_nms) & (conf > 0.05)
        conf_flat = conf[mask]
        if len(conf_flat) == 0:
            return None
        topk = min(top_k, len(conf_flat))
        scores_k, indices = torch.topk(conf_flat, topk)
        y, x = torch.where(mask[0, 0])
        y, x = y[indices], x[indices]
        grid_x = (x.float() / 639.0) * 2.0 - 1.0
        grid_y = (y.float() / 639.0) * 2.0 - 1.0
        grid = torch.stack([grid_x, grid_y], dim=1).unsqueeze(0).unsqueeze(2)
        desc_sampled = F.grid_sample(desc, grid, mode='bilinear', align_corners=False)
        desc_sampled = desc_sampled.squeeze(0).squeeze(2).t()
        desc_sampled = F.normalize(desc_sampled, p=2, dim=1)
        y = y.float() - pad_y
        x = x.float()
        valid = (y >= 0) & (y < 480)
        x, y, scores_k, desc_sampled = x[valid], y[valid], scores_k[valid], desc_sampled[valid]
        x_np, y_np, s_np, d_np = x.numpy(), y.numpy(), scores_k.numpy(), desc_sampled.numpy()
        x_arr = pa.array(x_np, type=pa.float32())
        y_arr = pa.array(y_np, type=pa.float32())
        s_arr = pa.array(s_np, type=pa.float32())
        d_flat = pa.array(d_np.flatten(), type=pa.float32())
        d_arr = pa.FixedSizeListArray.from_arrays(d_flat, 64)
        struct_arr = pa.StructArray.from_arrays(
            [x_arr, y_arr, s_arr, d_arr],
            names=['x', 'y', 'score', 'descriptor']
        )
        return struct_arr
def main():
    if "DORA_NODE_CONFIG" not in os.environ:
        print("Standalone simulation testing is bypassed.")
        simulation_app.close()
        sys.exit(0)
    print("Launching FSD local zero-copy integration pipeline with ARS...")
    dora_node = Node()
    fsd_assets_dir = "/home/zhz/fsd-car/assets"
    clean_usd_name = "fsd_car_clean1.usd"
    default_usd_name = "fsd_car_racetrack.usd"
    clean_usd_path = os.path.join(fsd_assets_dir, clean_usd_name)
    default_usd_path = os.path.join(fsd_assets_dir, default_usd_name)
    usd_path = clean_usd_path if os.path.exists(clean_usd_path) else default_usd_path
    if not os.path.exists(usd_path):
        sys.exit(1)
    open_stage(usd_path=usd_path)
    world = World(
        physics_dt=0.01, 
        rendering_dt=0.01, 
        backend="numpy"
    )
    from isaacsim.core.rendering_manager import RenderingManager
    RenderingManager.set_dt(0.01)
    car_path = "/Root/jetbot"
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(car_path):
        sys.exit(1)
    for prim in stage.Traverse():
        if prim.GetPath().HasPrefix(car_path) and prim.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(1e5)
            physx_joint = PhysxSchema.PhysxJointAPI.Apply(prim)
            physx_joint.CreateMaxJointVelocityAttr().Set(100000.0) 
    car = Articulation(prim_paths_expr=car_path, name="jetbot")
    world.scene.add(car)
    world.reset()
    init_positions, init_orientations = car.get_world_poses()
    print(f"✓ SOTA Spatial Anchoring: Saved init start pose: {init_positions[0]}")
    obstacle_paths = []
    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        if "jetbot" in prim_path.lower() or "groundplane" in prim_path.lower() or "camera" in prim_path.lower() or "light" in prim_path.lower() or prim_path == "/Root":
            continue
        name = prim.GetName().lower()
        if any(kw in name for kw in ["obstacle", "pallet", "box", "can", "bottle", "sphere", "cone", "table", "chair", "cube", "cylinder"]):
            if prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Mesh):
                obstacle_paths.append(prim_path)
    from isaacsim.core.prims import XFormPrim
    obstacle_path = "/Root/dynamic_obstacles*/box_obstacle_*"
    obstacle = None
    obs_init_poses = None
    obs_init_rots = None
    if stage.GetPrimAtPath("/Root"):
        obstacle = XFormPrim(prim_paths_expr=obstacle_path, name="box_obstacle")
        world.scene.add(obstacle)
    camera_path = f"{car_path}/chassis/rgb_camera/jetbot_camera"
    render_product = rep.create.render_product(camera_path, (640, 480))
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([render_product])
    clidd_model_path = "/home/zhz/fsd-car/model/xfeat_640x640.onnx"
    if not os.path.exists(clidd_model_path):
        sys.exit(1)
    clidd_engine = CLIDDEngine(clidd_model_path)
    if obstacle is not None:
        obs_init_poses, obs_init_rots = obstacle.get_world_poses()
    left_idx = 0
    right_idx = 1
    for idx, name in enumerate(car.dof_names):
        if "left" in name.lower():
            left_idx = idx
        elif "right" in name.lower():
            right_idx = idx
    world.play()
    left_sign = 1.0
    right_sign = 1.0
    for step_idx in range(60):
        if step_idx < 30:
            test_targets = np.zeros(len(car.dof_names))
            test_targets[left_idx] = 2.0
            test_targets[right_idx] = 2.0
            car.set_joint_velocity_targets(test_targets)
        elif step_idx == 30:
            joint_vels = car.get_joint_velocities()
            if joint_vels is not None and len(joint_vels) > 0:
                raw_left = float(joint_vels[0][left_idx])
                raw_right = float(joint_vels[0][right_idx])
                left_sign = 1.0 if raw_left >= 0.0 else -1.0
                right_sign = 1.0 if raw_right >= 0.0 else -1.0
            stop_targets = np.zeros(len(car.dof_names))
            car.set_joint_velocity_targets(stop_targets)
        world.step(render=True)
    L = 0.1125
    R = 0.03362 
    tick = 0
    v_cmd = 0.0
    w_cmd = 0.0
    rgb_frame = None
    alpha_ars = 30.0 
    v_left_ref = 0.0
    v_right_ref = 0.0
    current_goal_pose = [0.52, 4.11]
    try:
        while simulation_app.is_running():
            world.step(render=True)
            tick += 1
            if obstacle is not None and obs_init_poses is not None:
                t = tick * 0.01
                new_poses = obs_init_poses.copy()
                num_obstacles = len(obs_init_poses)
                phase_offsets = np.arange(num_obstacles) * 0.6
                new_poses[:, 0] = obs_init_poses[:, 0] + 1.2 * np.sin(1.5 * t + phase_offsets)
                obstacle.set_world_poses(positions=new_poses, orientations=obs_init_rots)
            rgb_raw = rgb_annotator.get_data()
            if rgb_raw is not None:
                rgb_frame = rgb_raw
                if len(rgb_frame.shape) == 3 and rgb_frame.shape[2] == 4:
                    rgb_frame = rgb_frame[:, :, :3]
                if rgb_frame.dtype == np.float32 or rgb_frame.dtype == np.float64:
                    rgb_frame = (rgb_frame * 255.0).astype(np.uint8)
            if rgb_frame is not None and rgb_frame.size > 0:
                positions, orientations = car.get_world_poses()
                pose_x, pose_y = float(positions[0][0]), float(positions[0][1])
                qw, qx, qy, qz = orientations[0]
                pose_yaw = float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
                dx_goal = current_goal_pose[0] - pose_x
                dy_goal = current_goal_pose[1] - pose_y
                dist_to_goal = np.sqrt(dx_goal**2 + dy_goal**2)
                if dist_to_goal < 0.25:
                    print(f"\n🚀 [仿真自愈] 车端自动重定位重置点...")
                    car.set_world_poses(positions=init_positions, orientations=init_orientations)
                    car.set_joint_velocity_targets(np.zeros(len(car.dof_names)))
                    pose_x, pose_y, pose_yaw = float(init_positions[0][0]), float(init_positions[0][1]), 0.0
                arrow_odom = pa.array([pose_x, pose_y, pose_yaw], type=pa.float32())
                dora_node.send_output("odometry", arrow_odom)
                bev_grid = np.zeros((192, 192), dtype=np.uint8)
                for yw_sample in np.linspace(pose_y, pose_y + 8.0, 50):
                    dx_left = -1.5 - pose_x
                    dy_left = yw_sample - pose_y
                    xl_l = dx_left * np.cos(pose_yaw) + dy_left * np.sin(pose_yaw)
                    yl_l = -dx_left * np.sin(pose_yaw) + dy_left * np.cos(pose_yaw)
                    col_l = int(96.0 - (yl_l / 0.03125))
                    row_l = int(191.0 - (xl_l / 0.03125))
                    if 0 <= row_l < 192 and 0 <= col_l < 192:
                        cv2.circle(bev_grid, (col_l, row_l), 4, 255, -1)
                    dx_right = 1.5 - pose_x
                    dy_right = yw_sample - pose_y
                    xl_r = dx_right * np.cos(pose_yaw) + dy_right * np.sin(pose_yaw)
                    yl_r = -dx_right * np.sin(pose_yaw) + dy_right * np.cos(pose_yaw)
                    col_r = int(96.0 - (yl_r / 0.03125))
                    row_r = int(191.0 - (xl_r / 0.03125))
                    if 0 <= row_r < 192 and 0 <= col_r < 192:
                        cv2.circle(bev_grid, (col_r, row_r), 4, 255, -1)
                for prim_path in obstacle_paths:
                    prim = stage.GetPrimAtPath(prim_path)
                    if not prim.IsValid():
                        continue
                    xform = UsdGeom.Xformable(prim)
                    time_code = Usd.TimeCode.Default()
                    matrix = xform.ComputeLocalToWorldTransform(time_code)
                    translation = matrix.ExtractTranslation()
                    ox, oy = float(translation[0]), float(translation[1])
                    dx_o = ox - pose_x
                    dy_o = oy - pose_y
                    xl_o = dx_o * np.cos(pose_yaw) + dy_o * np.sin(pose_yaw)
                    yl_o = -dx_o * np.sin(pose_yaw) + dy_o * np.cos(pose_yaw)
                    col_o = int(96.0 - (yl_o / 0.03125))
                    row_o = int(191.0 - (xl_o / 0.03125))
                    if -50 <= row_o < 242 and -50 <= col_o < 242:
                        cv2.circle(bev_grid, (col_o, row_o), 8, 255, -1)
                bev_flat = bev_grid.flatten()
                arrow_bev = pa.array(bev_flat, type=pa.uint8())
                dora_node.send_output("bev_grid", arrow_bev)
                bgr_frame = rgb_frame.copy() 
                clean_bgr = bgr_frame.copy()
                _, jpeg_encoded = cv2.imencode('.jpg', clean_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                arrow_jpeg = pa.array(jpeg_encoded.flatten(), type=pa.uint8())
                dora_node.send_output("jpeg_image", arrow_jpeg)
                if tick % 100 == 0:
                    arrow_clidd_features = clidd_engine.extract(rgb_frame, top_k=200)
                    if arrow_clidd_features is not None:
                        dora_node.send_output("xfeat_features", arrow_clidd_features)
            event = dora_node.next(timeout=0.001)
            if event is not None:
                ev_type = event["type"]
                if ev_type == "INPUT":
                    ev_id = event["id"]
                    if ev_id == "control_cmd":
                        cmd_arr = event["value"].to_numpy()
                        if len(cmd_arr) == 2:
                            v_cmd, w_cmd = float(cmd_arr[0]), float(cmd_arr[1])
                    elif ev_id == "human_prior":
                        prior_arr = event["value"].to_numpy()
                        if len(prior_arr) >= 2:
                            current_goal_pose = [float(prior_arr[0]), float(prior_arr[1])]
                elif ev_type == "STOP":
                    break
            v_left_cmd = (v_cmd - w_cmd * L / 2.0) / R
            v_right_cmd = (v_cmd + w_cmd * L / 2.0) / R
            joint_vels = car.get_joint_velocities()
            if joint_vels is not None and len(joint_vels) > 0:
                v_left_actual = float(joint_vels[0][left_idx]) * left_sign
                v_right_actual = float(joint_vels[0][right_idx]) * right_sign
            else:
                v_left_actual = 0.0
                v_right_actual = 0.0
            v_left_ref += 0.01 * alpha_ars * (v_left_cmd - v_left_ref)
            v_right_ref += 0.01 * alpha_ars * (v_right_cmd - v_right_ref)
            u_left = v_left_ref
            u_right = v_right_ref
            targets = np.zeros(len(car.dof_names))
            targets[left_idx] = u_left * left_sign
            targets[right_idx] = u_right * right_sign
            car.set_joint_velocity_targets(targets)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.set_joint_velocity_targets(np.array([[0.0, 0.0]]))
            world.step(render=True)
            world.stop()
        except Exception:
            pass
        simulation_app.close()
if __name__ == "__main__":
    main()
