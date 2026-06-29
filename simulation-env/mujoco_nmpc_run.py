# mujoco_nmpc_run.py
import ctypes
import os

import numpy as np

# 🛡️ 进程空间内存预热自愈：保障 MuJoCo 仿真跑道上的 acados 模型加载符号链安全完整
acados_source = os.environ.get(
    "ACADOS_SOURCE_DIR", "/home/zhz/fsd-car/simulation-env/acados"
)
if acados_source and os.path.exists(acados_source):
    acados_lib = os.path.join(acados_source, "lib")
    for lib_name in ["libqpOASES_e.so", "libblasfeo.so", "libhpipm.so"]:
        lib_path = os.path.join(acados_lib, lib_name)
        if os.path.exists(lib_path):
            try:
                ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
            except Exception:
                pass

import time

import mujoco
import mujoco.viewer
from acados_template import AcadosOcpSolver

# 物理模型：保持高抓地力轮胎并注入虚拟相机与视觉特征地标 [cite: 1.1.3]
xml_model = """
<mujoco>
  <option timestep="0.01"/>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <!-- 物理地表 -->
    <geom type="plane" size="15 15 0.1" rgba=".9 .9 .9 1"/>
    
    <!-- 🛡️ 为 XFeat 注入高对比度物理地标（圆柱、方箱），用作视觉纠偏的站牌 -->
    <body name="landmark_red_pillar" pos="3.0 1.2 0.4">
      <geom type="cylinder" size="0.15 0.4" rgba="1.0 0.0 0.0 1.0"/>
    </body>
    <body name="landmark_blue_box" pos="4.5 -1.0 0.4">
      <geom type="box" size="0.25 0.25 0.4" rgba="0.0 0.0 1.0 1.0"/>
    </body>
    <body name="landmark_green_pillar" pos="1.5 -1.5 0.4">
      <geom type="cylinder" size="0.1 0.4" rgba="0.0 1.0 0.0 1.0"/>
    </body>

    <body name="car" pos="0 0 0.08">
      <freejoint/>
      <!-- 车身半宽度缩窄为 0.11 米，拉开物理间隙 -->
      <geom type="box" size="0.2 0.11 0.05" rgba="0 0.5 0.8 1"/>
      
      <!-- 🛡️ 注入虚拟单目摄像头：使用标准的 xyaxes 规定右和上朝向，让镜头笔直向前看 -->
      <camera name="front_camera" pos="0.2 0.0 0.06" xyaxes="0 -1 0 0 0 1" fovy="70"/>
      
      <geom type="sphere" size="0.03" pos="-0.15 0 -0.05" condim="1" friction="0 0 0"/>
      <!-- 左轮 -->
      <body name="wheel_left" pos="0.0 0.18 0">
        <joint name="hinge_left" type="hinge" axis="0 1 0" damping="0.05"/>
        <geom type="cylinder" size="0.08 0.02" zaxis="0 1 0" rgba="0 0 0 1" friction="1.8 0.01 0.001"/>
      </body>
      <!-- 右轮 -->
      <body name="wheel_right" pos="0.0 -0.18 0">
        <joint name="hinge_right" type="hinge" axis="0 1 0" damping="0.05"/>
        <geom type="cylinder" size="0.08 0.02" zaxis="0 1 0" rgba="0 0 0 1" friction="1.8 0.01 0.001"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <!-- 将电调速度增益 kv 设为 150，获得澎湃且绝对跟手的响应，拒绝软弱打滑 -->
    <velocity name="motor_left" joint="hinge_left" kv="150" forcerange="-15 15"/>
    <velocity name="motor_right" joint="hinge_right" kv="150" forcerange="-15 15"/>
  </actuator>
</mujoco>
"""


# ====================================================
#  武器库武装 1：高保真 Mahony 6轴姿态滤波器 (C++ 1:1 移植)
# ====================================================
class MahonyAHRS:
    def __init__(self, sample_freq=100.0, kp=1.5, ki=0.005):
        self.twoKp = 2.0 * kp
        self.twoKi = 2.0 * ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # q0, q1, q2, q3
        self.integralFB = np.zeros(3)  # 零偏积分反馈
        self.invSampleFreq = 1.0 / sample_freq

    def update_imu(self, gx, gy, gz, ax, ay, az):
        # 1. 加速度计归一化
        norm_a = np.sqrt(ax * ax + ay * ay + az * az)
        if norm_a < 1e-6:
            return
        ax /= norm_a
        ay /= norm_a
        az /= norm_a

        # 2. 提取当前四元数乘积变量以减少重复运算
        q0, q1, q2, q3 = self.q[0], self.q[1], self.q[2], self.q[3]
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3

        # 3. 估计重力方向 (由当前姿态推算)
        halfvx = q1 * q3 - q0 * q2
        halfvy = q0 * q1 + q2 * q3
        halfvz = q0q0 - 0.5 + q3q3

        # 4. 计算重力估计方向与实际测量方向的叉积误差
        halfex = ay * halfvz - az * halfvy
        halfey = az * halfvx - ax * halfvz
        halfez = ax * halfvy - ay * halfvx

        # 5. 计算并应用积分反馈
        if self.twoKi > 0.0:
            self.integralFB[0] += self.twoKi * halfex * self.invSampleFreq
            self.integralFB[1] += self.twoKi * halfey * self.invSampleFreq
            self.integralFB[2] += self.twoKi * halfez * self.invSampleFreq
            gx += self.integralFB[0]
            gy += self.integralFB[1]
            gz += self.integralFB[2]
        else:
            self.integralFB = np.zeros(3)

        # 6. 应用比例反馈
        gx += self.twoKp * halfex
        gy += self.twoKp * halfey
        gz += self.twoKp * halfez

        # 7. 积分四元数变率
        gx *= 0.5 * self.invSampleFreq
        gy *= 0.5 * self.invSampleFreq
        gz *= 0.5 * self.invSampleFreq
        qa, qb, qc = q0, q1, q2
        self.q[0] += -qb * gx - qc * gy - q3 * gz
        self.q[1] += qa * gx + qc * gz - q3 * gy
        self.q[2] += qa * gy - qb * gz + q3 * gx
        self.q[3] += qa * gz + qb * gy - qc * gx

        # 8. 归一化四元数
        norm_q = np.sqrt(np.sum(self.q**2))
        self.q /= norm_q

    def get_estimated_yaw(self):
        # 从解算出的姿态四元数中提取偏航角 (Yaw)
        q = self.q
        return np.arctan2(
            2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)
        )


# ====================================================
#  3. 武器库武装 2：极简 PT1 低通滤波器 (C++ 1:1 移植)
# ====================================================
class PT1Filter:
    def __init__(self, cutoff_freq=10.0, sample_freq=100.0):
        # 计算低通滤波系数 alpha
        dt = 1.0 / sample_freq
        rc = 1.0 / (2.0 * np.pi * cutoff_freq)
        self.alpha = dt / (rc + dt)
        self.state = 0.0

    def update(self, input_val):
        self.state = self.alpha * input_val + (1.0 - self.alpha) * self.state
        return self.state


# ====================================================
#  4. 轨迹配置与决策参数准备
# ====================================================
total_steps = 1500
t_points = np.linspace(0, 15, total_steps)
ref_x = 0.3 * t_points
ref_y = 0.3 * np.sin(0.4 * t_points)  # 采用优雅的小幅度正弦弯

model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)
acados_solver = AcadosOcpSolver(
    None, json_file="acados_ocp.json", generate=False, build=False
)

N = 20
nx = 4
nu = 2

# 初始化 Mahony 滤波器与控制指令滤波器
mahony = MahonyAHRS(sample_freq=100.0, kp=1.5, ki=0.005)
filter_v = PT1Filter(cutoff_freq=3.0, sample_freq=100.0)
filter_w = PT1Filter(cutoff_freq=3.0, sample_freq=100.0)

# IMU 传感器级低通防抖滤网（飞控级核心过滤技术）
filter_gx = PT1Filter(cutoff_freq=15.0, sample_freq=100.0)
filter_gy = PT1Filter(cutoff_freq=15.0, sample_freq=100.0)
filter_gz = PT1Filter(cutoff_freq=15.0, sample_freq=100.0)

filter_ax = PT1Filter(cutoff_freq=5.0, sample_freq=100.0)
filter_ay = PT1Filter(cutoff_freq=5.0, sample_freq=100.0)
filter_az = PT1Filter(cutoff_freq=5.0, sample_freq=100.0)

# 显式“温启动”
initial_state = np.array([0.0, 0.0, 0.0, 0.0])
for k in range(N + 1):
    acados_solver.set(k, "x", initial_state)
for k in range(N):
    acados_solver.set(k, "u", np.zeros(nu))

# 🛡️ 架构师升级：直接从刚才新建的 XML 盘面中读取中国楼盘小区与 Scout Mini 车体
model = mujoco.MjModel.from_xml_path("china_residential_estate.xml")
data = mujoco.MjData(model)

# 容错获取底盘电机索引（Scout Mini 是四驱结构）
try:
    joint_left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hinge_lf")
    joint_right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hinge_rf")
except:
    joint_left_id = model.joint("hinge_lf").id
    joint_right_id = model.joint("hinge_rf").id

qvel_left_idx = model.jnt_dofadr[joint_left_id]
qvel_right_idx = model.jnt_dofadr[joint_right_id]

# 动态获取电机的执行通道控制索引
try:
    actuator_left_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_left"
    )
    actuator_right_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_right"
    )
except:
    actuator_left_id = 0
    actuator_right_id = 1

log_file_path = "nmpc_telemetry.log"
log_file = open(os.path.join("./", log_file_path), "w")
log_file.write(
    "Time,Current_X,Current_Y,Ref_X,Ref_Y,L2_Error_m,Cmd_v,Cmd_w,Est_Yaw,True_Yaw,SolveTime_ms\n"
)

print("\n" + "=" * 60)
print("🚀 FSD-car 数字化智驾仿真启动 [进入 V2.2 极致平顺与强劲动力调优]")
print(f"   本地数据日志已建立: {log_file_path}")
print("=" * 60)

with mujoco.viewer.launch_passive(model, data) as viewer:
    step = 0

    while viewer.is_running() and step < len(t_points) - N:
        step_start = time.time()

        true_q = data.qpos[3:7]
        true_yaw = np.arctan2(
            2 * (true_q[0] * true_q[3] + true_q[1] * true_q[2]),
            1 - 2 * (true_q[2] ** 2 + true_q[3] ** 2),
        )

        # 模拟 IMU 噪声注入
        gyro_noise = np.random.normal(0, 0.01, 3)
        accel_noise = np.random.normal(0, 0.05, 3)

        raw_gx = data.qvel[3] + gyro_noise[0]
        raw_gy = data.qvel[4] + gyro_noise[1]
        raw_gz = data.qvel[5] + gyro_noise[2]

        raw_ax = (
            2.0 * (true_q[1] * true_q[3] - true_q[0] * true_q[2]) * 9.81
            + accel_noise[0]
        )
        raw_ay = (
            2.0 * (true_q[0] * true_q[1] + true_q[2] * true_q[3]) * 9.81
            + accel_noise[1]
        )
        raw_az = (
            true_q[0] ** 2 - true_q[1] ** 2 - true_q[2] ** 2 + true_q[3] ** 2
        ) * 9.81 + accel_noise[2]

        # 传感器源头使用一阶低通滤波进行物理级抗噪
        gx = filter_gx.update(raw_gx)
        gy = filter_gy.update(raw_gy)
        gz = filter_gz.update(raw_gz)
        ax = filter_ax.update(raw_ax)
        ay = filter_ay.update(raw_ay)
        az = filter_az.update(raw_az)

        mahony.update_imu(gx, gy, gz, ax, ay, az)
        estimated_yaw = mahony.get_estimated_yaw()

        v_left = data.qvel[qvel_left_idx] * 0.08
        v_right = data.qvel[qvel_right_idx] * 0.08
        current_v = (v_left + v_right) / 2.0

        current_state = np.array([data.qpos[0], data.qpos[1], estimated_yaw, current_v])

        acados_solver.set(0, "lbx", current_state)
        acados_solver.set(0, "ubx", current_state)

        for k in range(N):
            idx = min(step + k, len(t_points) - 1)
            if idx < len(t_points) - 1:
                dy = ref_y[idx + 1] - ref_y[idx]
                dx = ref_x[idx + 1] - ref_x[idx]
                theta_ref = np.arctan2(dy, dx)
            else:
                theta_ref = 0.0

            ocp_yref = np.array(
                [ref_x[idx], ref_y[idx], theta_ref, 0.3, 0.0, 0.0]
            )  # 设定目标速度为 0.3 m/s
            acados_solver.set(k, "yref", ocp_yref)

        idx_e = min(step + N, len(t_points) - 1)
        ocp_yref_e = np.array([ref_x[idx_e], ref_y[idx_e], 0.0, 0.3])
        acados_solver.set(N, "yref", ocp_yref_e)

        status = acados_solver.solve()
        if status != 0:
            log_file.write(f"#{step}_SOLVER_FAILED_STATUS_{status}\n")

        u_opt = acados_solver.get(0, "u")
        a_opt = u_opt[0]
        w_cmd_raw = u_opt[1]

        v_cmd_raw = max(0.0, current_v + a_opt * 0.01)

        # 🛡️ 架构师修正：绝对禁止对 MPC 输出进行低通滤波！MPC 的输出即是最优解，滤波会引入致命的相位滞后。
        v_cmd = v_cmd_raw
        w_cmd = w_cmd_raw

        # 施加控制级极限限幅
        v_cmd = min(0.3, v_cmd)
        w_cmd = np.clip(w_cmd, -0.6, 0.6)

        R = 0.08
        L = 0.36

        # 【核心重构】：恢复数学与物理完全对称的控制极性，彻底闭合负反馈控制环！
        v_left_motor = (v_cmd - w_cmd * L / 2.0) / R
        v_right_motor = (v_cmd + w_cmd * L / 2.0) / R

        # 🛡️ 差速动力学：四轮差速动力同步输出给前、后桥电机通道
        data.ctrl[0] = v_left_motor  # motor_lf
        data.ctrl[1] = v_right_motor  # motor_rf
        data.ctrl[2] = v_left_motor  # motor_lr
        data.ctrl[3] = v_right_motor  # motor_rr

        # 🛡️ 离屏视网膜捕获：每隔 3 步 (仿照物理 33Hz)，对物理相机渲染并推送至通信神经网
        if step % 3 == 0:
            try:
                # 渲染 front_camera 挂载点的当前场景图像 [cite: 1.1.3]
                renderer.update_scene(data, camera="front_camera")
                rgb_frame = renderer.render()  # 返回 (480, 640, 3) 的 np.ndarray
                # 转换成连续的 C 语言裸字节数组并高速写入 Zenoh 管道
                raw_bytes = rgb_frame.tobytes()
                camera_pub.put(raw_bytes)
            except Exception:
                pass

        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.time() - step_start

        if step % 50 == 0:
            target_x = ref_x[step]
            target_y = ref_y[step]
            dist_error = np.sqrt(
                (current_state[0] - target_x) ** 2 + (current_state[1] - target_y) ** 2
            )

            # 记录真实姿态与估计姿态
            log_file.write(
                f"{data.time:.2f},{current_state[0]:.4f},{current_state[1]:.4f},"
                f"{target_x:.4f},{target_y:.4f},{dist_error:.4f},"
                f"{v_cmd:.3f},{w_cmd:.3f},{estimated_yaw:.4f},{true_yaw:.4f},{elapsed * 1000:.1f}\n"
            )

            # 终端实时简化遥测，不刷屏（1行完成）
            print(
                f"Time: {data.time:.2f}s | Err: {dist_error:.3f}m | Solve: {elapsed * 1000:.1f}ms"
            )

        step += 1

        # 路径跑完后保持窗口开启以供审计
        # 找到 mujoco_nmpc_run.py 最后的循环判断，将其修改为：
        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.time() - step_start

        if step % 50 == 0:
            target_x = ref_x[step]
            target_y = ref_y[step]
            dist_error = np.sqrt(
                (current_state[0] - target_x) ** 2 + (current_state[1] - target_y) ** 2
            )

            log_file.write(
                f"{data.time:.2f},{current_state[0]:.4f},{current_state[1]:.4f},"
                f"{target_x:.4f},{target_y:.4f},{dist_error:.4f},"
                f"{v_cmd:.3f},{w_cmd:.3f},{estimated_yaw:.4f},{true_yaw:.4f},{elapsed * 1000:.1f}\n"
            )

            print(
                f"Time: {data.time:.2f}s | Err: {dist_error:.3f}m | Solve: {elapsed * 1000:.1f}ms"
            )

        time.sleep(max(0, 0.01 - elapsed))
        step += 1

        # === 核心修改：到达 15 秒终点后，自动复位回原点重新执行 ===
        if step >= len(t_points) - N:
            print("\n🔄 [到达终点] 正在自动重置底盘与算法，返回原点重新执行...")

            # 1. 物理引擎复位：重置所有位置、速度和仿真时间为 0
            mujoco.mj_resetData(model, data)

            # 2. 软件变量复位
            step = 0
            prev_w_cmd = 0.0

            # 3. 姿态小脑复位：清空 Mahony 滤波器的积累误差
            mahony = MahonyAHRS(sample_freq=100.0, kp=1.5, ki=0.005)
            # === 新增：清除所有低通滤波器的“前世记忆” ===
            filter_v = PT1Filter(cutoff_freq=3.0, sample_freq=100.0)
            filter_w = PT1Filter(cutoff_freq=3.0, sample_freq=100.0)
            filter_gx = PT1Filter(cutoff_freq=15.0, sample_freq=100.0)
            filter_gy = PT1Filter(cutoff_freq=15.0, sample_freq=100.0)
            filter_gz = PT1Filter(cutoff_freq=15.0, sample_freq=100.0)
            filter_ax = PT1Filter(cutoff_freq=5.0, sample_freq=100.0)
            filter_ay = PT1Filter(cutoff_freq=5.0, sample_freq=100.0)
            filter_az = PT1Filter(cutoff_freq=5.0, sample_freq=100.0)

            # 4. 决策大脑复位：重新对 acados 进行时域温启动初始化
            for k in range(N + 1):
                acados_solver.set(k, "x", np.zeros(nx))
            for k in range(N):
                acados_solver.set(k, "u", np.zeros(nu))

            print("✅ 重置成功！开始新一轮智驾追踪...\n")

# 当你手动关闭 3D 窗口时，才会安全退出并保存日志
log_file.close()
viewer.close()
time.sleep(0.5)
print("\n🏆 [仿真安全退出] 数据已写入: nmpc_telemetry.log\n")
