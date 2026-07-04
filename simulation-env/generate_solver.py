# generate_solver.py

import os
import numpy as np

# 🛡️ 架构师 2026 净化：彻底移除 Windows/WSL 时代的 ctypes.CDLL 内存预热 Hack。
# 在原生的 Ubuntu 26.04 LTS 下，我们严格依赖标准的 LD_LIBRARY_PATH 或 RPATH 机制。
# 保持 Python 运行环境的绝对纯净与高内聚。

from acados_template import AcadosOcp, AcadosOcpSolver
from nmpc_model import setup_car_model


def generate_nmpc_solver():
    ocp = AcadosOcp()
    model = setup_car_model()
    ocp.model = model

    # 1. 设定时域参数
    N = 20  # 预测步长 (Horizon)
    T_pred = 1.0  # 预测总时间 (1秒)
    ocp.dims.N = N
    ocp.solver_options.tf = T_pred

    # 2. 设定代价函数 (Q 权重增加，R 控制惩罚加大以获得极致平顺感) [cite: 21]
    Q = np.diag([20.0, 20.0, 2.0, 1.0])
    # 🛡️ 架构师修正：释放底层动力。NMPC 本身已具备平滑约束，过大的 R 会导致转弯迟钝、轨迹漂移。
    R = np.diag([0.1, 0.05])

    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    Vx_e = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
    ocp.cost.W_e = Q * 1.5

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # 3. 设定约束条件 [cite: 24]
    # 我们将最大转向角速度硬限制收紧到 [-0.6, 0.6] rad/s，彻底杜绝原地打转 [cite: 24]
    ocp.constraints.lbu = np.array([-1.0, -0.6])  # 限制角速度在 [-0.6, 0.6] [cite: 24]
    ocp.constraints.ubu = np.array([1.0, 0.6])  # [cite: 24]
    ocp.constraints.idxbu = np.array([0, 1])

    # 4. 初始状态设定
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0])

    # 5. 配置求解器参数 [cite: 3, 9]
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"

    # 一键自动编译生成 C 代码 [cite: 8]
    json_file = os.path.join("./", "acados_ocp.json")
    solver = AcadosOcpSolver(ocp, json_file=json_file)
    return solver


if __name__ == "__main__":
    generate_nmpc_solver()
    print("====== acados C代码求解器生成并编译成功！ ======")
