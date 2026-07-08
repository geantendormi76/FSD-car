# generate_solver.py
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
import os
import numpy as np
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
    # 🛡️ SOTA 约束收紧：横向偏差 (Y轴) 权重拉满至 45.0，航向角 (theta) 权重拉至 5.0 [cite: 15]
    # 强制 NMPC 把“完美贴着黄金示教路径走”作为最高准则，绝不在立柱前发生任何割线和偏移！ [cite: 4]
    Q = np.diag([20.0, 45.0, 5.0, 1.0])
    # 降低控制打舵惩罚 R，确保在遇到横向偏差时具备极速、零迟滞的打舵响应
    R = np.diag([0.1, 0.03])

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

    # 3. 设定控制量硬约束 [cite: 24]
    # 我们将最大转向角速度硬限制收紧到 [-0.6, 0.6] rad/s，彻底杜绝原地打转 [cite: 24]
    ocp.constraints.lbu = np.array([-1.0, -0.6])  # 限制角速度在 [-0.6, 0.6] [cite: 24]
    ocp.constraints.ubu = np.array([1.0, 0.6])    # [cite: 24]
    ocp.constraints.idxbu = np.array([0, 1])

    # 4. 初始状态设定
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0])

    # 5. 🎯 战役三核心：配置非线性空间避障硬约束 (Non-linear Constraints)
    # 对应 model.con_h_expr，要求 h_expr >= 1.0
    ocp.constraints.lh = np.array([1.0])       # 下界为 1.0 (必须在椭圆外部)
    ocp.constraints.uh = np.array([1e15])      # 上界为无穷大 (离得越远越好)

    # 🎯 第四版核心重构：激活非线性约束的松弛变量 (Slack Variables / 软约束)
    # 将第 0 个非线性约束 (即 con_h_expr 椭圆禁区) 注册为可松弛的软约束
    ocp.constraints.idxsh = np.array([0])

    # 设定松弛变量的惩罚系数 (L1 & L2 Penalty)
    # zl, zu 是 L1 线性惩罚 (小车轻微触碰边缘时快速做出排斥反应)
    # Zl, Zu 是 L2 二次惩罚 (深度侵入时惩罚呈指数飙升，作为底盘绝对安全防线)
    # 1e3 与 1e5 的黄金配比不仅能够消灭数值病态 (Ill-conditioning)，更保证了数学空间绝对有解
    ocp.cost.zl = np.array([1e3])
    ocp.cost.zu = np.array([1e3])
    ocp.cost.Zl = np.array([1e5])
    ocp.cost.Zu = np.array([1e5])

    # 6. 🎯 战役三核心：初始化外部参数 (Parameters)
    # [obs_x, obs_y, a_axis, b_axis]
    # 默认将虚拟障碍物放在极远处 (1000.0, 1000.0)，避免在无障碍时干扰正常行驶
    # 默认椭圆半轴设为 0.1m，防止除以 0 的数学奇点崩溃
    ocp.parameter_values = np.array([1000.0, 1000.0, 0.1, 0.1])

    # 7. 配置求解器参数 [cite: 3, 9]
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