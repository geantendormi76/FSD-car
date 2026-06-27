# nmpc_model.py
from acados_template import AcadosModel
import casadi as ca

def setup_car_model():
    model_name = "diff_drive_car"

    # 1. 定义系统状态量 (States)
    # x: X轴位置, y: Y轴位置, theta: 航向角, v: 前进线速度 [cite: 21]
    x = ca.MX.sym('x')
    y = ca.MX.sym('y')
    theta = ca.MX.sym('theta')
    v = ca.MX.sym('v')
    states = ca.vcat([x, y, theta, v])

    # 2. 定义控制输入量 (Controls)
    # a: 前进加速度, omega: 旋转角速度
    a = ca.MX.sym('a')
    omega = ca.MX.sym('omega')
    controls = ca.vcat([a, omega])

    # 3. 定义状态导数 (State derivatives for ODE)
    x_dot = ca.MX.sym('x_dot')
    y_dot = ca.MX.sym('y_dot')
    theta_dot = ca.MX.sym('theta_dot')
    v_dot = ca.MX.sym('v_dot')
    states_dot = ca.vcat([x_dot, y_dot, theta_dot, v_dot])

    # 4. 连续时间非线性动力学方程 (连续常微分方程) [cite: 21]
    f_expl = ca.vcat([
        v * ca.cos(theta),
        v * ca.sin(theta),
        omega,
        a
    ])

    # 5. 组装 acados 模型对象
    model = AcadosModel()
    model.f_impl_expr = states_dot - f_expl # 隐式表达式
    model.f_expl_expr = f_expl             # 显式表达式
    model.x = states
    model.xdot = states_dot
    model.u = controls
    model.name = model_name

    return model