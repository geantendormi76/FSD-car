# nmpc_model.py
# 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
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

    # 5. 🎯 战役三核心：引入外部实时参数 (Parameters)
    # obs_x, obs_y: 虚拟动态障碍物中心坐标
    # a_axis, b_axis: 膨胀椭圆的半长轴和半短轴 (安全边界)
    obs_x = ca.MX.sym('obs_x')
    obs_y = ca.MX.sym('obs_y')
    a_axis = ca.MX.sym('a_axis')
    b_axis = ca.MX.sym('b_axis')
    parameters = ca.vcat([obs_x, obs_y, a_axis, b_axis])

    # 6. 🎯 战役三核心：构建非线性空间避障硬约束 (Non-linear Constraint)
    # 椭圆方程：(x - obs_x)^2 / a_axis^2 + (y - obs_y)^2 / b_axis^2 >= 1
    # 我们将表达式定义为 h_expr，稍后在 generate_solver 中限制其下界为 1.0
    h_expr = ((x - obs_x)**2) / (a_axis**2) + ((y - obs_y)**2) / (b_axis**2)

    # 7. 组装 acados 模型对象
    model = AcadosModel()
    model.f_impl_expr = states_dot - f_expl # 隐式表达式
    model.f_expl_expr = f_expl              # 显式表达式
    model.x = states
    model.xdot = states_dot
    model.u = controls
    model.p = parameters                    # 注册参数金库
    model.con_h_expr = h_expr               # 注册非线性约束表达式
    model.name = model_name

    return model