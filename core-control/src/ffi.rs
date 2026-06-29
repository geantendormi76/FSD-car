#![allow(non_camel_case_types)]
use libc::{c_char, c_int, c_void};

/// 🛡️ 不透明结构体：代表 acados 内部的求解器内存胶囊
#[repr(C)]
pub struct diff_drive_car_solver_capsule {
    _unused: [u8; 0],
}

// 声明外部 C 函数符号
extern "C" {
    // 生命周期管理 (现代 Acados API 契约)
    pub fn diff_drive_car_acados_create_capsule() -> *mut diff_drive_car_solver_capsule;
    pub fn diff_drive_car_acados_create(capsule: *mut diff_drive_car_solver_capsule) -> c_int;
    pub fn diff_drive_car_acados_solve(capsule: *mut diff_drive_car_solver_capsule) -> c_int;
    pub fn diff_drive_car_acados_free(capsule: *mut diff_drive_car_solver_capsule) -> c_int;
    pub fn diff_drive_car_acados_free_capsule(capsule: *mut diff_drive_car_solver_capsule) -> c_int;

    // 内部组件指针获取
    pub fn diff_drive_car_acados_get_nlp_in(capsule: *mut diff_drive_car_solver_capsule) -> *mut c_void;
    pub fn diff_drive_car_acados_get_nlp_out(capsule: *mut diff_drive_car_solver_capsule) -> *mut c_void;
    pub fn diff_drive_car_acados_get_nlp_config(capsule: *mut diff_drive_car_solver_capsule) -> *mut c_void;
    pub fn diff_drive_car_acados_get_nlp_dims(capsule: *mut diff_drive_car_solver_capsule) -> *mut c_void;

    // 通用数据注入与提取算子
    pub fn ocp_nlp_in_set(
        config: *mut c_void,
        dims: *mut c_void,
        nlp_in: *mut c_void,
        stage: c_int,
        field: *const c_char,
        value: *mut c_void,
    ) -> c_int;

    // 🛡️ 架构师 2026 SOTA 修正：现代 Acados 强制将约束与代价函数设置隔离为专用函数，废除万能的 in_set
    pub fn ocp_nlp_constraints_model_set(
        config: *mut c_void,
        dims: *mut c_void,
        nlp_in: *mut c_void,
        nlp_out: *mut c_void, 
        stage: c_int,
        field: *const c_char,
        value: *mut c_void,
    ) -> c_int;

    pub fn ocp_nlp_cost_model_set(
        config: *mut c_void,
        dims: *mut c_void,
        nlp_in: *mut c_void,
        stage: c_int,
        field: *const c_char,
        value: *mut c_void,
    ) -> c_int;

    pub fn ocp_nlp_out_get(
        config: *mut c_void,
        dims: *mut c_void,
        nlp_out: *mut c_void,
        stage: c_int,
        field: *const c_char,
        value: *mut c_void,
    ); 
}