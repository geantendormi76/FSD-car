// core-control/build.rs
use std::env;
use std::path::PathBuf;

fn main() {
    // 1. 获取当前 crate (core-control) 的绝对物理路径
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // 2. 向上自适应溯源至工作空间根目录 (FSD-car)，彻底告别任何硬编码
    let workspace_root = manifest_dir
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or(manifest_dir);

    // 3. 动态解析并定位 acados 底层 C 库的物理路径 (优先读取环境变量，否则使用工作空间自适应沙盘路径)
    let acados_lib_dir = if let Ok(acados_source) = env::var("ACADOS_SOURCE_DIR") {
        PathBuf::from(acados_source).join("lib")
    } else {
        workspace_root.join("simulation-env/acados/lib")
    };

    // 4. 定位 NMPC 自动生成的 OCP 求解器动态库物理路径
    let solver_lib_dir = workspace_root.join("simulation-env/c_generated_code");

    // -------------------------------------------------------------------------
    // 🛡️ 工业级 RPATH 链接保障契约
    // -------------------------------------------------------------------------

    // A. 绑定链接并注入 RPATH：libacados.so
    if acados_lib_dir.exists() {
        // 告诉 Cargo 编译期去哪里寻找 libacados.so
        println!("cargo:rustc-link-search=native={}", acados_lib_dir.display());
        // SOTA 核心：将动态库路径硬烧录进生成的二进制 ELF 文件（RPATH 机制） [cite: 1.2.8]
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", acados_lib_dir.display());
        // 🛡️ 架构师 2026 级自愈：现代 Ubuntu 默认启用 '--enable-new-dtags'，从而写入不具传递性的 RUNPATH。
        // 这会导致可执行文件无法自动加载依赖库的次级依赖（如 libacados.so 所需的 libqpOASES_e.so）。
        // 强制禁用 new-dtags 回归经典 RPATH，从而实现次级依赖的自动传递解析！
        println!("cargo:rustc-link-arg=-Wl,--disable-new-dtags");
    } else {
        println!(
            "cargo:warning=⚠️ [build.rs] 未找到 acados 核心动态库路径：{}，请检查 acados 是否正确编译安装。", 
            acados_lib_dir.display()
        );
    }

    // B. 绑定链接并注入 RPATH：libacados_ocp_solver_diff_drive_car.so
    if solver_lib_dir.exists() {
        // 编译期搜索
        println!("cargo:rustc-link-search=native={}", solver_lib_dir.display());
        // 运行时 RPATH 路径嵌入 [cite: 1.2.8]
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", solver_lib_dir.display());
    } else {
        println!(
            "cargo:warning=⚠️ [build.rs] 未找到自动生成求解器代码路径：{}，请确保在 simulation-env 目录下运行过 Python 生成器。", 
            solver_lib_dir.display()
        );
    }

    // 5. 声明需要动态链接的库名称
    println!("cargo:rustc-link-lib=dylib=acados");
    // 🛡️ 对齐实际物理生成的动态库名称（带 _ocp_ 命名极性）
    println!("cargo:rustc-link-lib=dylib=acados_ocp_solver_diff_drive_car");

    // 6. 编译防腐哨兵：如果 C 求解器源文件被重新生成，强制触发 Rust 重新编译
    let c_solver_source = solver_lib_dir.join("acados_solver_diff_drive_car.c");
    if c_solver_source.exists() {
        println!("cargo:rerun-if-changed={}", c_solver_source.display());
    }
}