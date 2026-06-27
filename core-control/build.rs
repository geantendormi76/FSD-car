use std::env;
use std::path::PathBuf;

fn main() {
    // 1. 获取 acados 的安装路径 (优先读取环境变量，否则使用默认沙盘路径)
    let acados_dir = env::var("ACADOS_SOURCE_DIR")
        .unwrap_or_else(|_| "/home/zhz/FSD-car/acados".to_string());
    
    // 2. 指向我们在 simulation-env 中用 Python 生成的 C 代码目录
    let generated_code_dir = PathBuf::from("../simulation-env/c_generated_code");

    // 3. 告诉 Rust 链接器去哪里找 .so 动态库
    println!("cargo:rustc-link-search=native={}/lib", acados_dir);
    println!("cargo:rustc-link-search=native={}", generated_code_dir.display());

    // 4. 声明需要链接的库名称 (去掉 lib 前缀和 .so 后缀)
    println!("cargo:rustc-link-lib=dylib=acados");
    println!("cargo:rustc-link-lib=dylib=acados_solver_diff_drive_car");

    // 5. 缓存守卫：如果 C 代码发生重新生成，强制触发 Rust 重新编译
    println!("cargo:rerun-if-changed={}/acados_solver_diff_drive_car.c", generated_code_dir.display());
}