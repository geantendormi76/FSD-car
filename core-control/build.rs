use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() {
    // 1. 获取当前 crate (core-control) 的绝对物理路径
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    
    // 2. 溯源至多包工作空间的物理根目录 (FSD-car)
    let workspace_root = match manifest_dir.parent() {
        Some(parent) => parent.to_path_buf(),
        None => {
            println!("cargo:warning=❌ 无法向上溯源工作空间根目录，回退至当前目录！");
            manifest_dir.clone()
        }
    };
    
    // 3. 定位全局资产总库 lib_dylib 的绝对物理路径
    let lib_dylib_dir = workspace_root.join("lib_dylib");
    if !lib_dylib_dir.exists() {
        fs::create_dir_all(&lib_dylib_dir).expect("❌ 无法创建 lib_dylib 资产库！");
    }

    // 4. 定义规控底层库（C 语言动态链接库）的标准绝对物理路径
    let acados_lib_src = workspace_root.join("simulation-env/acados/lib/libacados.so");
    let solver_lib_src = workspace_root.join("simulation-env/c_generated_code/libacados_solver_diff_drive_car.so");

    // 5. 🛡️ 【智能侦察机制】：如果标准路径缺失，深度遍历工作空间自动搜寻底层库，彻底干掉硬编码
    let mut actual_acados_path = if acados_lib_src.exists() {
        Some(acados_lib_src)
    } else {
        println!("cargo:warning=🔍 [智能侦察] 未在标准路径下找到 libacados.so，启动工作空间深度检索...");
        let scouted = scout_library(&workspace_root, "libacados.so");
        if scouted.is_none() {
            println!("cargo:warning=⚠️ [智能侦察] 深度检索也未能定位 libacados.so，请确保 acados 已正确编译！");
        }
        scouted
    };

    let actual_solver_path = if solver_lib_src.exists() {
        Some(solver_lib_src)
    } else {
        println!("cargo:warning=🔍 [智能侦察] 未在标准路径下找到求解器，启动工作空间深度检索...");
        let scouted = scout_library(&workspace_root, "libacados_solver_diff_drive_car.so");
        if scouted.is_none() {
            println!("cargo:warning=⚠️ [智能侦察] 未能定位求解器，请确保在 simulation-env 下运行了 python generate_solver.py！");
        }
        scouted
    };

    // 6. 执行【编译期资产并网契约】：将侦察到的底层 .so 动态库自动同步同步到全局资产库 lib_dylib 中
    if let Some(src_path) = actual_acados_path {
        let dest = lib_dylib_dir.join("libacados.so");
        if fs::copy(&src_path, &dest).is_ok() {
            println!("cargo:warning=✓ [资产并网成功] 成功同步 {} 到 lib_dylib", src_path.display());
        }
    }

    if let Some(src_path) = actual_solver_path {
        let dest = lib_dylib_dir.join("libacados_solver_diff_drive_car.so");
        if fs::copy(&src_path, &dest).is_ok() {
            println!("cargo:warning=✓ [资产并网成功] 成功同步 {} 到 lib_dylib", src_path.display());
        }
    }

    // 7. 告诉 Cargo 唯一受信任的物理链接搜寻路径：全局资产总库 lib_dylib (绝对路径)
    println!("cargo:rustc-link-search=native={}", lib_dylib_dir.display());

    // 8. 强力绑定 C 语言底层动力符号
    println!("cargo:rustc-link-lib=dylib=acados");
    println!("cargo:rustc-link-lib=dylib=acados_solver_diff_drive_car");

    // 9. 编译防腐哨兵：如果底层的 C 代码发生重构，自动触发 Rust 规控模块重新编译
    let c_solver_source = workspace_root.join("simulation-env/c_generated_code/acados_solver_diff_drive_car.c");
    if c_solver_source.exists() {
        println!("cargo:rerun-if-changed={}", c_solver_source.display());
    }
}

/// 🛡️ 工作空间深度检索算法
/// 自动过滤大文件目录和环境目录，秒级定位物理资产，杜绝路径写死
fn scout_library(dir: &Path, target_lib_name: &str) -> Option<PathBuf> {
    if dir.is_dir() {
        if let Ok(entries) = fs::read_dir(dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
                    // 避开编译产物目录、虚拟环境与 Conda 目录，防止无限递归与编译延迟
                    if name == "target" || name.starts_with('.') || name == "miniconda3" || name == ".venv" || name == "node_modules" {
                        continue;
                    }
                    if let Some(found) = scout_library(&path, target_lib_name) {
                        return Some(found);
                    }
                } else if path.file_name().and_then(|s| s.to_str()) == Some(target_lib_name) {
                    return Some(path);
                }
            }
        }
    }
    None
}