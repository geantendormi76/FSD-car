pub mod perception;

#[cfg(test)]
pub(crate) static MODEL_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

use opencv::core::Mat;

pub trait 仿生图像特征提取器: Send + Sync {
    type 特征点;

    fn 提取特征(
        &self,
        输入图像: &Mat,
        最大角点数: usize,
    ) -> Result<Vec<Self::特征点>, String>;
}

pub trait 仿生路面分割器: Send + Sync {
    fn 分割(&self, 输入图像: &Mat) -> Result<Mat, String>;
}

impl 仿生图像特征提取器 for perception::xfeat_engine::仿生特征提取器 {
    type 特征点 = perception::xfeat_engine::稀疏特征点;

    fn 提取特征(
        &self,
        输入图像: &Mat,
        最大角点数: usize,
    ) -> Result<Vec<Self::特征点>, String> {
        self.提取特征(输入图像, 最大角点数)
    }
}

impl 仿生路面分割器 for perception::pidnet_engine::PidnetEngine {
    fn 分割(&self, 输入图像: &Mat) -> Result<Mat, String> {
        self.segment(输入图像)
    }
}

pub fn self_heal_load_onnx_dylib() {
    if std::env::var_os("ORT_DYLIB_PATH").is_some() {
        return;
    }

    if let Some(directory) = std::env::var_os("ONNXRUNTIME_CAPI_DIR") {
        if set_best_onnx_library(onnx_libraries_in(std::path::Path::new(&directory))) {
            return;
        }
    }
    if set_best_onnx_library(onnx_libraries_in(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("lib_dylib"),
    )) {
        return;
    }
    if let Some(home) = std::env::var_os("HOME") {
        let python_lib = std::path::Path::new(&home).join("isaacsim/kit/python/lib");
        if let Ok(entries) = std::fs::read_dir(python_lib) {
            for python in entries.filter_map(Result::ok) {
                let capi = python.path().join("site-packages/onnxruntime/capi");
                if set_best_onnx_library(onnx_libraries_in(&capi)) {
                    return;
                }
            }
        }
    }
    for directory in ["/usr/lib/x86_64-linux-gnu", "/usr/local/lib", "/usr/lib"] {
        if set_best_onnx_library(onnx_libraries_in(std::path::Path::new(directory))) {
            return;
        }
    }
}

fn set_best_onnx_library(mut candidates: Vec<std::path::PathBuf>) -> bool {
    candidates.sort();
    let Some(path) = candidates.pop() else {
        return false;
    };
    std::env::set_var("ORT_DYLIB_PATH", path);
    true
}

fn onnx_libraries_in(directory: &std::path::Path) -> Vec<std::path::PathBuf> {
    let Ok(entries) = std::fs::read_dir(directory) else {
        return Vec::new();
    };
    entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| {
                    name == "libonnxruntime.so" || name.starts_with("libonnxruntime.so.")
                })
        })
        .collect()
}
