// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
/*
=================================================================
🛰️  [NEXUS 感知抽象层] 仿生特征与分割器 Trait 契约中心 (v3.0)
设计哲学: 类型即契约 | 屏蔽底层 ONNX/RKNN 硬件与推理细节
=================================================================
*/
pub mod perception;

use opencv::core::Mat;

/// 🛡️ 仿生图像局部特征提取器契约
pub trait 仿生图像特征提取器: Send + Sync {
    type 特征点;
    
    /// 输入原始图像 Mat 帧，输出提取后的局部稀疏特征点集合
    fn 提取特征(
        &self, 
        输入图像: &Mat, 
        最大角点数: usize
    ) -> Result<Vec<Self::特征点>, String>;
}

/// 🛡️ 仿生路面分类与分割器契约
pub trait 仿生路面分割器: Send + Sync {
    /// 输入图像 Mat 帧，输出统一分类编号的语义分割单通道矩阵
    fn 分割(&self, 输入图像: &Mat) -> Result<Mat, String>;
}

// -------------------------------------------------------------------------
// 🛰️ 静态分发适配：为已有具体实现类无损并网实现 Trait 契约
// -------------------------------------------------------------------------

impl 仿生图像特征提取器 for perception::xfeat_engine::仿生特征提取器 {
    type 特征点 = perception::xfeat_engine::稀疏特征点;
    
    fn 提取特征(
        &self, 
        输入图像: &Mat, 
        最大角点数: usize
    ) -> Result<Vec<Self::特征点>, String> {
        self.提取特征(输入图像, 最大角点数)
    }
}

impl 仿生路面分割器 for perception::pidnet_engine::PidnetEngine {
    fn 分割(&self, 输入图像: &Mat) -> Result<Mat, String> {
        self.segment(输入图像)
    }
}

/// 🛡️ 架构自愈：装载 WSL2/Ubuntu 动态库运行时，防止 dlopen 闪退
pub fn self_heal_load_onnx_dylib() {
    if std::env::var("ORT_DYLIB_PATH").is_ok() {
        return; 
    }
    let capi_dir = "/home/zhz/isaacsim/kit/python/lib/python3.12/site-packages/onnxruntime/capi";
    if std::path::Path::new(capi_dir).exists() {
        if let Ok(entries) = std::fs::read_dir(capi_dir) {
            for entry in entries {
                if let Ok(entry) = entry {
                    let path = entry.path();
                    if let Some(file_name) = path.file_name() {
                        let name_str = file_name.to_string_lossy();
                        if name_str.starts_with("libonnxruntime.so") {
                            let abs_path = path.to_string_lossy().into_owned();
                            std::env::set_var("ORT_DYLIB_PATH", abs_path);
                            return;
                        }
                    }
                }
            }
        }
    }
    let fallback_path = "/home/zhz/fsd-car/core-perception/lib_dylib/libonnxruntime.so";
    if std::path::Path::new(fallback_path).exists() {
        std::env::set_var("ORT_DYLIB_PATH", fallback_path);
    }
}
