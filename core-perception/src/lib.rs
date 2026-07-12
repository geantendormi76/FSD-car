pub mod perception;

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
