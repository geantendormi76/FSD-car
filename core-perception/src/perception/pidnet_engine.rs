use opencv::{
    prelude::*,
    core::{self, Mat, Size, Scalar},
    imgproc,
};
use ort::session::Session;
use ort::value::Value;
use ort::session::builder::GraphOptimizationLevel;
use std::sync::Mutex;

pub struct PidnetEngine {
    session: Mutex<Session>,
    model_width: i32,
    model_height: i32,
}

impl PidnetEngine {
    pub fn new<P: AsRef<std::path::Path>>(model_path: P) -> Result<Self, String> {
        crate::self_heal_load_onnx_dylib();

        let session = Session::builder()
            .map_err(|e| e.to_string())?
            .with_execution_providers([
                ort::ep::CUDA::default().build(),
                ort::ep::CPU::default().build(),
            ])
            .map_err(|e| e.to_string())?
            .with_optimization_level(GraphOptimizationLevel::Level3)
            .map_err(|e| e.to_string())?
            .with_intra_threads(1)
            .map_err(|e| e.to_string())?
            .commit_from_file(model_path)
            .map_err(|e| e.to_string())?;

        Ok(Self {
            session: Mutex::new(session),
            model_width: 640,
            model_height: 480,
        })
    }

    pub fn segment(&self, input_image: &Mat) -> Result<Mat, String> {
        let mut resized = Mat::default();
        imgproc::resize(
            input_image,
            &mut resized,
            Size::new(self.model_width, self.model_height),
            0.0,
            0.0,
            imgproc::INTER_LINEAR,
        ).map_err(|e| e.to_string())?;

        let mut rgb_image = Mat::default();
        imgproc::cvt_color(&resized, &mut rgb_image, imgproc::COLOR_BGR2RGB, 0)
            .map_err(|e| e.to_string())?;

        let h = self.model_height as usize;
        let w = self.model_width as usize;
        let mut tensor_data = vec![0.0f32; 3 * h * w];

        let mean = [0.485, 0.456, 0.406];
        let std = [0.229, 0.224, 0.225];

        for y in 0..h {
            let row = rgb_image.ptr(y as i32).map_err(|e| e.to_string())?;
            let row_slice = unsafe { std::slice::from_raw_parts(row, w * 3) };
            for x in 0..w {
                let r = row_slice[x * 3] as f32 / 255.0;
                let g = row_slice[x * 3 + 1] as f32 / 255.0;
                let b = row_slice[x * 3 + 2] as f32 / 255.0;

                tensor_data[(0 * h + y) * w + x] = (r - mean[0]) / std[0];
                tensor_data[(1 * h + y) * w + x] = (g - mean[1]) / std[1];
                tensor_data[(2 * h + y) * w + x] = (b - mean[2]) / std[2];
            }
        }

        let shape = [1, 3, h, w];
        let input_value = Value::from_array((shape, tensor_data)).map_err(|e| e.to_string())?;
        let input_tensor = ort::inputs![input_value];

        let mut session_lock = self.session.lock().map_err(|e| e.to_string())?;
        let outputs = session_lock.run(input_tensor).map_err(|e| e.to_string())?;

        let (_shape, output_data) = outputs[0].try_extract_tensor::<f32>().map_err(|e| e.to_string())?;

        let out_h = h / 8;
        let out_w = w / 8;

        let mut small_class_map = Mat::new_rows_cols_with_default(out_h as i32, out_w as i32, core::CV_8UC1, Scalar::all(0.0))
            .map_err(|e| e.to_string())?;

        for y in 0..out_h {
            let row_ptr = small_class_map.ptr_mut(y as i32).map_err(|e| e.to_string())?;
            let row_slice = unsafe { std::slice::from_raw_parts_mut(row_ptr, out_w) };
            for x in 0..out_w {
                let mut max_val = -f32::INFINITY;
                let mut max_class = 0u8;
                for c in 0..19 {
                    let val = output_data[((c * out_h) + y) * out_w + x];
                    if val > max_val {
                        max_val = val;
                        max_class = c as u8;
                    }
                }
                row_slice[x] = max_class;
            }
        }

        let mut class_map = Mat::default();
        imgproc::resize(
            &small_class_map,
            &mut class_map,
            Size::new(self.model_width, self.model_height),
            0.0,
            0.0,
            imgproc::INTER_NEAREST,
        ).map_err(|e| e.to_string())?;

        Ok(class_map)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn test_pidnet_loading_and_shapes() {
        let model_path = Path::new("../model/pidnet_s.onnx");
        if !model_path.exists() {
            println!("Skipping unit test: model not found at {:?}", model_path);
            return;
        }
        let engine = PidnetEngine::new(model_path).unwrap();
        assert_eq!(engine.model_width, 640);
        assert_eq!(engine.model_height, 480);
        println!("SUCCESS: PIDNet Engine loaded and initialized successfully!");
    }
}
