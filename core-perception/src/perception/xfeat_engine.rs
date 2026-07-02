use opencv::{
    prelude::*,
    core::{self, Mat, Size, Scalar},
    imgproc,
};
use ort::session::Session;
use ort::value::Value;

/// 🛡️ 领域模型：XFeat 单特征点数据载荷
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct 稀疏特征点 {
    pub x: f32,
    pub y: f32,
    pub 置信度: f32,
    /// 64 维局部高精度描述子特征
    pub 描述子: Vec<f32>,
}

/// 🛡️ XFeat 仿生视觉特征提取引擎
pub struct 仿生特征提取器 {
    // 🛡️ 工业级并发自愈：在 ort 2.0-rc.10+ 中，Session::run 被升级为了 &mut self 独占借用。
    // 我们在此通过 Mutex 建立内部可变性（Interior Mutability）状态金库，
    // 使得高层业务能继续以只读引用 &self 的形式进行多线程安全并发调用，彻底解决 Arc 共享生命周期死锁！
    推理会话: std::sync::Mutex<Session>,
    模型宽度: i32,
    模型高度: i32,
}

impl 仿生特征提取器 {
    /// 实例化提取器并加载本地 ONNX 权重
    pub fn new<P: std::convert::AsRef<std::path::Path>>(model_path: P) -> Result<Self, String> {
        // 创建高性能 ONNX 运行环境
        let 推理会话 = Session::builder()
            .map_err(|e| e.to_string())?
            .with_intra_threads(1)
            .map_err(|e| e.to_string())?
            .commit_from_file(model_path)
            .map_err(|e| e.to_string())?;

        Ok(Self {
            推理会话: std::sync::Mutex::new(推理会话),
            模型宽度: 640,
            模型高度: 640,
        })
    }

    /// 双三次权重插值辅助算子
    fn 计算双三次插值权重(t: f32) -> (f32, f32, f32, f32) {
        let a = -0.75f32;
        let t2 = t * t;
        let t3 = t2 * t;
        let wm1 = a * (t3 - 2.0 * t2 + t);
        let w0  = (a + 2.0) * t3 - (a + 3.0) * t2 + 1.0;
        let w1  = -(a + 2.0) * t3 + (2.0 * a + 3.0) * t2 - a * t;
        let w2  = a * (-t3 + t2);
        (wm1, w0, w1, w2)
    }

    /// 核心处理：通过亚像素级插值获取 64 维描述子
    fn 插值描述子(&self, 描述子数据: &[f32], 目标缓冲: &mut [f32], 坐标_x: f32, 坐标_y: f32) {
        let x0 = 坐标_x.floor() as i32;
        let y0 = 坐标_y.floor() as i32;
        let xm1 = x0 - 1;
        let ym1 = y0 - 1;
        let dx = 坐标_x - x0 as f32;
        let dy = 坐标_y - y0 as f32;

        let (wxm1, wx0, wx1, wx2) = Self::计算双三次插值权重(dx);
        let (wym1, wy0, wy1, wy2) = Self::计算双三次插值权重(dy);

        let 宽d8 = self.模型宽度 / 8;
        let 高d8 = self.模型高度 / 8;

        let get_val_ptr = |y_idx: i32, x_idx: i32| -> usize {
            let clamped_y = y_idx.clamp(0, 高d8 - 1) as usize;
            let clamped_x = x_idx.clamp(0, 宽d8 - 1) as usize;
            (clamped_y * 宽d8 as usize + clamped_x) * 64
        };

        let ptr_xm1_ym1 = get_val_ptr(ym1, xm1);
        let ptr_x0_ym1 = get_val_ptr(ym1, x0);
        let ptr_x1_ym1 = get_val_ptr(ym1, x0 + 1);
        let ptr_x2_ym1 = get_val_ptr(ym1, x0 + 2);

        let ptr_xm1_y0 = get_val_ptr(y0, xm1);
        let ptr_x0_y0 = get_val_ptr(y0, x0);
        let ptr_x1_y0 = get_val_ptr(y0, x0 + 1);
        let ptr_x2_y0 = get_val_ptr(y0, x0 + 2);

        let ptr_xm1_y1 = get_val_ptr(y0 + 1, xm1);
        let ptr_x0_y1 = get_val_ptr(y0 + 1, x0);
        let ptr_x1_y1 = get_val_ptr(y0 + 1, x0 + 1);
        let ptr_x2_y1 = get_val_ptr(y0 + 1, x0 + 2);

        let ptr_xm1_y2 = get_val_ptr(y0 + 2, xm1);
        let ptr_x0_y2 = get_val_ptr(y0 + 2, x0);
        let ptr_x1_y2 = get_val_ptr(y0 + 2, x0 + 1);
        let ptr_x2_y2 = get_val_ptr(y0 + 2, x0 + 2);

        let mut 平方和 = 0.0f32;
        for i in 0..64 {
            let v_m1 = wxm1 * 描述子数据[ptr_xm1_ym1 + i] + wx0 * 描述子数据[ptr_x0_ym1 + i] + wx1 * 描述子数据[ptr_x1_ym1 + i] + wx2 * 描述子数据[ptr_x2_ym1 + i];
            let v_0  = wxm1 * 描述子数据[ptr_xm1_y0 + i]  + wx0 * 描述子数据[ptr_x0_y0 + i]  + wx1 * 描述子数据[ptr_x1_y0 + i]  + wx2 * 描述子数据[ptr_x2_y0 + i];
            let v_1  = wxm1 * 描述子数据[ptr_xm1_y1 + i]  + wx0 * 描述子数据[ptr_x0_y1 + i]  + wx1 * 描述子数据[ptr_x1_y1 + i]  + wx2 * 描述子数据[ptr_x2_y1 + i];
            let v_2  = wxm1 * 描述子数据[ptr_xm1_y2 + i]  + wx0 * 描述子数据[ptr_x0_y2 + i]  + wx1 * 描述子数据[ptr_x1_y2 + i]  + wx2 * 描述子数据[ptr_x2_y2 + i];

            let v = wym1 * v_m1 + wy0 * v_0 + wy1 * v_1 + wy2 * v_2;
            目标缓冲[i] = v;
            平方和 += v * v;
        }

        // L2 归一化描述子
        let 模长倒数 = 1.0 / (平方和.sqrt().max(1e-12));
        for i in 0..64 {
            目标缓冲[i] *= 模长倒数;
        }
    }

    /// 核心图像计算：提取当前帧的 XFeat 稀疏特征点集合
    pub fn 提取特征(&self, 输入图像: &Mat, 最大角点数: usize) -> Result<Vec<稀疏特征点>, String> {
        let (宽, 高) = (输入图像.cols(), 输入图像.rows());
        
        // 1. 物理边缘对齐：将图像通过 Padding 补边到 640x640
        let mut 适配帧 = Mat::default();
        if 宽 != self.模型宽度 || 高 != self.模型高度 {
            let 偏移_x = (self.模型宽度 - 宽) / 2;
            let 偏移_y = (self.模型高度 - 高) / 2;
            core::copy_make_border(
                输入图像,
                &mut 适配帧,
                偏移_y, self.模型高度 - 高 - 偏移_y,
                偏移_x, self.模型宽度 - 宽 - 偏移_x,
                core::BORDER_CONSTANT,
                Scalar::all(0.0)
            ).map_err(|e| e.to_string())?;
        } else {
            适配帧 = 输入图像.clone();
        }

        // 2. 图像转单通道灰度（XFeat 要求 1 通道输入）
        let mut 灰度帧 = Mat::default();
        if 适配帧.channels() == 3 {
            imgproc::cvt_color(&适配帧, &mut 灰度帧, imgproc::COLOR_BGR2GRAY, 0).map_err(|e| e.to_string())?;
        } else {
            灰度帧 = 适配帧.clone();
        }

        // 3. 图像转浮点并完成 Z-score 均值归一化 (修复 subtract & divide2 兼容参数)
        let mut 浮点帧 = Mat::default();
        灰度帧.convert_to(&mut 浮点帧, core::CV_32F, 1.0, 0.0).map_err(|e| e.to_string())?;
        
        let mut 均值 = Mat::default();
        let mut 标准差 = Mat::default();
        core::mean_std_dev(&浮点帧, &mut 均值, &mut 标准差, &core::no_array()).map_err(|e| e.to_string())?;
        
        // 🛡️ 借用安全自愈：利用 `归一化帧` 建立乒乓双缓存，彻底避开 Rust 编译器对
        // subtract 与 divide2 算子中输入输出同矩阵导致的 Double Borrow（可变借用冲突）！
        let mut 归一化帧 = Mat::default();
        core::subtract(&浮点帧, &均值, &mut 归一化帧, &core::no_array(), -1).map_err(|e| e.to_string())?;
        core::divide2(&归一化帧, &标准差, &mut 浮点帧, 1.0, -1).map_err(|e| e.to_string())?;

        // 4. 将 OpenCV 浮点像素强转为 f32
        let mut 张量数据 = vec![0.0f32; (self.模型宽度 * self.模型高度) as usize];
        for y in 0..self.模型高度 {
            let 行数据_u8 = 浮点帧.ptr(y).map_err(|e| e.to_string())?;
            let 行数据_f32 = 行数据_u8 as *const f32; // 手动指针转换
            let slice = unsafe { std::slice::from_raw_parts(行数据_f32, self.模型宽度 as usize) };
            let start = (y * self.模型宽度) as usize;
            张量数据[start..start + self.模型宽度 as usize].copy_from_slice(slice);
        }

        // 5. 🛡️ 极速物理自愈：直接利用 `([usize; N], Vec<f32>)` 结构体装载 Tensor。
        // 注意：在 ort 中，ToShape 特征没有为 tuple (即圆括号) 实现，但已经为原生 array [即方括号] 实现了！
        // 彻底切断 ndarray 链条，100% 免疫任何复杂的依赖版本冲突！
        let 维度元组 = (
            [1usize, 1, self.模型高度 as usize, self.模型宽度 as usize], 
            张量数据
        );
        let 输入值 = Value::from_array(维度元组).map_err(|e| e.to_string())?;

        // 5. ONNX 引擎前向推理
        let 采样输入 = ort::inputs![输入值];
        
        // 🛡️ 独占锁自愈：在计算边界内临时锁住推理会话，安全借出 &mut Session 完成推理后立即自动释放锁！
        let mut 会话锁 = self.推理会话.lock().map_err(|e| e.to_string())?;
        let 输出 = 会话锁.run(采样输入).map_err(|e| e.to_string())?;

        // 提取 1D 裸切片元组 ((&Shape, &[f32]))，规避 ndarray 高版本兼容断裂
        let (_描述子形状, 描述子数据) = 输出[0].try_extract_tensor::<f32>().map_err(|e| e.to_string())?;
        let (_置信度形状, 置信度数据) = 输出[1].try_extract_tensor::<f32>().map_err(|e| e.to_string())?;
        let (_可靠性形状, 可靠性数据) = 输出[2].try_extract_tensor::<f32>().map_err(|e| e.to_string())?;

        let 宽d8 = self.模型宽度 / 8;
        let 高d8 = self.模型高度 / 8;

        // 7. 置信度 Softmax 概率重整与 Flatten 展平（1D 极速展平寻址）
        let mut 概率展平图 = vec![0.0f32; (self.模型宽度 * self.模型高度) as usize];
        for y_c in 0..高d8 as usize {
            for x_c in 0..宽d8 as usize {
                let cell_offset = (y_c * 宽d8 as usize + x_c) * 65;
                
                let mut cell_scores = [0.0f32; 65];
                let mut max_val = -f32::INFINITY;
                for c in 0..65 {
                    let val = 置信度数据[cell_offset + c];
                    cell_scores[c] = val;
                    if val > max_val { max_val = val; }
                }

                let mut sum = 0.0f32;
                for c in 0..65 {
                    cell_scores[c] = (cell_scores[c] - max_val).exp();
                    sum += cell_scores[c];
                }
                let inv_sum = 1.0 / sum;
                for c in 0..65 {
                    cell_scores[c] *= inv_sum;
                }

                // 废弃第 65 通道，将 64 个通道映射回 8x8 网格中
                for k in 0..8 {
                    for l in 0..8 {
                        let val_idx = k * 8 + l;
                        let dst_y = y_c * 8 + k;
                        let dst_x = x_c * 8 + l;
                        概率展平图[dst_y * self.模型宽度 as usize + dst_x] = cell_scores[val_idx];
                    }
                }
            }
        }

        // 8. 可靠性图插值放大并与置信度相乘，排除非稳定噪点
        let mut 可靠性小图 = Mat::new_rows_cols_with_default(高d8, 宽d8, core::CV_32F, Scalar::all(0.0)).map_err(|e| e.to_string())?;
        for y in 0..高d8 {
            let row_u8 = 可靠性小图.ptr_mut(y).map_err(|e| e.to_string())?;
            let row_f32 = row_u8 as *mut f32; // 指针强转
            let slice = unsafe { std::slice::from_raw_parts_mut(row_f32, 宽d8 as usize) };
            for x in 0..宽d8 {
                slice[x as usize] = 可靠性数据[y as usize * 宽d8 as usize + x as usize];
            }
        }

        let mut 可靠性大图 = Mat::default();
        imgproc::resize(&可靠性小图, &mut 可靠性大图, Size::new(self.模型宽度, self.模型高度), 0.0, 0.0, imgproc::INTER_LINEAR).map_err(|e| e.to_string())?;

        // 9. 5x5 极限 NMS (Non-Maximum Suppression) 并剔除边缘点
        let nms_kernel = 5;
        let half_k = nms_kernel / 2;
        let 阈值 = 0.05f32;
        let mut 候选特征点 = Vec::new();

        for y in half_k..(self.模型高度 - half_k) {
            for x in half_k..(self.模型宽度 - half_k) {
                let current_idx = (y * self.模型宽度 + x) as usize;
                let original_score = 概率展平图[current_idx];
                
                let reliability = *可靠性大图.at_2d::<f32>(y, x).map_err(|e| e.to_string())?;
                let score = original_score * reliability;

                if score <= 阈值 { continue; }

                let mut is_max = true;
                for ky in -half_k..=half_k {
                    for kx in -half_k..=half_k {
                        if kx == 0 && ky == 0 { continue; }
                        let n_idx = ((y + ky) * self.模型宽度 + (x + kx)) as usize;
                        let n_reliability = *可靠性大图.at_2d::<f32>(y + ky, x + kx).map_err(|e| e.to_string())?;
                        let n_score = 概率展平图[n_idx] * n_reliability;
                        if score < n_score {
                            is_max = false;
                            break;
                        }
                    }
                    if !is_max { break; }
                }

                if is_max && x > 12 && x < self.模型宽度 - 12 && y > 12 && y < self.模型高度 - 12 {
                    候选特征点.push((x, y, score));
                }
            }
        }

        候选特征点.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());

        // 10. 双三次亚像素插值描述子，计算 64 维特征描述子
        let 缩放_x = (宽d8 as f32) / (self.模型宽度 as f32 - 1.0);
        let 缩放_y = (高d8 as f32) / (self.模型高度 as f32 - 1.0);
        let mut 最终结果 = Vec::new();

        for (px, py, score) in 候选特征点.into_iter().take(最大角点数) {
            let interp_x = (px as f32) * 缩放_x - 0.5;
            let interp_y = (py as f32) * 缩放_y - 0.5;

            let mut 描述子 = vec![0.0f32; 64];
            self.插值描述子(描述子数据, &mut 描述子, interp_x, interp_y);
            
            let 还原_x = px as f32 - ((self.模型宽度 - 宽) / 2) as f32;
            let 还原_y = py as f32 - ((self.模型高度 - 高) / 2) as f32;

            最终结果.push(稀疏特征点 {
                x: 还原_x,
                y: 还原_y,
                置信度: score,
                描述子,
            });
        }

        Ok(最终结果)
    }
}