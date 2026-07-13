// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。

use super::xfeat_engine::稀疏特征点;
use opencv::{
    prelude::*,
    core::{self, Point2f},
    calib3d,
};
use std::sync::Mutex;
use ort::session::Session;
use ort::value::Value;
use ort::session::builder::GraphOptimizationLevel;

/// 🚀 架构师对齐优化：视觉亚像素纠偏“显微镜”引擎
/// 支持双通道自愈防线：首选 8x8 Refinement MLP 神经网络推理，未检测到权重时自动安全退避至【数学级二次梯度自愈插值算子】
pub struct 仿生亚像素显微镜 {
    推理会话: Option<Mutex<Session>>,
}

impl 仿生亚像素显微镜 {
    /// 实例化视觉显微镜引擎
    pub fn new() -> Self {
        let model_path = "model/refinement_mlp.onnx";
        if !std::path::Path::new(model_path).exists() {
            println!("⚠️ [视觉显微镜] 未检测到 model/refinement_mlp.onnx 物理权重，自动激活【数学级二次梯度自愈算子】(0 NPU开销)。");
            return Self { 推理会话: None };
        }

        // 绑定异构加速执行器，采用与 XFeat 骨干网络完全相同的级联分配策略 [cite: 1.1.2]
        let session_res = Session::builder()
            .and_then(|b| b.with_execution_providers([
                ort::ep::CUDA::default().build(),
                ort::ep::CPU::default().build(),
            ]))
            .and_then(|b| b.with_optimization_level(GraphOptimizationLevel::Level3))
            .and_then(|b| b.with_intra_threads(1))
            .and_then(|b| b.commit_from_file(model_path));

        match session_res {
            Ok(session) => {
                println!("🟢 [视觉显微镜] 亚像素 Refinement MLP 神经核并网成功，已激活 NPU/GPU 硬件加速！");
                Self { 推理会话: Some(Mutex::new(session)) }
            }
            Err(e) => {
                println!("⚠️ [视觉显微镜] ONNX 实例化失败: {:?}，自动安全退让至数学插值自愈兜底防线", e);
                Self { 推理会话: None }
            }
        }
    }

    /// ⚡ [主通道 - 神经显微镜]：拼接 64+64 描述子进行 8x8 偏移概率网格推理 [cite: 1.2.2]
    pub fn 预测亚像素偏移(&self, f_a: &[f32], f_b: &[f32]) -> Result<(f32, f32), String> {
        if let Some(ref session_lock) = self.推理会话 {
            // A. 将两个 64 维描述子拼接成 128 维特征张量 [cite: 1.2.2]
            let mut concat_vec = Vec::with_capacity(128);
            concat_vec.extend_from_slice(f_a);
            concat_vec.extend_from_slice(f_b);

            // B. 极速组装一维连续阵列 [cite: 1.1.4]
            let 维度 = ([1usize, 128], concat_vec);
            let 输入值 = Value::from_array(维度).map_err(|e| e.to_string())?;
            let 采样输入 = ort::inputs![输入值];

            // C. 执行推理并释放锁
            let mut 会话锁 = session_lock.lock().map_err(|e| e.to_string())?;
            let 输出 = 会话锁.run(采样输入).map_err(|e| e.to_string())?;

            // D. 提取 8x8 Logits 分布图 (1, 64) [cite: 1.2.2]
            let (_shape, logits_data) = 输出[0].try_extract_tensor::<f32>().map_err(|e| e.to_string())?;

            // E. Argmax 寻优：寻找概率最高的目标像素格 [cite: 1.2.2]
            let mut max_idx = 0;
            let mut max_val = -f32::INFINITY;
            for i in 0..64 {
                if logits_data[i] > max_val {
                    max_val = logits_data[i];
                    max_idx = i;
                }
            }

            // F. 逆向重塑为原始分辨率偏移：将 1D 线性索引还原为 8x8 网格下的 (x, y) 相对偏移量 [cite: 1.2.2]
            let grid_y = (max_idx / 8) as f32; // [0..7]
            let grid_x = (max_idx % 8) as f32; // [0..7]

            // 归一化偏差限制在 [-0.5, 0.5] 个粗网格单位，并恢复为原图相对像素偏移
            let dx = grid_x - 3.5;
            let dy = grid_y - 3.5;

            return Ok((dx, dy));
        }

        Err("视觉显微镜处于数学自愈备用模式".to_string())
    }

    /// 🛡️ [自愈备用通道 - 数学级二次梯度插值算子]：通过匹配点周边描述子场梯度二次抛物线拟合解算亚像素极值
    pub fn 插值亚像素偏移(
        &self,
        f_a: &[f32],
        desc_tensor2: &[f32],
        w_d8: usize,
        h_d8: usize,
        x2: f32, // 图像 2 中已匹配点的原图 X 坐标
        y2: f32, // 图像 2 中已匹配点的原图 Y 坐标
    ) -> (f32, f32) {
        let width_scale = (w_d8 as f32) / 639.0;
        let height_scale = (h_d8 as f32) / 639.0;

        // 1. 将原图坐标投射回 1/8 尺寸的特征图坐标系中 [cite: 1.2.2]
        let x_coarse = x2 * width_scale - 0.5;
        let y_coarse = y2 * height_scale - 0.5;

        // 2. 探针采样：在特征图上采样 5 个极小值相邻梯度点的描述子向量 (中心、上下左右各偏移 1/8 个网格单位) [cite: 1.2.2]
        let step = 0.125; // 相当于原图 1 像素的步长

        let f_0 = self.插值描述子_双线性(desc_tensor2, w_d8, h_d8, x_coarse, y_coarse);
        let f_x_minus = self.插值描述子_双线性(desc_tensor2, w_d8, h_d8, x_coarse - step, y_coarse);
        let f_x_plus = self.插值描述子_双线性(desc_tensor2, w_d8, h_d8, x_coarse + step, y_coarse);
        let f_y_minus = self.插值描述子_双线性(desc_tensor2, w_d8, h_d8, x_coarse, y_coarse - step);
        let f_y_plus = self.插值描述子_双线性(desc_tensor2, w_d8, h_d8, x_coarse, y_coarse + step);

        // 3. 计算各个邻域的余弦距离相似度（由于描述子已 L2 归一化，点积即为余弦值）
        let s_0 = self.余弦相似度(f_a, &f_0);
        let s_x_minus = self.余弦相似度(f_a, &f_x_minus);
        let s_x_plus = self.余弦相似度(f_a, &f_x_plus);
        let s_y_minus = self.余弦相似度(f_a, &f_y_minus);
        let s_y_plus = self.余弦相似度(f_a, &f_y_plus);

        // 4. 二次抛物线泰勒拟合：解算亚像素级实对称二次型微分零点，获得局部连续场的极大值顶点 [cite: 1.2.5]
        let denom_x = s_x_minus - 2.0 * s_0 + s_x_plus;
        let dx = if denom_x.abs() > 1e-5 {
            (s_x_minus - s_x_plus) / (2.0 * denom_x)
        } else {
            0.0
        };

        let denom_y = s_y_minus - 2.0 * s_0 + s_y_plus;
        let dy = if denom_y.abs() > 1e-5 {
            (s_y_minus - s_y_plus) / (2.0 * denom_y)
        } else {
            0.0
        };

        // 限制在 $\pm 3.5$ 像素的 Nms 容忍半径内，防止偶发发散
        (dx.clamp(-3.5, 3.5), dy.clamp(-3.5, 3.5))
    }

    /// 高吞吐双线性描述子快速插值算子
    fn 插值描述子_双线性(&self, desc_tensor: &[f32], w_d8: usize, h_d8: usize, x: f32, y: f32) -> [f32; 64] {
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        let dx = x - x0 as f32;
        let dy = y - y0 as f32;

        let w00 = (1.0 - dx) * (1.0 - dy);
        let w10 = dx * (1.0 - dy);
        let w01 = (1.0 - dx) * dy;
        let w11 = dx * dy;

        let get_ptr = |y_idx: i32, x_idx: i32| -> usize {
            let cy = y_idx.clamp(0, h_d8 as i32 - 1) as usize;
            let cx = x_idx.clamp(0, w_d8 as i32 - 1) as usize;
            (cy * w_d8 + cx) * 64
        };

        let ptr00 = get_ptr(y0, x0);
        let ptr10 = get_ptr(y0, x1);
        let ptr01 = get_ptr(y1, x0);
        let ptr11 = get_ptr(y1, x1);

        let mut out = [0.0f32; 64];
        let mut sum_sq = 0.0f32;
        for i in 0..64 {
            let val = w00 * desc_tensor[ptr00 + i] +
                      w10 * desc_tensor[ptr10 + i] +
                      w01 * desc_tensor[ptr01 + i] +
                      w11 * desc_tensor[ptr11 + i];
            out[i] = val;
            sum_sq += val * val;
        }

        let inv_norm = 1.0 / (sum_sq.sqrt().max(1e-12));
        for i in 0..64 {
            out[i] *= inv_norm;
        }
        out
    }

    #[inline]
    fn 余弦相似度(&self, v1: &[f32], v2: &[f32]) -> f32 {
        let mut dot = 0.0f32;
        for i in 0..64 {
            dot += v1[i] * v2[i];
        }
        dot
    }
}

/// 🛡️ 特征匹配与几何说谎者过滤核心组件
pub struct 仿生匹配器;

impl 仿生匹配器 {
    /// 利用双向余弦距离交叉检验 (Mutual Nearest Neighbor) 进行特征向量极速硬核比对
    pub fn 交叉匹配(
        实时特征: &[稀疏特征点],
        历史快照: &[稀疏特征点],
        最小相似度阈值: f32,
    ) -> Vec<(usize, usize, f32)> {
        if 实时特征.is_empty() || 历史快照.is_empty() { return Vec::new(); }

        let rows1 = 实时特征.len();
        let rows2 = 历史快照.len();
        
        let mut 匹配12 = vec![-1i32; rows1];
        let mut 得分12 = vec![0.0f32; rows1];

        for i in 0..rows1 {
            let mut max_score = -1.0f32;
            let mut max_idx = -1i32;
            for j in 0..rows2 {
                let 相似度 = Self::余弦相似度(&实时特征[i].描述子, &历史快照[j].描述子);
                if 相似度 > max_score {
                    max_score = 相似度;
                    max_idx = j as i32;
                }
            }
            匹配12[i] = max_idx;
            得分12[i] = max_score;
        }

        let mut 匹配21 = vec![-1i32; rows2];
        for j in 0..rows2 {
            let mut max_score = -1.0f32;
            let mut max_idx = -1i32;
            for i in 0..rows1 {
                let 相似度 = Self::余弦相似度(&实时特征[i].描述子, &历史快照[j].描述子);
                if 相似度 > max_score {
                    max_score = 相似度;
                    max_idx = i as i32;
                }
            }
            匹配21[j] = max_idx;
        }

        let mut 最终匹配对 = Vec::new();
        for i in 0..rows1 {
            let j = 匹配12[i];
            if j >= 0 {
                let j_idx = j as usize;
                if 匹配21[j_idx] == i as i32 && 得分12[i] > 最小相似度阈值 {
                    最终匹配对.push((i, j_idx, 得分12[i]));
                }
            }
        }

        最终匹配对
    }

    /// RANSAC 对极几何外点过滤（彻底滤除动态噪点和假阴影点）
    pub fn 几何纠偏过滤(
        实时特征: &[稀疏特征点],
        历史快照: &[稀疏特征点],
        匹配对: &[(usize, usize, f32)],
        ransac_误差阈值: f64,
    ) -> Result<Vec<(usize, usize, f32)>, String> {
        if 匹配对.len() < 8 {
            return Err("❌ 匹配对数量少于 8 对，物理约束不足，无法进行纠偏计算！".to_string());
        }

        let mut 投影点_实时 = core::Vector::<Point2f>::new();
        let mut 投影点_历史 = core::Vector::<Point2f>::new();

        for &(idx1, idx2, _) in 匹配对 {
            let pt1 = &实时特征[idx1];
            let pt2 = &历史快照[idx2];
            投影点_实时.push(Point2f::new(pt1.x, pt1.y));
            投影点_历史.push(Point2f::new(pt2.x, pt2.y));
        }

        let mut 状态掩码 = Mat::default();
        
        calib3d::find_fundamental_mat(
            &投影点_实时,
            &投影点_历史,
            calib3d::FM_RANSAC,
            ransac_误差阈值,
            0.999,
            1000,
            &mut 状态掩码
        ).map_err(|e| e.to_string())?;

        let mut 干净的匹配对 = Vec::new();
        for i in 0..匹配对.len() {
            let 判决 = *状态掩码.at::<u8>(i as i32).map_err(|e| e.to_string())?;
            if 判决 != 0 {
                干净的匹配对.push(匹配对[i]);
            }
        }

        Ok(干净的匹配对)
    }

    #[inline]
    fn 余弦相似度(v1: &[f32], v2: &[f32]) -> f32 {
        let mut 点积 = 0.0f32;
        for i in 0..64 {
            点积 += v1[i] * v2[i];
        }
        点积
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parabolic_sub_pixel_interpolation_math() {
        println!("🛡️ [验证探针] 启动数学级二次梯度自愈插值算子测试...");

        let microscope = 仿生亚像素显微镜::new();
        
        // 设 w_d8 = 8, h_d8 = 8. 总描述子通道空间 = 8 * 8 * 64
        let w_d8 = 8;
        let h_d8 = 8;
        let mut desc_tensor = vec![0.0f32; w_d8 * h_d8 * 64];

        // 设中心坐标为 (x=4, y=4) 的网格单元
        let cx = 4;
        let cy = 4;

        // 设参考描述子 f_a 只有第一维为 1.0 (标准 L2 归一化向量)
        let mut f_a = [0.0f32; 64];
        f_a[0] = 1.0;

        let get_offset = |gx: usize, gy: usize| -> usize {
            (gy * w_d8 + gx) * 64
        };

        // 注入正规化的描述子，点积值即为期望的余弦相似度
        let set_normalized_desc = |tensor: &mut [f32], gx: usize, gy: usize, sim: f32| {
            let offset = get_offset(gx, gy);
            tensor[offset] = sim;
            let remaining = (1.0 - sim * sim).sqrt();
            tensor[offset + 1] = remaining; // 勾股定理补充，保证 L2 范数绝对为 1.0
        };

        // 注入测试梯度值（基于中心点向四周扩散的相似度梯度）
        // s_0 (中心匹配点) = 0.9
        set_normalized_desc(&mut desc_tensor, cx, cy, 0.9);
        // s_x_minus (左侧相邻点) = 0.7
        set_normalized_desc(&mut desc_tensor, cx - 1, cy, 0.7);
        // s_x_plus (右侧相邻点) = 0.8
        set_normalized_desc(&mut desc_tensor, cx + 1, cy, 0.8);
        // s_y_minus (下方相邻点) = 0.6
        set_normalized_desc(&mut desc_tensor, cx, cy - 1, 0.6);
        // s_y_plus (上方相邻点) = 0.85
        set_normalized_desc(&mut desc_tensor, cx, cy + 1, 0.85);

        // 模拟图像 2 中的像素中心。根据 1/8 缩放关系逆推：
        let width_scale = (w_d8 as f32) / 639.0;
        let height_scale = (h_d8 as f32) / 639.0;
        let x2 = (cx as f32 + 0.5) / width_scale;
        let y2 = (cy as f32 + 0.5) / height_scale;

        // 执行抛物线自愈拟合
        let (dx, dy) = microscope.插值亚像素偏移(&f_a, &desc_tensor, w_d8, h_d8, x2, y2);

        println!("📊 [验证探针] 解算出的亚像素相对偏移量: dx = {:.6}, dy = {:.6}", dx, dy);


        let dx_expected = 0.143602;
        let dy_expected = 0.324049;

        // 严格断言精度，必须满足 1e-4 亚毫米级偏差以内
        assert!((dx - dx_expected).abs() < 1e-4, "❌ X 轴抛物线极值解算偏离理论值！实际: {}, 期望: {}", dx, dx_expected);
        assert!((dy - dy_expected).abs() < 1e-4, "❌ Y 轴抛物线极值解算偏离理论值！实际: {}, 期望: {}", dy, dy_expected);

        println!("🏆 [验证结论] 阶段三数学自愈插值算子通过物理验证！精度完美对齐泰勒级数微分零点！");
    }
}