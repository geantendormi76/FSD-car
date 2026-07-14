use super::{ipm_projector::CameraCalibration, xfeat_engine::稀疏特征点};
use opencv::{
    calib3d,
    core::{self, Mat, Point2f},
    prelude::*,
};
use ort::{
    session::{builder::GraphOptimizationLevel, Session},
    value::Value,
};
use std::sync::Mutex;

const DESCRIPTOR_DIM: usize = 64;

pub struct 仿生亚像素显微镜 {
    推理会话: Option<Mutex<Session>>,
}

impl 仿生亚像素显微镜 {
    pub fn new() -> Self {
        let model_path = "model/refinement_mlp.onnx";
        if !std::path::Path::new(model_path).exists() {
            return Self { 推理会话: None };
        }
        let session = Session::builder()
            .and_then(|builder| {
                builder.with_execution_providers([
                    ort::ep::CUDA::default().build(),
                    ort::ep::CPU::default().build(),
                ])
            })
            .and_then(|builder| builder.with_optimization_level(GraphOptimizationLevel::Level3))
            .and_then(|builder| builder.with_intra_threads(1))
            .and_then(|builder| builder.commit_from_file(model_path));
        Self {
            推理会话: session.ok().map(Mutex::new),
        }
    }

    pub fn 预测亚像素偏移(&self, f_a: &[f32], f_b: &[f32]) -> Result<(f32, f32), String> {
        validate_descriptor(f_a)?;
        validate_descriptor(f_b)?;
        let session = self
            .推理会话
            .as_ref()
            .ok_or_else(|| "亚像素 MLP 权重不可用".to_string())?;
        let mut concatenated = Vec::with_capacity(DESCRIPTOR_DIM * 2);
        concatenated.extend_from_slice(f_a);
        concatenated.extend_from_slice(f_b);
        let input = Value::from_array(([1usize, DESCRIPTOR_DIM * 2], concatenated))
            .map_err(|e| e.to_string())?;
        let mut session = session.lock().map_err(|e| e.to_string())?;
        let outputs = session
            .run(ort::inputs![input])
            .map_err(|e| e.to_string())?;
        if outputs.len() != 1 {
            return Err(format!("亚像素 MLP 期望一个输出，实际 {}", outputs.len()));
        }
        let output = outputs
            .values()
            .next()
            .ok_or_else(|| "亚像素 MLP 输出为空".to_string())?;
        let (shape, logits) = output
            .try_extract_tensor::<f32>()
            .map_err(|e| e.to_string())?;
        if shape.num_elements() != 64 || logits.len() != 64 || logits.iter().any(|v| !v.is_finite())
        {
            return Err(format!("亚像素 MLP 输出契约错误：shape={shape:?}"));
        }
        let best = logits
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.total_cmp(b.1))
            .map(|(index, _)| index)
            .ok_or_else(|| "亚像素 MLP 输出为空".to_string())?;
        Ok(((best % 8) as f32 - 3.5, (best / 8) as f32 - 3.5))
    }

    /// 在 HWC 64 维稠密描述子图上进行局部抛物线细化，输出原图像素偏移。
    pub fn 插值亚像素偏移(
        &self,
        f_a: &[f32],
        desc_tensor2: &[f32],
        w_d8: usize,
        h_d8: usize,
        x2: f32,
        y2: f32,
    ) -> Result<(f32, f32), String> {
        validate_descriptor(f_a)?;
        if w_d8 < 2
            || h_d8 < 2
            || desc_tensor2.len() != w_d8 * h_d8 * DESCRIPTOR_DIM
            || desc_tensor2.iter().any(|v| !v.is_finite())
            || !x2.is_finite()
            || !y2.is_finite()
        {
            return Err("稠密描述子或图像坐标契约无效".to_string());
        }
        let image_width = w_d8 * 8;
        let image_height = h_d8 * 8;
        let x_coarse = x2 * w_d8 as f32 / (image_width - 1) as f32 - 0.5;
        let y_coarse = y2 * h_d8 as f32 / (image_height - 1) as f32 - 0.5;
        let step = 0.125f32;
        let score = |x: f32, y: f32| {
            let sampled = interpolate_descriptor(desc_tensor2, w_d8, h_d8, x, y);
            cosine_similarity(f_a, &sampled)
        };
        let center = score(x_coarse, y_coarse);
        let x_minus = score(x_coarse - step, y_coarse);
        let x_plus = score(x_coarse + step, y_coarse);
        let y_minus = score(x_coarse, y_coarse - step);
        let y_plus = score(x_coarse, y_coarse + step);
        let refine = |minus: f32, center: f32, plus: f32| {
            let denominator = minus - 2.0 * center + plus;
            if denominator.abs() > 1e-6 {
                ((minus - plus) / (2.0 * denominator)).clamp(-1.0, 1.0)
            } else {
                0.0
            }
        };
        Ok((
            refine(x_minus, center, x_plus),
            refine(y_minus, center, y_plus),
        ))
    }
}

impl Default for 仿生亚像素显微镜 {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub struct 单应性估计 {
    /// 从实时图像像素映射到历史图像像素的 3x3 单应矩阵。
    pub 实时到历史: [[f64; 3]; 3],
    pub 内点匹配: Vec<(usize, usize, f32)>,
}

#[derive(Debug, Clone, Copy)]
pub struct 平面度量变换 {
    /// 将实时相机地面坐标变换到历史相机地面坐标。
    pub 向前平移米: f32,
    pub 向左平移米: f32,
    pub 偏航弧度: f32,
    pub 内点数: usize,
}

pub struct 仿生匹配器;

impl 仿生匹配器 {
    pub fn 交叉匹配(
        实时特征: &[稀疏特征点],
        历史快照: &[稀疏特征点],
        最小相似度阈值: f32,
    ) -> Vec<(usize, usize, f32)> {
        if !最小相似度阈值.is_finite() {
            return Vec::new();
        }
        let valid_realtime: Vec<_> = 实时特征
            .iter()
            .enumerate()
            .filter(|(_, feature)| valid_feature(feature))
            .collect();
        let valid_history: Vec<_> = 历史快照
            .iter()
            .enumerate()
            .filter(|(_, feature)| valid_feature(feature))
            .collect();
        if valid_realtime.is_empty() || valid_history.is_empty() {
            return Vec::new();
        }

        let forward: Vec<_> = valid_realtime
            .iter()
            .map(|(_, current)| {
                valid_history
                    .iter()
                    .enumerate()
                    .map(|(position, (_, history))| {
                        (
                            position,
                            cosine_similarity(&current.描述子, &history.描述子),
                        )
                    })
                    .max_by(|a, b| a.1.total_cmp(&b.1))
                    .unwrap_or((usize::MAX, f32::NEG_INFINITY))
            })
            .collect();
        let backward: Vec<_> = valid_history
            .iter()
            .map(|(_, history)| {
                valid_realtime
                    .iter()
                    .enumerate()
                    .map(|(position, (_, current))| {
                        (
                            position,
                            cosine_similarity(&current.描述子, &history.描述子),
                        )
                    })
                    .max_by(|a, b| a.1.total_cmp(&b.1))
                    .map_or(usize::MAX, |best| best.0)
            })
            .collect();

        forward
            .into_iter()
            .enumerate()
            .filter_map(|(current_position, (history_position, score))| {
                (history_position < backward.len()
                    && backward[history_position] == current_position
                    && score >= 最小相似度阈值)
                    .then_some((
                        valid_realtime[current_position].0,
                        valid_history[history_position].0,
                        score,
                    ))
            })
            .collect()
    }

    pub fn 几何纠偏过滤(
        实时特征: &[稀疏特征点],
        历史快照: &[稀疏特征点],
        匹配对: &[(usize, usize, f32)],
        ransac_误差阈值: f64,
    ) -> Result<Vec<(usize, usize, f32)>, String> {
        if 匹配对.len() < 8 {
            return Err("基础矩阵 RANSAC 至少需要 8 对匹配".to_string());
        }
        let (realtime, history) = matched_points(实时特征, 历史快照, 匹配对)?;
        let mut mask = Mat::default();
        let fundamental = calib3d::find_fundamental_mat(
            &realtime,
            &history,
            calib3d::FM_RANSAC,
            valid_threshold(ransac_误差阈值)?,
            0.999,
            2000,
            &mut mask,
        )
        .map_err(|e| e.to_string())?;
        if fundamental.empty() || mask.total() != 匹配对.len() {
            return Err("基础矩阵估计退化或内点掩码无效".to_string());
        }
        filter_by_mask(匹配对, &mask)
    }

    pub fn 估计单应性(
        实时特征: &[稀疏特征点],
        历史快照: &[稀疏特征点],
        匹配对: &[(usize, usize, f32)],
        ransac_误差阈值: f64,
    ) -> Result<单应性估计, String> {
        if 匹配对.len() < 4 {
            return Err("单应性 RANSAC 至少需要 4 对匹配".to_string());
        }
        let (realtime, history) = matched_points(实时特征, 历史快照, 匹配对)?;
        let mut mask = Mat::default();
        let homography = calib3d::find_homography_ext(
            &realtime,
            &history,
            calib3d::RANSAC,
            valid_threshold(ransac_误差阈值)?,
            &mut mask,
            2000,
            0.999,
        )
        .map_err(|e| e.to_string())?;
        if homography.empty() || homography.rows() != 3 || homography.cols() != 3 {
            return Err("单应性估计退化".to_string());
        }
        let inliers = filter_by_mask(匹配对, &mask)?;
        if inliers.len() < 4 {
            return Err("单应性有效内点少于 4".to_string());
        }
        let mut matrix = [[0.0f64; 3]; 3];
        for (row, matrix_row) in matrix.iter_mut().enumerate() {
            for (col, value) in matrix_row.iter_mut().enumerate() {
                *value = *homography
                    .at_2d::<f64>(row as i32, col as i32)
                    .map_err(|e| e.to_string())?;
            }
        }
        Ok(单应性估计 {
            实时到历史: matrix,
            内点匹配: inliers,
        })
    }

    /// 仅适用于已知落在地平面上的特征。非地面特征必须使用完整位姿估计。
    pub fn 估计地面度量变换(
        实时特征: &[稀疏特征点],
        历史快照: &[稀疏特征点],
        匹配对: &[(usize, usize, f32)],
        calibration: CameraCalibration,
        ransac_误差阈值: f64,
    ) -> Result<平面度量变换, String> {
        let estimate = Self::估计单应性(实时特征, 历史快照, 匹配对, ransac_误差阈值)?;
        let metric_pairs: Vec<_> = estimate
            .内点匹配
            .iter()
            .filter_map(|(current, history, _)| {
                let current = &实时特征[*current];
                let history = &历史快照[*history];
                Some((
                    calibration.pixel_to_ground(current.x, current.y)?,
                    calibration.pixel_to_ground(history.x, history.y)?,
                ))
            })
            .collect();
        if metric_pairs.len() < 3 {
            return Err("地平面有效度量匹配少于 3 对".to_string());
        }
        let count = metric_pairs.len() as f32;
        let current_center = metric_pairs.iter().fold((0.0, 0.0), |sum, pair| {
            (sum.0 + pair.0 .0, sum.1 + pair.0 .1)
        });
        let history_center = metric_pairs.iter().fold((0.0, 0.0), |sum, pair| {
            (sum.0 + pair.1 .0, sum.1 + pair.1 .1)
        });
        let current_center = (current_center.0 / count, current_center.1 / count);
        let history_center = (history_center.0 / count, history_center.1 / count);
        let (dot, cross) = metric_pairs.iter().fold((0.0, 0.0), |sum, pair| {
            let current = (pair.0 .0 - current_center.0, pair.0 .1 - current_center.1);
            let history = (pair.1 .0 - history_center.0, pair.1 .1 - history_center.1);
            (
                sum.0 + current.0 * history.0 + current.1 * history.1,
                sum.1 + current.0 * history.1 - current.1 * history.0,
            )
        });
        if dot.abs() + cross.abs() <= 1e-8 {
            return Err("地面度量匹配几何退化".to_string());
        }
        let yaw = cross.atan2(dot);
        let (sin_yaw, cos_yaw) = yaw.sin_cos();
        let rotated_center = (
            cos_yaw * current_center.0 - sin_yaw * current_center.1,
            sin_yaw * current_center.0 + cos_yaw * current_center.1,
        );
        Ok(平面度量变换 {
            向前平移米: history_center.0 - rotated_center.0,
            向左平移米: history_center.1 - rotated_center.1,
            偏航弧度: yaw,
            内点数: metric_pairs.len(),
        })
    }
}

fn valid_feature(feature: &稀疏特征点) -> bool {
    feature.x.is_finite()
        && feature.y.is_finite()
        && feature.置信度.is_finite()
        && validate_descriptor(&feature.描述子).is_ok()
}

fn validate_descriptor(descriptor: &[f32]) -> Result<(), String> {
    if descriptor.len() != DESCRIPTOR_DIM || descriptor.iter().any(|value| !value.is_finite()) {
        return Err("XFeat 描述子必须是 64 个有限浮点数".to_string());
    }
    Ok(())
}

fn valid_threshold(threshold: f64) -> Result<f64, String> {
    if threshold.is_finite() && threshold > 0.0 {
        Ok(threshold)
    } else {
        Err("RANSAC 阈值必须是有限正数".to_string())
    }
}

fn matched_points(
    realtime: &[稀疏特征点],
    history: &[稀疏特征点],
    matches: &[(usize, usize, f32)],
) -> Result<(core::Vector<Point2f>, core::Vector<Point2f>), String> {
    let mut realtime_points = core::Vector::new();
    let mut history_points = core::Vector::new();
    for &(current_index, history_index, score) in matches {
        let current = realtime
            .get(current_index)
            .ok_or_else(|| format!("实时特征索引越界：{current_index}"))?;
        let historical = history
            .get(history_index)
            .ok_or_else(|| format!("历史特征索引越界：{history_index}"))?;
        if !valid_feature(current) || !valid_feature(historical) || !score.is_finite() {
            return Err("匹配对包含无效特征或得分".to_string());
        }
        realtime_points.push(Point2f::new(current.x, current.y));
        history_points.push(Point2f::new(historical.x, historical.y));
    }
    Ok((realtime_points, history_points))
}

fn filter_by_mask(
    matches: &[(usize, usize, f32)],
    mask: &Mat,
) -> Result<Vec<(usize, usize, f32)>, String> {
    if mask.total() != matches.len() {
        return Err("RANSAC 内点掩码长度不匹配".to_string());
    }
    let mut inliers = Vec::new();
    for (index, item) in matches.iter().enumerate() {
        if *mask.at::<u8>(index as i32).map_err(|e| e.to_string())? != 0 {
            inliers.push(*item);
        }
    }
    Ok(inliers)
}

fn interpolate_descriptor(
    tensor: &[f32],
    width: usize,
    height: usize,
    x: f32,
    y: f32,
) -> [f32; DESCRIPTOR_DIM] {
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let dx = x - x0 as f32;
    let dy = y - y0 as f32;
    let offset = |sample_x: i32, sample_y: i32| {
        let sample_x = sample_x.clamp(0, width as i32 - 1) as usize;
        let sample_y = sample_y.clamp(0, height as i32 - 1) as usize;
        (sample_y * width + sample_x) * DESCRIPTOR_DIM
    };
    let offsets = [
        offset(x0, y0),
        offset(x0 + 1, y0),
        offset(x0, y0 + 1),
        offset(x0 + 1, y0 + 1),
    ];
    let weights = [
        (1.0 - dx) * (1.0 - dy),
        dx * (1.0 - dy),
        (1.0 - dx) * dy,
        dx * dy,
    ];
    let mut result = [0.0f32; DESCRIPTOR_DIM];
    let mut norm_sq = 0.0f32;
    for channel in 0..DESCRIPTOR_DIM {
        result[channel] = offsets
            .iter()
            .zip(weights)
            .map(|(offset, weight)| tensor[*offset + channel] * weight)
            .sum();
        norm_sq += result[channel] * result[channel];
    }
    let inverse_norm = norm_sq.sqrt().max(1e-12).recip();
    for value in &mut result {
        *value *= inverse_norm;
    }
    result
}

fn cosine_similarity(left: &[f32], right: &[f32]) -> f32 {
    let (dot, left_norm_sq, right_norm_sq) =
        left.iter()
            .zip(right)
            .fold((0.0f32, 0.0f32, 0.0f32), |sum, (left, right)| {
                (
                    sum.0 + left * right,
                    sum.1 + left * left,
                    sum.2 + right * right,
                )
            });
    let norm_product = (left_norm_sq * right_norm_sq).sqrt();
    if !norm_product.is_finite() || norm_product <= 1e-12 {
        f32::NEG_INFINITY
    } else {
        (dot / norm_product).clamp(-1.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feature(x: f32, y: f32, descriptor_index: usize) -> 稀疏特征点 {
        let mut descriptor = vec![0.0; 64];
        descriptor[descriptor_index] = 1.0;
        稀疏特征点 {
            x,
            y,
            置信度: 1.0,
            描述子: descriptor,
        }
    }

    #[test]
    fn invalid_descriptors_are_not_matched() {
        let mut invalid = feature(0.0, 0.0, 0);
        invalid.描述子.pop();
        assert!(仿生匹配器::交叉匹配(&[invalid], &[feature(0.0, 0.0, 0)], 0.5).is_empty());
    }

    #[test]
    fn matching_uses_cosine_similarity_for_scaled_descriptors() {
        let mut realtime = feature(0.0, 0.0, 0);
        let mut history = feature(1.0, 1.0, 0);
        realtime.描述子[0] = 2.0;
        history.描述子[0] = 3.0;

        let matches = 仿生匹配器::交叉匹配(&[realtime], &[history], 0.99);
        assert_eq!(matches.len(), 1);
        assert!((matches[0].2 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn homography_estimation_returns_inliers() {
        let realtime = vec![
            feature(10.0, 10.0, 0),
            feature(100.0, 10.0, 1),
            feature(10.0, 100.0, 2),
            feature(100.0, 100.0, 3),
            feature(55.0, 55.0, 4),
        ];
        let history: Vec<_> = realtime
            .iter()
            .enumerate()
            .map(|(index, point)| feature(point.x + 4.0, point.y - 2.0, index))
            .collect();
        let matches: Vec<_> = (0..realtime.len())
            .map(|index| (index, index, 1.0))
            .collect();
        let estimate = 仿生匹配器::估计单应性(&realtime, &history, &matches, 1.0).unwrap();
        assert_eq!(estimate.内点匹配.len(), 5);
        assert!((estimate.实时到历史[0][2] - 4.0).abs() < 1e-3);
        assert!((estimate.实时到历史[1][2] + 2.0).abs() < 1e-3);
    }

    #[test]
    fn ground_matches_produce_metric_translation() {
        let calibration = CameraCalibration {
            image_width: 640,
            image_height: 480,
            fx: 204.25533,
            fy: 153.1915,
            cx: 319.5,
            cy: 239.5,
            forward_offset_m: 0.069,
            left_offset_m: 0.0,
            height_m: 0.133,
            yaw_rad: 0.0,
            pitch_rad: 0.169,
            roll_rad: 0.0,
        };
        let ground_points = [
            (0.8, -0.2),
            (0.8, 0.2),
            (1.2, -0.3),
            (1.2, 0.3),
            (1.8, -0.4),
            (1.8, 0.4),
        ];
        let realtime: Vec<_> = ground_points
            .iter()
            .enumerate()
            .map(|(index, point)| {
                let pixel = calibration.ground_to_pixel(point.0, point.1).unwrap();
                feature(pixel.0, pixel.1, index)
            })
            .collect();
        let history: Vec<_> = ground_points
            .iter()
            .enumerate()
            .map(|(index, point)| {
                let pixel = calibration
                    .ground_to_pixel(point.0 + 0.10, point.1 + 0.05)
                    .unwrap();
                feature(pixel.0, pixel.1, index)
            })
            .collect();
        let matches: Vec<_> = (0..ground_points.len())
            .map(|index| (index, index, 1.0))
            .collect();
        let transform = 仿生匹配器::估计地面度量变换(
            &realtime,
            &history,
            &matches,
            calibration,
            1.0,
        )
        .unwrap();
        assert!((transform.向前平移米 - 0.10).abs() < 1e-3);
        assert!((transform.向左平移米 - 0.05).abs() < 1e-3);
        assert!(transform.偏航弧度.abs() < 1e-3);
    }

    #[test]
    fn subpixel_fallback_rejects_wrong_tensor() {
        let microscope = 仿生亚像素显微镜::new();
        assert!(microscope
            .插值亚像素偏移(&[0.0; 64], &[0.0; 63], 1, 1, 0.0, 0.0)
            .is_err());
    }
}
