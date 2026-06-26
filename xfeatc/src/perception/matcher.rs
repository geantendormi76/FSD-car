use super::xfeat_engine::稀疏特征点;
use opencv::{
    prelude::*,
    core::{self, Point2f},
    calib3d,
};

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

    /// RANSAC 对极几何外点过滤（彻底滤除动态噪点和假阴影点） (修复 7 参数接口)
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
        
        // 核心纠偏：传入 7 个完整参数，max_iters 设定为 1000 次最大迭代
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

    fn 余弦相似度(v1: &[f32], v2: &[f32]) -> f32 {
        let mut 点积 = 0.0f32;
        for i in 0..64 {
            点积 += v1[i] * v2[i];
        }
        点积
    }
}