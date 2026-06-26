use opencv::{
    prelude::*,
    core::{self, Mat, Point, Size, Scalar},
    imgproc,
};

/// 🛡️ 领域模型：2D 动态势场 (常作为通信载荷，高度优化为粗糙网格以节省带宽)
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct 动态势场 {
    /// 势场网格 (例如 20x20 的局部避障栅格，数值越大代表排斥力越大)
    pub 斥力网格: Vec<Vec<f32>>, 
    /// 视野内的最大危险指数 (0.0 表示绝对安全，1.0 表示即将碰撞)
    pub 最高危险等级: f32,
    /// 避障推荐逃逸向量 (x_force, y_force)
    pub 逃逸方向: (f32, f32),
}

/// 🛡️ 仿生视觉核心契约
pub trait 伪青蛙眼感知器: Send {
    /// 初始化感知器内部的内存金库，避免运行时 malloc 造成的延迟抖动
    fn 初始化(&mut self, 宽: i32, 高: i32) -> Result<(), String>;

    /// 核心处理：输入最新图像帧与 IMU 偏航角增量（弧度），输出计算好的势场
    /// * `角速度_yaw_delta`: 过去一帧时间内的偏航角变化，用于策略 A 的运动补偿
    fn 处理图像帧(&mut self, 原始帧: &Mat, 角速度_yaw_delta: f32) -> Result<动态势场, String>;

    /// 获取渲染后的调试画面 (叠加了红色势场热力图)
    fn 获取调试帧(&self) -> Result<Mat, String>;
}

/// 🛡️ 仿生青蛙眼核心实现结构体
pub struct 仿生青蛙眼 {
    宽: i32,
    高: i32,
    /// 抑制感受野 (IRF)：存储上一帧的灰度背景
    上一帧灰度: Mat,
    /// 兴奋感受野 (ERF)：当前帧的灰度
    当前帧灰度: Mat,
    /// 帧间差值缓存
    差异帧: Mat,
    /// 二值化运动斑块
    运动斑块: Mat,
    /// 势场膨胀热力图
    势场图: Mat,
    /// 策略 A 仿射变换补偿矩阵
    运动补偿矩阵: Mat,
}

impl 仿生青蛙眼 {
    /// 构造全新的空壳结构体，不分配堆内存
    pub fn new() -> Self {
        Self {
            宽: 0,
            高: 0,
            上一帧灰度: Mat::default(),
            当前帧灰度: Mat::default(),
            差异帧: Mat::default(),
            运动斑块: Mat::default(),
            势场图: Mat::default(),
            运动补偿矩阵: Mat::default(),
        }
    }
}

impl 伪青蛙眼感知器 for 仿生青蛙眼 {
    fn 初始化(&mut self, 宽: i32, 高: i32) -> Result<(), String> {
        self.宽 = 宽;
        self.高 = 高;
        
        // 预分配所有内存金库，防止运行时动态分配内存（采用安全可靠的 new_rows_cols_with_default 机制）
        self.上一帧灰度 = Mat::new_rows_cols_with_default(高, 宽, core::CV_8UC1, Scalar::all(0.0)).map_err(|e| e.to_string())?;
        self.当前帧灰度 = Mat::new_rows_cols_with_default(高, 宽, core::CV_8UC1, Scalar::all(0.0)).map_err(|e| e.to_string())?;
        self.差异帧 = Mat::new_rows_cols_with_default(高, 宽, core::CV_8UC1, Scalar::all(0.0)).map_err(|e| e.to_string())?;
        self.运动斑块 = Mat::new_rows_cols_with_default(高, 宽, core::CV_8UC1, Scalar::all(0.0)).map_err(|e| e.to_string())?;
        self.势场图 = Mat::new_rows_cols_with_default(高, 宽, core::CV_8UC1, Scalar::all(0.0)).map_err(|e| e.to_string())?;
        
        // 使用 from_slice_2d 一步到位预分配 2x3 仿射变换矩阵，避免 reshape 产生 BoxedRef 引用生命周期问题
        let 补偿数据: &[&[f32]] = &[
            &[1.0, 0.0, 0.0],
            &[0.0, 1.0, 0.0],
        ];
        self.运动补偿矩阵 = Mat::from_slice_2d(补偿数据).map_err(|e| e.to_string())?;

        Ok(())
    }

    fn 处理图像帧(&mut self, 原始帧: &Mat, 角速度_yaw_delta: f32) -> Result<动态势场, String> {
        if self.宽 == 0 || self.高 == 0 {
            return Err("❌ 伪青蛙眼感知器未初始化！".to_string());
        }

        // 1. 转为灰度并存入【当前帧灰度】缓存
        let mut 临时灰度 = Mat::default();
        imgproc::cvt_color(原始帧, &mut 临时灰度, imgproc::COLOR_BGR2GRAY, 0).map_err(|e| e.to_string())?;
        imgproc::gaussian_blur(&临时灰度, &mut self.当前帧灰度, Size::new(21, 21), 0.0, 0.0, core::BORDER_DEFAULT).map_err(|e| e.to_string())?;

        // 检查是否是第一帧输入
        let 元素总数 = self.上一帧灰度.total();
        let mut 是否全零 = true;
        if 元素总数 > 0 {
            // 快速检查上一帧是否全空
            if let Ok(均值) = core::mean(&self.上一帧灰度, &core::no_array()) {
                if 均值[0] > 0.1 { 是否全零 = false; }
            }
        }

        if 是否全零 {
            self.当前帧灰度.copy_to(&mut self.上一帧灰度).map_err(|e| e.to_string())?;
            return Ok(动态势场 { 斥力网格: vec![vec![0.0; 20]; 20], 最高危险等级: 0.0, 逃逸方向: (0.0, 0.0) });
        }

        // 2. 策略 A 运动自愈：基于 IMU 偏航角速度，反向补偿上一帧（抵消自车转弯造成的背景漂移）
        let mut 补偿后的上一帧 = Mat::default();
        if 角速度_yaw_delta.abs() > 0.001 {
            // 基于单目相机焦距算子估算水平像素漂移量 (假设水平焦距为 500 像素)
            let 焦距_f_x = 500.0f32;
            let 水平像素漂移 = 焦距_f_x * 角速度_yaw_delta;

            // 动态注入反向平移偏差到 2x3 仿射矩阵的 X轴偏移量参数
            self.运动补偿矩阵.at_2d_mut::<f32>(0, 2).map(|v| *v = -水平像素漂移).map_err(|e| e.to_string())?;

            // 执行极速仿射变换
            imgproc::warp_affine(
                &self.上一帧灰度,
                &mut 补偿后的上一帧,
                &self.运动补偿矩阵,
                Size::new(self.宽, self.高),
                imgproc::INTER_LINEAR,
                core::BORDER_REPLICATE,
                Scalar::default()
            ).map_err(|e| e.to_string())?;
        } else {
            self.上一帧灰度.copy_to(&mut 补偿后的上一帧).map_err(|e| e.to_string())?;
        }

        // 3. 青蛙眼核心算子：抑制感受野(IRF)与兴奋感受野(ERF)相减，提取纯动态变化
        core::absdiff(&补偿后的上一帧, &self.当前帧灰度, &mut self.差异帧).map_err(|e| e.to_string())?;

        // 4. 提取运动斑块并进行形态学膨胀
        imgproc::threshold(&self.差异帧, &mut self.运动斑块, 25.0, 255.0, imgproc::THRESH_BINARY).map_err(|e| e.to_string())?;
        
        // * 步骤 B：执行形态学膨胀。由于此时 `差异帧` 数据已过时不再需要，我们将其作为输出缓存（Dst）安全复用！
        imgproc::dilate(
            &self.运动斑块,
            &mut self.差异帧,
            &core::no_array(),
            Point::new(-1, -1),
            2,
            core::BORDER_CONSTANT,
            Scalar::default()
        ).map_err(|e| e.to_string())?;

        // 5. 模拟感受野辐射：对膨胀后保存在 `差异帧` 中的斑块进行超强高斯模糊，生成 2D 连续势场
        imgproc::gaussian_blur(&self.差异帧, &mut self.势场图, Size::new(101, 101), 0.0, 0.0, core::BORDER_DEFAULT).map_err(|e| e.to_string())?;

        // 6. 降维计算：将高分辨率势场图压缩为 20x20 的粗糙网格，并解算出逃逸向量
        let 栅格大小 = 20;
        let mut 斥力网格 = vec![vec![0.0f32; 栅格大小]; 栅格大小];
        let 块宽 = self.宽 / 栅格大小 as i32;
        let 块高 = self.高 / 栅格大小 as i32;
        let mut 最大值: u8 = 0;
        
        let mut 斥力中心_x = 0.0f32;
        let mut 斥力中心_y = 0.0f32;
        let mut 总斥力 = 0.0f32;

        for 行 in 0..栅格大小 {
            for 列 in 0..栅格大小 {
                // 截取 20x20 对应的 ROI 区域，原地求均值
                let roi = core::Rect::new(列 as i32 * 块宽, 行 as i32 * 块高, 块宽, 块高);
                
                // 改用链式方法调用风格，触发 Rust 自动解引用 (Deref Coercion)，平滑转换为 col_bounds 可用的形式
                let 行切片 = self.势场图.row_bounds(roi.y, roi.y + roi.height).map_err(|e| e.to_string())?;
                let 局部块 = 行切片.col_bounds(roi.x, roi.x + roi.width).map_err(|e| e.to_string())?;
                
                // 通过 &* 对 BoxedRef 解引用，将底层 Mat 裸引用安全传递给 mean 算子
                if let Ok(均值) = core::mean(&局部块, &core::no_array()) {
                    let 势能值 = (均值[0] / 255.0) as f32;
                    斥力网格[行][列] = 势能值;
                    
                    if 均值[0] as u8 > 最大值 {
                        最大值 = 均值[0] as u8;
                    }

                    // 物理质心累计计算，用于解算逃逸方向
                    if 势能值 > 0.05 {
                        let 相对坐标_x = (列 as f32 / 栅格大小 as f32) - 0.5; // [-0.5, 0.5]
                        let 相对坐标_y = 0.5 - (行 as f32 / 栅格大小 as f32); // [0.5, -0.5]
                        斥力中心_x += 相对坐标_x * 势能值;
                        斥力中心_y += 相对坐标_y * 势能值;
                        总斥力 += 势能值;
                    }
                }
            }
        }

        // 解算逃逸方向 (反向质心向量)
        let 逃逸方向 = if 总斥力 > 0.01 {
            let 质心_x = 斥力中心_x / 总斥力;
            let 质心_y = 斥力中心_y / 总斥力;
            // 逃逸向量与质心方向相反
            (-质心_x, -质心_y)
        } else {
            (0.0, 0.0)
        };

        // 7. 将当前帧更新为下一轮的抑制感受野背景
        self.当前帧灰度.copy_to(&mut self.上一帧灰度).map_err(|e| e.to_string())?;

        Ok(动态势场 {
            斥力网格,
            最高危险等级: 最大值 as f32 / 255.0,
            逃逸方向,
        })
    }

    fn 获取调试帧(&self) -> Result<Mat, String> {
        Ok(self.势场图.clone())
    }
}