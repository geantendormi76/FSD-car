use opencv::{
    prelude::*,
    core::{self, Mat, Point, Scalar},
    imgproc,
    highgui,
    videoio::{VideoCapture, CAP_ANY},
};
use std::time::Duration;
use std::sync::{Arc, Mutex};
use std::thread;

// 🛡️ 核心并网：直接从工作空间的核心算法库中，引入感知器和匹配器
use core_perception::perception::xfeat_engine::仿生特征提取器;
use core_perception::perception::matcher::仿生匹配器;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("{}", "=".repeat(80));
    println!("🛡️  启动【WSL2 仿生视觉 XFeat 纠偏与双通道画线验证沙盘 v2.5】");
    println!("{}", "=".repeat(80));

    // 1. 加载本地下载的标准 640x640 ONNX 模型权重
    let model_path = "model/xfeat_640x640.onnx";
    let 提取器 = Arc::new(仿生特征提取器::new(model_path)?);
    println!("✓ [ONNX] XFeat 神经网络推理引擎加载成功！");

    // 2. 连接物理主权手机视频源
    let 手机视频流地址 = "http://192.168.5.19:8080/video";
    let mut 视频源 = VideoCapture::from_file(手机视频流地址, CAP_ANY)?;
    if !VideoCapture::is_opened(&视频源)? {
        println!("❌ [并网失败] 物理视频并网失败，请检查手机网络与 IP 端口状态！");
        return Ok(());
    }
    println!("✓ [并网成功] 手机视频通路并网成功！正在启动 Mailbox 高频抓取...");

    // 获取并对齐物理视频源尺寸
    let 帧宽 = 视频源.get(opencv::videoio::CAP_PROP_FRAME_WIDTH)? as i32;
    let 帧高 = 视频源.get(opencv::videoio::CAP_PROP_FRAME_HEIGHT)? as i32;
    println!("✓ [分辨率对齐] 对齐参数: {}x{}", 帧宽, 帧高);

    // 3. 构建多线程 Single-Element Mailbox 信箱
    let 共享信箱 = Arc::new(Mutex::new(None));
    let 共享信箱_抓取 = 共享信箱.clone();

    thread::spawn(move || {
        loop {
            let mut 临时帧 = Mat::default();
            if let Ok(true) = 视频源.read(&mut 临时帧) {
                if !临时帧.empty() {
                    let mut lock = 共享信箱_抓取.lock().unwrap();
                    *lock = Some(临时帧);
                }
            }
        }
    });

    // 4. 等待 2 秒，小车开机并就位。然后“睁眼拍照”，保存基准帧与基准特征点
    println!("⏳ [系统初始化] 小车开机自检中，请将手机摄像头对准一处静止场景（地标站牌）...");
    tokio::time::sleep(Duration::from_secs(2)).await;

    let (基准帧_mat, 基准地标快照) = loop {
        let 当前帧 = {
            let mut lock = 共享信箱.lock().unwrap();
            lock.take()
        };
        if let Some(帧) = 当前帧 {
            println!("📸 [基准站牌录制] 咔嚓！提取 XFeat 骨干特征点...");
            let 提取器_clone = 提取器.clone();
            let 帧_clone = 帧.clone();
            let 特征结果 = tokio::task::spawn_blocking(move || {
                提取器_clone.提取特征(&帧_clone, 200)
            }).await??;
            
            println!("✓ 成功录制基准站牌！提取了 {} 个 XFeat 骨干特征点！", 特征结果.len());
            break (帧, 特征结果);
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    };

    println!("\n{}", "=".repeat(80));
    println!("📊 [探针数值监测面板开启] 按 'q' 键可退出可视化窗口");
    println!("格式: [数值探针] | 实时特征点 | 原始匹配 | RANSAC生存 | 几何一致性 | 纠偏判定");
    println!("{}", "=".repeat(80));

    // 创建可视化渲染窗口
    let 窗口名称 = "FSD Bionic Eye - XFeat Homing Sandbox";
    highgui::named_window(窗口名称, highgui::WINDOW_AUTOSIZE)?;

    // 5. 进入 30Hz 自适应智驾环
    loop {
        let 当前帧 = {
            let mut lock = 共享信箱.lock().unwrap();
            lock.take()
        };

        if let Some(帧) = 当前帧 {
            let 提取器_ref = 提取器.clone();
            let 基准_ref = 基准地标快照.clone();
            let 基准帧_mat_ref = 基准帧_mat.clone();
            let 帧_copy = 帧.clone();

            // 派发到 CPU 密集线程池进行并行特征计算
            let (计算结果, 渲染帧) = tokio::task::spawn_blocking(move || -> (Result<(), String>, Mat) {
                // 初始化双通道大画布 [宽 = 帧宽 * 2, 高 = 帧高]
                let mut 画布 = Mat::new_rows_cols_with_default(帧高, 帧宽 * 2, core::CV_8UC3, Scalar::all(0.0)).unwrap();
                
                // 🛡️ 极速顺序物理拷贝机制：通过局部作用域隔离借用周期，100% 绕开 Rust 双重可变借用红线！
                {
                    let mut row_slice = 画布.row_bounds_mut(0, 帧高).unwrap();
                    let mut 左画布 = row_slice.col_bounds_mut(0, 帧宽).unwrap();
                    帧_copy.copy_to(&mut 左画布).unwrap();
                }

                {
                    let mut row_slice = 画布.row_bounds_mut(0, 帧高).unwrap();
                    let mut 右画布 = row_slice.col_bounds_mut(帧宽, 帧宽 * 2).unwrap();
                    基准帧_mat_ref.copy_to(&mut 右画布).unwrap();
                }

                let 实时特征 = match 提取器_ref.提取特征(&帧_copy, 200) {
                    Ok(f) => f,
                    Err(e) => return (Err(e), 画布),
                };
                
                // 极速交叉匹配
                let 匹配对 = 仿生匹配器::交叉匹配(&实时特征, &基准_ref, 0.82);

                // RANSAC 对极几何说谎者过滤
                let 过滤结果 = 仿生匹配器::几何纠偏过滤(&实时特征, &基准_ref, &匹配对, 3.0);

                match 过滤结果 {
                    Ok(干净匹配) => {
                        let 一致性 = if !匹配对.is_empty() {
                            (干净匹配.len() as f32 / 匹配对.len() as f32) * 100.0
                        } else {
                            0.0
                        };

                        // 结构化探针数据输出
                        println!(
                            "[数值探针] | 实时特征点: {:>3} | 原始匹配: {:>3} | RANSAC生存: {:>3} | 几何一致性: {:>5.1}% | 状态: 位置纠偏成功 🧭",
                            实时特征.len(),
                            匹配对.len(),
                            干净匹配.len(),
                            一致性
                        );

                        // 绘制匹配连线
                        for &(idx1, idx2, _) in &干净匹配 {
                            let pt1 = &实时特征[idx1];
                            let pt2 = &基准_ref[idx2];

                            let 实时坐标 = Point::new(pt1.x as i32, pt1.y as i32);
                            // 历史特征点在右侧子画布，X 坐标需要向右平移 1 个帧宽
                            let 历史坐标 = Point::new(pt2.x as i32 + 帧宽, pt2.y as i32);

                            // 在左侧画绿色圆圈，右侧画蓝色圆圈
                            let _ = imgproc::circle(&mut 画布, 实时坐标, 4, Scalar::new(0.0, 255.0, 0.0, 0.0), 2, imgproc::LINE_8, 0);
                            let _ = imgproc::circle(&mut 画布, 历史坐标, 4, Scalar::new(255.0, 0.0, 0.0, 0.0), 2, imgproc::LINE_8, 0);
                            
                            // 用绿色亮线条连起来
                            let _ = imgproc::line(&mut 画布, 实时坐标, 历史坐标, Scalar::new(0.0, 255.0, 0.0, 0.0), 1, imgproc::LINE_8, 0);
                        }
                    }
                    Err(_e) => {
                        println!(
                            "[数值探针] | 实时特征点: {:>3} | 原始匹配: {:>3} | RANSAC生存:   0 | 几何一致性:  0.0% | 状态: ⚠️ 自愈挂盘(匹配数少于8)",
                            实时特征.len(),
                            匹配对.len()
                        );
                    }
                }
                (Ok(()), 画布)
            }).await?;

            if let Err(e) = 计算结果 {
                println!("❌ 算法引擎异常: {}", e);
            }

            // 在窗口渲染双通道画面
            highgui::imshow(窗口名称, &渲染帧)?;
            
            // 检测键盘输入，按 'q' 退出
            let key = highgui::wait_key(10)?;
            if key == 113 { // 'q' 的 ASCII 码
                break;
            }
        }

        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    Ok(())
}