use opencv::{
    prelude::*,
    core::{self, Mat, Scalar},
    imgproc,
    highgui,
    videoio::{VideoCapture, CAP_ANY},
};
use core_perception::perception::frog_eye::{仿生青蛙眼, 伪青蛙眼感知器};
use std::time::Duration;
use std::sync::{Arc, Mutex};
use std::thread;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("{}", "=".repeat(80));
    println!("🛡️  启动【WSL2 仿生青蛙眼 - 2D 动态人地势场可视化大屏 v3.0】");
    println!("{}", "=".repeat(80));

    // 1. 连接物理主权手机视频源
    let 手机视频流地址 = "http://192.168.5.19:8080/video";
    let mut 视频源 = VideoCapture::from_file(手机视频流地址, CAP_ANY)?;
    if !VideoCapture::is_opened(&视频源)? {
        println!("❌ [并网失败] 物理视频并网失败，请检查手机网络与 IP 摄像头开启状态！");
        return Ok(());
    }
    println!("✓ [并网成功] 手机视频通路并网成功！正在启动 Mailbox 高频抓取...");

    // 获取并对齐物理视频源尺寸
    let 帧宽 = 视频源.get(opencv::videoio::CAP_PROP_FRAME_WIDTH)? as i32;
    let 帧高 = 视频源.get(opencv::videoio::CAP_PROP_FRAME_HEIGHT)? as i32;
    println!("✓ [分辨率对齐] 对齐参数: {}x{}", 帧宽, 帧高);

    // 2. 初始化仿生青蛙眼感知器并完成零拷贝预分配
    let mut 青蛙眼 = 仿生青蛙眼::new(); // 🛡️ 修复：调用零参数的标准构造函数
    // 强制调用感知器金库初始化
    青蛙眼.初始化(帧宽, 帧高).map_err(|e| e.to_string())?;
    println!("✓ [零拷贝] 仿生青蛙眼感受野内存金库预分配完成！");

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

    println!("\n{}", "=".repeat(80));
    print!("📊 [势场监测大屏开启] 请挥舞你的双手，或者移动摄像头观测势场强度的变化...\n");
    println!("📊 按 'q' 键可安全退出大屏");
    println!("{}", "=".repeat(80));

    // 创建可视化渲染窗口
    let 窗口名称 = "WSL2 - Pseudo Frog-Eye Potential Field";
    highgui::named_window(窗口名称, highgui::WINDOW_AUTOSIZE)?;

    // 4. 进入 30Hz 自适应避障感知环
    loop {
        let 当前帧 = {
            let mut lock = 共享信箱.lock().unwrap();
            lock.take()
        };

        if let Some(帧) = 当前帧 {
            // 避免阻塞主线程，派发到密集线程池计算势场
            let mut 青蛙眼_clone = 青蛙眼;
            let 帧_clone = 帧.clone();
            
            let (更新后的青蛙眼, 势场计算结果) = tokio::task::spawn_blocking(move || {
                // 模拟偏航角增量为 0.0 (静态测试)
                let 结果 = 青蛙眼_clone.处理图像帧(&帧_clone, 0.0);
                (青蛙眼_clone, 结果)
            }).await?;

            青蛙眼 = 更新后的青蛙眼;

            match 势场计算结果 {
                Ok(势场) => {
                    // 获取模糊后的 2D 势场灰度大图
                    if let Ok(势场图) = 青蛙眼.获取调试帧() {
                        let mut 热力图 = Mat::default();
                        let mut 渲染混合帧 = Mat::default();

                        // 1. 将 2D 连续势场映射为 JET 伪彩色热力图 (越红代表斥力越大，越危险)
                        imgproc::apply_color_map(&势场图, &mut 热力图, imgproc::COLORMAP_JET)?;

                        // 2. 将原图与热力图以 0.6 : 0.4 的物理高保真比例融合
                        core::add_weighted(&帧, 0.6, &热力图, 0.4, 0.0, &mut 渲染混合帧, -1)?;

                        // 3. 在画面左上角渲染最高危险等级指标，便于直观观测
                        let 警报文字 = format!("Max Threat Level: {:.2}", 势场.最高危险等级);
                        let 颜色 = if 势场.最高危险等级 > 0.3 {
                            Scalar::new(0.0, 0.0, 255.0, 0.0) // 危险：红色
                        } else {
                            Scalar::new(0.0, 255.0, 0.0, 0.0) // 安全：绿色
                        };
                        
                        imgproc::put_text(
                            &mut 渲染混合帧,
                            &警报文字,
                            core::Point::new(20, 40),
                            imgproc::FONT_HERSHEY_SIMPLEX,
                            0.8,
                            颜色,
                            2,
                            imgproc::LINE_AA,
                            false
                        )?;

                        // 在大窗口进行实时高帧率渲染
                        highgui::imshow(窗口名称, &渲染混合帧)?;
                    }
                }
                Err(e) => println!("❌ [势场解算异常]: {}", e),
            }

            // 检测键盘输入，按 'q' 退出
            let key = highgui::wait_key(10)?;
            if key == 113 { 
                break;
            }
        }

        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    Ok(())
}