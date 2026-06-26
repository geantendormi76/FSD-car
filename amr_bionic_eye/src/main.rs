mod perception;

use opencv::{
    prelude::*,
    videoio::{VideoCapture, CAP_ANY},
};
use perception::frog_eye::{仿生青蛙眼, 伪青蛙眼感知器};
use std::time::Duration;
use std::sync::{Arc, Mutex};
use std::thread;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🛡️ 启动【纯视觉具身智能 AMR 仿真沙盘 V2.5】");

    // 手机 IP Webcam 的并网视频流地址
    let 手机视频流地址 = "http://192.168.5.19:8080/video";
    
    // 打开视频流通道
    let mut 视频源 = VideoCapture::from_file(手机视频流地址, CAP_ANY)?;
    if !VideoCapture::is_opened(&视频源)? {
        println!("❌ 物理视频流并网失败，请检查 WSL2 网络与手机 App 状态！");
        return Ok(());
    }
    println!("✓ 物理视频通道并网成功！");

    // 获取视频源的物理尺寸
    let 帧宽 = 视频源.get(opencv::videoio::CAP_PROP_FRAME_WIDTH)? as i32;
    let 帧高 = 视频源.get(opencv::videoio::CAP_PROP_FRAME_HEIGHT)? as i32;
    println!("✓ 图像分辨率对齐: {}x{}", 帧宽, 帧高);

    // 在堆中创建我们的仿生青蛙眼，并完成零拷贝预分配
    let mut 青蛙眼 = 仿生青蛙眼::new();
    青蛙眼.初始化(帧宽, 帧高)?;
    println!("✓ 仿生感受野零拷贝预分配完成！");

    // 🛡️ 工业级并网核心：建立线程安全的单元素覆盖缓冲区（Mailbox Pattern）
    // 彻底解决网络视频流在 TCP 缓冲区堆积导致的累进延迟问题
    let 共享信箱 = Arc::new(Mutex::new(None));
    let 共享信箱_抓取线程 = 共享信箱.clone();

    // 启动高优先级硬件视频帧捕获线程，以硬件最大速度榨干网络缓冲区
    thread::spawn(move || {
        loop {
            let mut 临时帧 = Mat::default();
            // 持续阻塞式读取，该调用由手机摄像头的物理 FPS 决定频率（如 30Hz）
            if let Ok(true) = 视频源.read(&mut 临时帧) {
                if !临时帧.empty() {
                    let mut 信箱锁 = 共享信箱_抓取线程.lock().unwrap();
                    // 覆盖写入最新的帧，未被消费的旧帧将被 Rust 自动 Drop 析构，确保没有延迟堆积
                    *信箱锁 = Some(临时帧); 
                }
            }
        }
    });
    
    // 模拟底盘传来的陀螺仪 Yaw 偏差量
    let 模拟_yaw_角速度 = 0.0f32; 

    loop {
        // 尝试从信箱中提取最新的物理帧
        let 当前帧 = {
            let mut 信箱锁 = 共享信箱.lock().unwrap();
            信箱锁.take() // 消费掉当前帧，信箱归为 None
        };

        if let Some(原始帧) = 当前帧 {
            // 🛡️ 绝对防线：为了不阻塞 Tokio 异步主线程，我们将图像计算派发到专用阻塞线程池中
            let mut 青蛙眼_clone = 青蛙眼; // 所有权转移进行计算
            let 原始帧_clone = 原始帧.clone(); // 浅拷贝，共享底层像素指针
            
            let (更新后的青蛙眼, 势场结果) = tokio::task::spawn_blocking(move || {
                let 结果 = 青蛙眼_clone.处理图像帧(&原始帧_clone, 模拟_yaw_角速度);
                (青蛙眼_clone, 结果)
            }).await?;

            // 收回感知器所有权，维持状态金库
            青蛙眼 = 更新后的青蛙眼;

            match 势场结果 {
                Ok(势场) => {
                    if 势场.最高危险等级 > 0.1 {
                        println!(
                            "🚨 避障警报！最高危险指数: {:.2} | 逃逸方向: ({:.2}, {:.2})",
                            势场.最高危险等级, 势场.逃逸方向.0, 势场.逃逸方向.1
                        );
                    } else {
                        println!("✓ 视线内安全，保持巡航...");
                    }
                }
                Err(e) => println!("❌ 感知计算异常: {}", e),
            }
        }

        // 极速自适应自旋周期 (10ms)，确保以最高响应速度轮询最新图像，彻底告别累进卡顿
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}