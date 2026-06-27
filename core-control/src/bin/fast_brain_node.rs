// 🛡️ 极速并网：使用 DORA 内部重导出的 arrow
use dora_node_api::arrow::array::Float32Array;
use core_control::预测控制求解器;
use core_perception::perception::frog_eye::{仿生青蛙眼, 伪青蛙眼感知器};
// 🛡️ 架构师修正：直接引入 DORA 原生导出的 Parameter 枚举，消除不稳定的 metadata 模块依赖
use dora_node_api::{DoraNode, Event, MetadataParameters, Parameter};
use eyre::{eyre, Context};
use opencv::core::{Mat, CV_8UC3};
use std::ffi::c_void;

fn main() -> eyre::Result<()> {
    println!("🧠 [快系统] 规控大脑节点已启动，等待 DORA 共享内存注入...");

    // 1. 接入 DORA 数据流网络 (接管生命周期与共享内存池)
    let (mut node, mut events) = DoraNode::init_from_env()?;

    // 2. 初始化全简体中文业务逻辑主权对象
    let mut 青蛙眼 = 仿生青蛙眼::new();
    let mut 规控大脑 = 预测控制求解器::new().map_err(|e| eyre!(e))?;
    
    let mut 物理主权已初始化 = false;

    // 3. 事件驱动循环 (由底层 epoll 极速唤醒，避免线程死锁)
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => {
                // 拦截来自 Isaac Sim 的图像输入
                if id.as_str() == "image" {
                    // ---------------------------------------------------------
                    // [阶段 A]：元数据解析与状态金库初始化
                    // ---------------------------------------------------------
                    // 🛡️ 架构师修正：利用原生 Match 模式匹配提取参数，0 上游兼容风险，极限性能
                    let 宽度 = match metadata.parameters.get("width") {
                        Some(Parameter::Integer(val)) => *val as i32,
                        _ => 640,
                    };
                    let 高度 = match metadata.parameters.get("height") {
                        Some(Parameter::Integer(val)) => *val as i32,
                        _ => 480,
                    };

                    if !物理主权已初始化 {
                        青蛙眼.初始化(宽度, 高度).map_err(|e| eyre!(e))?;
                        规控大脑.设置当前状态(0.0, 0.0, 0.0, 0.0).map_err(|e| eyre!(e))?;
                        物理主权已初始化 = true;
                        println!("✅ [快系统] 感受野与 NMPC 求解器初始化完成 ({}x{})", 宽度, 高度);
                    }

                    // ---------------------------------------------------------
                    // [阶段 B]：物理级零拷贝 (Arrow -> OpenCV Mat)
                    // ---------------------------------------------------------
                    let 裸内存切片: &[u8] = (&data).try_into().context("❌ Arrow 内存强转 &[u8] 失败")?;

                    // 调用 new_rows_cols_with_data_unsafe 显式声明我们在操作裸指针
                    let 视网膜帧 = unsafe {
                        Mat::new_rows_cols_with_data_unsafe(
                            高度,
                            宽度,
                            CV_8UC3,
                            裸内存切片.as_ptr() as *mut c_void,
                            opencv::core::Mat_AUTO_STEP,
                        ).context("❌ OpenCV Mat 映射失败")?
                    };

                    // ---------------------------------------------------------
                    // [阶段 C]：仿生感知与规控解算
                    // ---------------------------------------------------------
                    let 势场 = 青蛙眼.处理图像帧(&视网膜帧, 0.0).map_err(|e| eyre!(e))?;

                    // 2. 势场梯度转化为 NMPC 参考轨迹偏移
                    let 期望_x = 0.3 + 势场.逃逸方向.0 as f64;
                    let 期望_y = 0.0 + 势场.逃逸方向.1 as f64;
                    
                    for k in 0..=20 {
                        规控大脑.设置参考轨迹点(k, 期望_x, 期望_y, 0.0, 0.3).map_err(|e| eyre!(e))?;
                    }
                    
                    let (线速度_v, 角速度_w) = 规控大脑.求解最优控制量(0.0).map_err(|e| eyre!(e))?;

                    // ---------------------------------------------------------
                    // [阶段 D]：零拷贝输出指令
                    // ---------------------------------------------------------
                    let 运动指令 = Float32Array::from(vec![线速度_v as f32, 角速度_w as f32]);
                    
                    node.send_output(
                        "control_cmd".to_string().into(),
                        MetadataParameters::default(),
                        运动指令,
                    ).context("❌ 控制指令发送失败")?;
                }
            }
            Event::Stop(_) => {
                println!("🛑 [快系统] 接收到 DORA 全局停止信号，安全释放 C 语言内存胶囊并退出...");
                break;
            }
            _ => {} 
        }
    }
    
    Ok(())
}