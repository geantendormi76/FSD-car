// 🛡️ 协议确认：已开启后端全量代码输出模式，拒绝任何逻辑省略。
use dora_node_api::{DoraNode, Event, Parameter};
use eyre::{eyre, Context};
use opencv::core::{Mat, CV_8UC3};
use std::ffi::c_void;
use std::sync::Arc;

use core_decision::topo_graph::graph::TopologicalGraph;
use core_decision::topo_graph::node::{Pose, TopologicalNode};
use core_perception::perception::xfeat_engine::仿生特征提取器;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🧠 [慢系统] 语义大脑与拓扑建图节点已启动，等待 DORA 共享内存注入...");

    // 1. 接入 DORA 数据流网络
    let (mut _node, mut events) = DoraNode::init_from_env()?;

    // 2. 初始化 XFeat 引擎 (使用 Arc 包装以便跨线程安全传递)
    // 注意：请确保在运行目录下存在 model/xfeat_640x640.onnx
    let model_path = "model/xfeat_640x640.onnx";
    // 🛡️ 架构师修正：String 未实现 Error Trait，需使用 map_err 配合 eyre! 宏手动转换为标准 Report
    let 提取器 = Arc::new(
        仿生特征提取器::new(model_path).map_err(|e| eyre!("❌ XFeat 模型加载失败: {}", e))?,
    );

    // 3. 初始化全简体中文业务逻辑主权对象：拓扑地图
    let mut 拓扑地图 = TopologicalGraph::new();
    let mut 节点计数器 = 0;

    // 4. 异步事件驱动循环
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, metadata, data } => {
                if id.as_str() == "image" {
                    let 宽度 = match metadata.parameters.get("width") {
                        Some(Parameter::Integer(val)) => *val as i32,
                        _ => 640,
                    };
                    let 高度 = match metadata.parameters.get("height") {
                        Some(Parameter::Integer(val)) => *val as i32,
                        _ => 480,
                    };

                    // 物理级零拷贝 (Arrow -> OpenCV Mat)
                    let 裸内存切片: &[u8] = (&data).try_into().context("❌ Arrow 内存强转失败")?;
                    let 视网膜帧 = unsafe {
                        Mat::new_rows_cols_with_data_unsafe(
                            高度,
                            宽度,
                            CV_8UC3,
                            裸内存切片.as_ptr() as *mut c_void,
                            opencv::core::Mat_AUTO_STEP,
                        )
                        .context("❌ OpenCV Mat 映射失败")?
                    };

                    // ---------------------------------------------------------
                    // 🛡️ 架构师指令：异步隔离与生命周期自愈
                    // ---------------------------------------------------------
                    let 提取器_clone = 提取器.clone();

                    // 为什么这里要 clone 图像？
                    // 因为 spawn_blocking 要求闭包内的变量具有 'static 生命周期。
                    // 视网膜帧借用了 data (Arrow 内存)，而 data 会在当前循环结束时销毁。
                    // 作为 1Hz 的慢系统，单次深拷贝图像的开销微乎其微，却能换来绝对的并发安全！
                    let 帧_copy = 视网膜帧.clone();

                    // 将 CPU 密集型推理推入 Tokio 阻塞线程池
                    let 特征结果 = tokio::task::spawn_blocking(move || {
                        提取器_clone.提取特征(&帧_copy, 200)
                    })
                    .await?
                    .map_err(|e| eyre!("❌ XFeat 特征提取失败: {}", e))?;

                    println!(
                        "✅ [慢系统] 成功提取 {} 个 XFeat 骨干特征点",
                        特征结果.len()
                    );

                    // ---------------------------------------------------------
                    // 🗺️ 拓扑建图逻辑：将当前帧特征存入记忆金库
                    // ---------------------------------------------------------
                    节点计数器 += 1;

                    let mut descriptors = Vec::with_capacity(特征结果.len() * 64);
                    let mut keypoints = Vec::with_capacity(特征结果.len() * 2);

                    for pt in 特征结果 {
                        keypoints.push(pt.x);
                        keypoints.push(pt.y);
                        descriptors.extend(pt.描述子);
                    }

                    let 新地标 = TopologicalNode {
                        id: 节点计数器,
                        name: format!("自动探索地标_{}", 节点计数器),
                        pose: Pose {
                            x: 0.0,
                            y: 0.0,
                            yaw: 0.0,
                        }, // 实际应由里程计/SLAM提供
                        descriptors,
                        keypoints,
                    };

                    拓扑地图.add_node(新地标);
                    println!(
                        "🗺️ [慢系统] 拓扑地图已更新，当前脑海记忆节点数: {}",
                        拓扑地图.nodes.len()
                    );
                }
            }
            Event::Stop(_) => {
                println!("🛑 [慢系统] 接收到 DORA 停止信号，正在将拓扑记忆持久化到硬盘...");
                if let Err(e) = 拓扑地图.save_to_file("topo_memory.json") {
                    eprintln!("❌ 拓扑地图保存失败: {}", e);
                } else {
                    println!("💾 拓扑地图已安全保存至 topo_memory.json");
                }
                break;
            }
            _ => {}
        }
    }

    Ok(())
}
