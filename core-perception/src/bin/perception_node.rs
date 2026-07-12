use core_perception::perception::pidnet_engine::PidnetEngine;
use core_perception::perception::ipm_projector::IpmProjector;
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use opencv::prelude::*;
use std::io::Write; 

#[tokio::main]
async fn main() -> eyre::Result<()> {
    println!("🛰️  [Rust 感知端] PIDNet-IPM 鸟瞰空间重构节点已启动...");
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let model_path = "model/pidnet_s.onnx";
    let pidnet = PidnetEngine::new(model_path)
        .map_err(|e| eyre!("Failed to init PIDNet: {}", e))?;
    println!("🟢 [感知自愈] PIDNet-S 算子推理引擎装载完毕。");
    let projector = IpmProjector::new(640, 480, 192)
        .map_err(|e| eyre!("Failed to init IPM Projector: {}", e))?;
    println!("🟢 [感知自愈] GPU-IPM 逆透视投影网关构建完毕。");
    let mut tick_count = 0u64;
    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() == "jpeg_image" {
                    tick_count += 1;
                    let jpeg_bytes: Vec<u8> = dora_node_api::into_vec(&data)
                        .map_err(|e| eyre!("Failed to extract jpeg: {}", e))?;
                    let src_vec = opencv::core::Vector::<u8>::from_slice(&jpeg_bytes);
                    let frame = opencv::imgcodecs::imdecode(&src_vec, opencv::imgcodecs::IMREAD_COLOR)
                        .map_err(|e| eyre!("OpenCV decoding failed: {}", e))?;
                    if frame.empty() {
                        continue;
                    }
                    let class_map = pidnet.segment(&frame)
                        .map_err(|e| eyre!("PIDNet segment failed: {}", e))?;
                    
                    // 🛡️ 架构师自愈探针：每 30 帧将分割分类矩阵与原始视讯图像导出到 /tmp 下用于逆向调试
                    if tick_count % 30 == 0 {
                        // A. 写入分类矩阵
                        let mut file = std::fs::File::create("/tmp/fsd_live_class.txt").ok();
                        if let Some(ref mut f) = file {
                            for y in (0..480).step_by(8) {
                                for x in (0..640).step_by(8) {
                                    if let Ok(class_val) = class_map.at_2d::<u8>(y, x) {
                                        let _ = write!(f, "{} ", class_val);
                                    }
                                }
                                let _ = writeln!(f);
                            }
                        }
                        // B. 写入原始图像帧（用于排查相机是否瞎了/黑了）
                        let params = opencv::core::Vector::<i32>::new();
                        let _ = opencv::imgcodecs::imwrite("/tmp/fsd_debug_frame.jpg", &frame, &params);
                    }

                    let bev_grid = projector.project(&class_map)
                        .map_err(|e| eyre!("IPM project failed: {}", e))?;
                    let mut flat_data = vec![0u8; 192 * 192];
                    let mut idx = 0;
                    for y in 0..192 {
                        let row = bev_grid.ptr(y as i32)
                            .map_err(|e| eyre!("Row pointer extract failed: {}", e))?;
                        let row_slice = unsafe { std::slice::from_raw_parts(row, 192) };
                        flat_data[idx..idx + 192].copy_from_slice(row_slice);
                        idx += 192;
                    }
                    let bev_arrow = dora_node_api::arrow::array::UInt8Array::from(flat_data);
                    node.send_output(
                        "bev_grid".to_string().into(),
                        MetadataParameters::default(),
                        bev_arrow,
                    )?;
                }
            }
            Event::Stop(_) => {
                println!("🛑 [Rust 感知端] 收到 DORA 停止信号，安全退出。");
                break;
            }
            _ => {}
        }
    }
    Ok(())
}
