use core_perception::perception::pidnet_engine::PidnetEngine;
use core_perception::perception::ipm_projector::IpmProjector;
use dora_node_api::{DoraNode, Event, MetadataParameters};
use eyre::eyre;
use opencv::prelude::*;

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
                    
                    if tick_count % 30 == 0 {
                        println!("[感知遥测] 帧数: {:<6} | 192x192 BEV 网格已向 DORA 总线广播！", tick_count);
                    }
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
