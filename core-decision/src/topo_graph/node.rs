use serde::{Serialize, Deserialize};

/// 🛡️ 物理三维姿态量
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
}

/// 🛡️ 拓扑空间地标节点：作为小车在大脑中对物理世界一个“站牌”的记忆
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TopologicalNode {
    pub id: u32,                     // 唯一的节点站牌 ID
    pub name: String,                // 节点地标语义名称 (如 "5栋楼下", "1号别墅转角")
    pub pose: Pose,                  // 拍照时的里程计/北斗绝对坐标
    
    /// XFeat 描述子集合：扁平化的描述子特征向量 [N_features * 64 维]
    pub descriptors: Vec<f32>,
    
    /// 每个特征点的 (x, y) 亚像素物理坐标：扁平化的坐标矩阵 [N_features * 2]
    pub keypoints: Vec<f32>,
}