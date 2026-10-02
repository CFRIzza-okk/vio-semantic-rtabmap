# 一次观测到的语义物体（已反投影到 3D 并变换到 header.frame_id 坐标系）

std_msgs/Header header

# ---------------- 语义 ----------------
string  class_name          # 类别名，如 "person" / "car" / "chair"
int32   class_id            # 类别索引（YOLO 的 class id）
float32 confidence          # 本次观测的检测置信度 [0,1]

# ---------------- 实例（数据关联后） ----------------
int32   instance_id         # 全局实例 ID；-1 表示尚未关联到任何实例
int32   observation_count   # 该实例累计被观测到的次数
float32 fused_confidence    # 多帧融合后的置信度（对数几率融合）

# ---------------- 几何 ----------------
float32[4] bbox             # 2D 检测框 [x_min, y_min, x_max, y_max]，像素
geometry_msgs/Point   position  # 物体中心，header.frame_id 下，单位米
geometry_msgs/Vector3 size      # 3D 尺寸估计 (x=宽, y=高, z=可见厚度)，单位米
float32 yaw                 # 绕 z 轴朝向，弧度
float32 distance            # 到相机光心的距离，单位米
int32   points_used         # 参与反投影的有效深度像素数

# ---------------- 与 RTAB-Map 的关联 ----------------
int32   rtabmap_node_id     # 关联到的 RTAB-Map 节点 ID；-1 表示未关联

# ---------------- 语义约束质量 ----------------
# 由 semantic_mapper 根据检测质量自适应计算，供 RTAB-Map 软约束和调试使用
float32 landmark_linear_variance   # 地标位置方差（m^2）
float32 landmark_angular_variance  # 地标朝向方差（rad^2）
float32 semantic_quality            # [0,1]，越大表示越可信
