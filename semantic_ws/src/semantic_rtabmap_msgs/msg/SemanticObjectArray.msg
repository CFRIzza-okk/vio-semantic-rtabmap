# 一帧内的全部语义物体观测

std_msgs/Header header

# 自节点启动以来的处理帧序号，便于观测丢帧
uint64 frame_seq

# 本帧 YOLO 推理耗时（毫秒），用于实时性分析章节
float32 inference_ms

SemanticObject[] objects
