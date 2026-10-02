# 导航层状态反馈, 周期发布, 供地面站/上位机监控
std_msgs/Header header

# 状态机当前状态:
#   IDLE      - 空闲, 等待指令
#   PLANNING  - 正在用 Nav2 规划路径
#   FOLLOWING - 正在沿路径飞行
#   HOLD      - 已到达目标点, 悬停等待外部指令(终止状态)
#   FAILED    - 规划失败/超时, 已放弃本次任务
string state

# ---- 当前导航目标(未导航时为空/0) ----
string target_class
int32 target_instance_id
geometry_msgs/PoseStamped goal

# ---- 载具当前位姿(map 帧) ----
geometry_msgs/PoseStamped current_pose

# 到目标停靠点的剩余距离(米); 无目标时为 -1
float32 distance_to_goal

# ---- 路径信息 ----
uint32 path_points
float32 path_length

# 本次任务已耗时(秒)
float32 elapsed

# 状态补充说明(失败原因 / 重规划次数等)
string message
