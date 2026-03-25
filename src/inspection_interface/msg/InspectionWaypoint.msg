# Inspection waypoint - 单个站位点（AGV + 机械臂联合状态）
std_msgs/Header header

# AGV 目标位姿（地图坐标系）
geometry_msgs/Pose agv_pose

# 机械臂关节角（弧度），顺序与 robot_description 一致
float64[] joint_angles

# 对应检测点ID（用于追溯）
int32 detection_point_id

# 该站位的可操作性度量（可选，用于调试/优化）
float64 manipulability
