# Inspection path - 完整规划路径（AGV + 机械臂联合路径）
std_msgs/Header header

# 站位点序列
InspectionWaypoint[] waypoints

# 总移动距离（米）
float64 total_distance

# 规划是否成功
bool success
string error_message
