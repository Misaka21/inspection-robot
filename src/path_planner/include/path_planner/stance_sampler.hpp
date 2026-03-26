#ifndef PATH_PLANNER_STANCE_SAMPLER_HPP
#define PATH_PLANNER_STANCE_SAMPLER_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace path_planner {

/**
 * @brief AGV站位采样器
 *
 * 在工件周围圆上采样候选AGV站位，并计算对应的机械臂TCP目标。
 * 纯几何计算，无ROS依赖。
 */
class StanceSampler {
public:
    struct Stance {
        geometry_msgs::msg::Pose agv_pose;   // AGV base位姿（map系）
        geometry_msgs::msg::Pose tcp_pose;   // 机械臂TCP目标（map系）
        double distance_to_workpiece;        // AGV到工件中心的距离
    };

    /**
     * @brief 为单个检测点采样候选站位
     *
     * @param workpiece_pose 工件位姿（检测点位置+表面法向）
     * @param camera_working_dist 相机到工件表面的最优距离（米）
     * @param candidate_radius AGV采样圆半径（米）
     * @param yaw_step_deg 采样角度步长（度）
     * @param agv_z AGV的z坐标（默认为0）
     * @return std::vector<Stance> 候选站位列表
     */
    std::vector<Stance> sample_stances(
        const geometry_msgs::msg::Pose& workpiece_pose,
        double camera_working_dist,
        double candidate_radius,
        double yaw_step_deg,
        double agv_z = 0.0);

private:
    /**
     * @brief 从偏航角计算四元数（只绕Z轴）
     */
    geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);

    /**
     * @brief 提取旋转矩阵的Z轴（法向）
     */
    void extract_z_axis(const geometry_msgs::msg::Pose& pose, double& x, double& y, double& z);

    /**
     * @brief 计算look-at旋转（Z轴指向目标）
     */
    geometry_msgs::msg::Pose compute_look_at_pose(
        const geometry_msgs::msg::Point& from,
        const geometry_msgs::msg::Point& to);
};

}  // namespace path_planner

#endif  // PATH_PLANNER_STANCE_SAMPLER_HPP
