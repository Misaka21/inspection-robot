#ifndef PATH_PLANNER_IK_ADAPTER_HPP
#define PATH_PLANNER_IK_ADAPTER_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <vector>

namespace path_planner {

/**
 * @brief MoveIt IK封装
 *
 * 检查给定AGV站位下，机械臂是否能够到达TCP目标。
 * 封装MoveIt IK调用，提供简化接口。
 */
class IkAdapter {
public:
    struct IkResult {
        bool reachable = false;              // 是否可达
        std::vector<double> joint_angles;    // 关节角解（6轴）
        double manipulability = 0.0;         // 可操作性度量
        std::string error_msg;               // 错误信息
    };

    /**
     * @brief 初始化，加载机器人模型
     *
     * @param node ROS节点（用于获取参数和日志）
     * @param planning_group 规划组名称（如"elfin_arm"）
     * @return bool 是否初始化成功
     */
    bool init(rclcpp::Node::SharedPtr node,
              const std::string& planning_group = "elfin_arm");

    /**
     * @brief 检查可达性
     *
     * @param agv_pose AGV位姿（map系）
     * @param tcp_goal TCP目标位姿（map系）
     * @return IkResult IK求解结果
     */
    IkResult check_reachability(
        const geometry_msgs::msg::Pose& agv_pose,
        const geometry_msgs::msg::Pose& tcp_goal);

    /**
     * @brief 获取关节名称
     */
    const std::vector<std::string>& get_joint_names() const { return joint_names_; }

private:
    /**
     * @brief 将map系的TCP目标转换到机械臂base系
     */
    geometry_msgs::msg::Pose transform_to_base(
        const geometry_msgs::msg::Pose& tcp_in_map,
        const geometry_msgs::msg::Pose& agv_in_map);

    /**
     * @brief 计算可操作性（雅可比矩阵条件数的倒数）
     */
    double compute_manipulability(
        const std::vector<double>& joint_values);

    rclcpp::Node::SharedPtr node_;
    moveit::core::RobotModelPtr robot_model_;
    const moveit::core::JointModelGroup* joint_group_ = nullptr;
    std::vector<std::string> joint_names_;

    // URDF中机械臂base相对于AGV base的偏移
    // 假设：AGV base_link 与 arm_base 有固定偏移
    double arm_base_offset_x_ = 0.0;
    double arm_base_offset_y_ = 0.0;
    double arm_base_offset_z_ = 0.0;
};

}  // namespace path_planner

#endif  // PATH_PLANNER_IK_ADAPTER_HPP
