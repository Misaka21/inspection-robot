#include "path_planner/ik_adapter.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace path_planner {

bool IkAdapter::init(rclcpp::Node::SharedPtr node,
                     const std::string& planning_group) {
    node_ = node;

    // 从参数获取机械臂base偏移
    node_->declare_parameter("arm_base_offset_x", 0.0);
    node_->declare_parameter("arm_base_offset_y", 0.0);
    node_->declare_parameter("arm_base_offset_z", 0.0);
    node_->get_parameter("arm_base_offset_x", arm_base_offset_x_);
    node_->get_parameter("arm_base_offset_y", arm_base_offset_y_);
    node_->get_parameter("arm_base_offset_z", arm_base_offset_z_);

    // 加载机器人模型
    robot_model_loader::RobotModelLoader model_loader(node_, "robot_description");
    robot_model_ = model_loader.getModel();

    if (!robot_model_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load robot model");
        return false;
    }

    joint_group_ = robot_model_->getJointModelGroup(planning_group);
    if (!joint_group_) {
        RCLCPP_ERROR(node_->get_logger(), "Planning group '%s' not found",
                     planning_group.c_str());
        return false;
    }

    joint_names_ = joint_group_->getActiveJointModelNames();
    RCLCPP_INFO(node_->get_logger(), "IKAdapter initialized with %zu joints",
                joint_names_.size());

    return true;
}

// Pose转换为Eigen::Isometry3d的辅助函数
static Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() << pose.position.x, pose.position.y, pose.position.z;
    Eigen::Quaterniond q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    transform.rotate(q);
    return transform;
}

// Eigen::Isometry3d转换为Pose的辅助函数
static geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    Eigen::Quaterniond q(transform.rotation());
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    return pose;
}

geometry_msgs::msg::Pose IkAdapter::transform_to_base(
    const geometry_msgs::msg::Pose& tcp_in_map,
    const geometry_msgs::msg::Pose& agv_in_map) {

    // map -> agv_base -> arm_base
    // tcp_in_map 在 map 系中
    // 需要转换到 arm_base 系中

    // 1. 计算 agv -> tcp 的变换
    Eigen::Isometry3d T_map_agv = poseToEigen(agv_in_map);
    Eigen::Isometry3d T_map_tcp = poseToEigen(tcp_in_map);

    // 2. arm_base 相对于 agv_base 的偏移
    Eigen::Isometry3d T_agv_arm = Eigen::Isometry3d::Identity();
    T_agv_arm.translation() << arm_base_offset_x_, arm_base_offset_y_, arm_base_offset_z_;

    // 3. map -> arm_base
    Eigen::Isometry3d T_map_arm = T_map_agv * T_agv_arm;

    // 4. arm_base -> tcp = (map->arm_base)^-1 * (map->tcp)
    Eigen::Isometry3d T_arm_tcp = T_map_arm.inverse() * T_map_tcp;

    return eigenToPose(T_arm_tcp);
}

double IkAdapter::compute_manipulability(const std::vector<double>& joint_values) {
    if (!robot_model_ || !joint_group_) {
        return 0.0;
    }

    // 创建robot state
    moveit::core::RobotState state(robot_model_);
    state.setJointGroupPositions(joint_group_, joint_values);

    // 获取雅可比矩阵
    Eigen::MatrixXd jacobian;
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    const std::vector<const moveit::core::LinkModel*>& links = joint_group_->getLinkModels();
    if (links.empty()) {
        return 0.0;
    }
    const std::string& tip_link_name = links.back()->getName();
    if (!state.getJacobian(joint_group_,
                           state.getLinkModel(tip_link_name),
                           reference_point,
                           jacobian)) {
        return 0.0;
    }

    // 可操作性 = sqrt(det(J * J^T))
    // 简化：用条件数的倒数，或最小奇异值
    Eigen::MatrixXd jjT = jacobian * jacobian.transpose();
    double det = jjT.determinant();
    return std::sqrt(std::max(0.0, det));
}

IkAdapter::IkResult IkAdapter::check_reachability(
    const geometry_msgs::msg::Pose& agv_pose,
    const geometry_msgs::msg::Pose& tcp_goal) {

    IkResult result;

    if (!robot_model_ || !joint_group_) {
        result.error_msg = "IKAdapter not initialized";
        return result;
    }

    // 转换到机械臂base系
    geometry_msgs::msg::Pose tcp_in_base = transform_to_base(tcp_goal, agv_pose);

    // 创建robot state
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();

    // 设置IK超时
    double timeout = 0.1;  // 100ms

    // 调用IK
    bool found_ik = state.setFromIK(joint_group_,
                                    tcp_in_base,
                                    joint_group_->getLinkModels().back()->getName(),
                                    timeout);

    if (!found_ik) {
        result.error_msg = "IK solution not found";
        return result;
    }

    // 提取关节角
    state.copyJointGroupPositions(joint_group_, result.joint_angles);

    // 检查关节限位
    const std::vector<const moveit::core::JointModel*>& joints = joint_group_->getActiveJointModels();
    for (size_t i = 0; i < joints.size() && i < result.joint_angles.size(); ++i) {
        double angle = result.joint_angles[i];

        // 获取该关节的所有变量边界
        const std::vector<moveit::core::VariableBounds>& bounds = joints[i]->getVariableBounds();
        if (bounds.empty()) continue;

        // 处理连续关节（如腕部旋转）
        if (bounds[0].position_bounded_) {
            // 归一化到 [-pi, pi]
            while (angle > M_PI) angle -= 2 * M_PI;
            while (angle < -M_PI) angle += 2 * M_PI;

            if (angle < bounds[0].min_position_ || angle > bounds[0].max_position_) {
                result.error_msg = "Joint " + std::to_string(i) + " out of bounds";
                return result;
            }
        }
    }

    // 计算可操作性
    result.manipulability = compute_manipulability(result.joint_angles);
    result.reachable = true;

    return result;
}

}  // namespace path_planner
