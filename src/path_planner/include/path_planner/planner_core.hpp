#ifndef PATH_PLANNER_PLANNER_CORE_HPP
#define PATH_PLANNER_PLANNER_CORE_HPP

#include "path_planner/ik_adapter.hpp"
#include "path_planner/stance_sampler.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace path_planner {

/**
 * @brief 规划核心（纯算法，无ROS定时器/订阅）
 *
 * 整合采样+IK+筛选，生成AGV+机械臂的联合规划。
 */
class PlannerCore {
public:
    struct PlanningRequest {
        // 检测点位（工件上的多个目标点）
        std::vector<geometry_msgs::msg::Pose> detection_points;

        // 当前AGV位姿（作为路径起点）
        geometry_msgs::msg::Pose current_agv_pose;

        // 约束参数
        double camera_working_dist = 0.3;     // 相机工作距离（米）
        double camera_dist_tolerance = 0.05;  // 距离容差（米）
        double arm_reach_min = 0.2;           // 机械臂最小可达（米）
        double arm_reach_max = 0.8;           // 机械臂最大可达（米）
        double candidate_radius = 0.6;        // AGV采样圆半径（米）
        double yaw_step_deg = 15.0;           // 采样角度步长（度）
        int max_candidates_per_point = 5;     // 每检测点保留的最大候选数
    };

    struct Waypoint {
        geometry_msgs::msg::Pose agv_pose;        // AGV目标位姿
        std::vector<double> joint_angles;         // 机械臂关节角
        int detection_point_id;                   // 对应哪个检测点
        double manipulability;                    // 可操作性度量
        double distance_to_current;               // 距离当前位置
    };

    struct PlanningResult {
        bool success = false;
        std::vector<Waypoint> waypoints;          // 按顺序的站位点
        double total_distance = 0.0;              // 总移动距离
        std::string error_msg;
    };

    /**
     * @brief 初始化
     */
    bool init(rclcpp::Node::SharedPtr node);

    /**
     * @brief 执行规划
     */
    PlanningResult plan(const PlanningRequest& req);

private:
    /**
     * @brief 为单个检测点生成候选站位
     */
    std::vector<Waypoint> generate_candidates_for_point(
        const geometry_msgs::msg::Pose& workpiece_pose,
        int point_id,
        const PlanningRequest& req);

    /**
     * @brief 用最近邻启发式选择站位并排序
     */
    std::vector<Waypoint> select_stances_and_order(
        const std::vector<std::vector<Waypoint>>& candidates_per_point,
        const geometry_msgs::msg::Pose& start_pose);

    /**
     * @brief 计算两个AGV位姿的欧氏距离
     */
    double compute_distance(const geometry_msgs::msg::Pose& a,
                           const geometry_msgs::msg::Pose& b);

    StanceSampler stance_sampler_;
    IkAdapter ik_adapter_;
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
};

}  // namespace path_planner

#endif  // PATH_PLANNER_PLANNER_CORE_HPP
