#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "path_planner/planner_core.hpp"

namespace path_planner {

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode()
        : Node("path_planner_node") {
        RCLCPP_INFO(this->get_logger(), "Starting Path Planner Node with PlannerCore");

        // 声明参数
        this->declare_parameter("camera_working_dist", 0.3);
        this->declare_parameter("camera_dist_tolerance", 0.05);
        this->declare_parameter("arm_reach_min", 0.2);
        this->declare_parameter("arm_reach_max", 0.8);
        this->declare_parameter("candidate_radius", 0.6);
        this->declare_parameter("yaw_step_deg", 15.0);
        this->declare_parameter("max_candidates_per_point", 5);

        // 订阅输入
        detection_points_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "detection_points", 10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                detection_points_ = msg->poses;
                has_detection_points_ = true;
                RCLCPP_INFO(this->get_logger(), "Received %zu detection points",
                            detection_points_.size());
            });

        agv_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/inspection/agv/current_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_agv_pose_ = msg->pose;
                has_agv_pose_ = true;
            });

        // 发布规划结果
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path", 10);

        // 创建服务
        plan_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "optimize",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                response->success = execute_planning();
                response->message = response->success ? "Planning successful" : last_error_;
            });

        // 初始化 PlannerCore
        if (!planner_core_.init(shared_from_this())) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize PlannerCore");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "PathPlannerNode initialized");
    }

private:
    bool execute_planning() {
        if (!has_detection_points_) {
            last_error_ = "No detection points received";
            RCLCPP_ERROR(this->get_logger(), "%s", last_error_.c_str());
            return false;
        }

        if (!has_agv_pose_) {
            RCLCPP_WARN(this->get_logger(), "No AGV pose received, using origin");
            current_agv_pose_.orientation.w = 1.0;  // 默认在原点
        }

        // 构造规划请求
        PlannerCore::PlanningRequest req;
        req.detection_points = detection_points_;
        req.current_agv_pose = current_agv_pose_;

        this->get_parameter("camera_working_dist", req.camera_working_dist);
        this->get_parameter("camera_dist_tolerance", req.camera_dist_tolerance);
        this->get_parameter("arm_reach_min", req.arm_reach_min);
        this->get_parameter("arm_reach_max", req.arm_reach_max);
        this->get_parameter("candidate_radius", req.candidate_radius);
        this->get_parameter("yaw_step_deg", req.yaw_step_deg);
        this->get_parameter("max_candidates_per_point", req.max_candidates_per_point);

        // 执行规划
        auto result = planner_core_.plan(req);

        if (!result.success) {
            last_error_ = result.error_msg;
            return false;
        }

        // 发布路径
        geometry_msgs::msg::PoseArray path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (const auto& wp : result.waypoints) {
            path_msg.poses.push_back(wp.agv_pose);
        }

        path_pub_->publish(path_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published path with %zu waypoints, total distance: %.2fm",
                    path_msg.poses.size(), result.total_distance);

        return true;
    }

    PlannerCore planner_core_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detection_points_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr agv_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_srv_;

    std::vector<geometry_msgs::msg::Pose> detection_points_;
    geometry_msgs::msg::Pose current_agv_pose_;
    bool has_detection_points_ = false;
    bool has_agv_pose_ = false;
    std::string last_error_;
};

}  // namespace path_planner

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planner::PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
