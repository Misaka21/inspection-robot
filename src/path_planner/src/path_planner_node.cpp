#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace path_planner {

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode()
        : Node("path_planner_node") {
        RCLCPP_INFO(this->get_logger(), "Starting Path Planner Node");

        // 订阅工件位姿
        workpiece_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/inspection/perception/detected_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                workpiece_pose_ = *msg;
                has_workpiece_pose_ = true;
            });

        // 订阅 AGV 当前位姿
        agv_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/inspection/agv/current_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                agv_pose_ = *msg;
                has_agv_pose_ = true;
            });

        // 发布规划路径
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "~/path", 10);

        // 声明参数
        this->declare_parameter("camera_working_dist", 0.3);
        this->declare_parameter("arm_reach_max", 0.8);
        this->declare_parameter("arm_reach_min", 0.2);
        this->declare_parameter("candidate_radius", 0.6);
        this->declare_parameter("candidate_yaw_step_deg", 15.0);

        // 创建服务
        optimize_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/optimize",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                RCLCPP_INFO(this->get_logger(), "Received optimize request");
                plan();  // 调用规划
                response->success = true;
                response->message = "Path optimization triggered";
            });
    }

    void plan() {
        if (!has_workpiece_pose_ || !has_agv_pose_) {
            RCLCPP_WARN(this->get_logger(), "Missing workpiece or AGV pose");
            return;
        }

        // TODO: 实现联合路径规划
        // 1. 采样候选 AGV 站位
        // 2. 位姿链反推 TCP
        // 3. IK 求解筛选
        // 4. MoveJ 碰撞检查
        // 5. 代价函数评估

        RCLCPP_INFO(this->get_logger(), "Path planning not yet implemented");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr workpiece_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr agv_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr optimize_srv_;

    geometry_msgs::msg::PoseStamped workpiece_pose_;
    geometry_msgs::msg::PoseStamped agv_pose_;
    bool has_workpiece_pose_ = false;
    bool has_agv_pose_ = false;
};

}  // namespace path_planner

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planner::PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
