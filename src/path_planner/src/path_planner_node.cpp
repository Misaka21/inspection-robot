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
        __workpiece_posesub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/inspection/perception/detected_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                _workpiece_pose = *msg;
                _has_workpiece_pose = true;
            });

        // 订阅 AGV 当前位姿
        __agv_posesub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/inspection/agv/current_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                _agv_pose = *msg;
                _has_agv_pose = true;
            });

        // 发布规划路径
        _path_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "~/path", 10);

        // 声明参数
        this->declare_parameter("camera_working_dist", 0.3);
        this->declare_parameter("arm_reach_max", 0.8);
        this->declare_parameter("arm_reach_min", 0.2);
        this->declare_parameter("candidate_radius", 0.6);
        this->declare_parameter("candidate_yaw_step_deg", 15.0);

        // 创建服务
        _optimize_srv = this->create_service<std_srvs::srv::Trigger>(
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
        if (!_has_workpiece_pose || !_has_agv_pose) {
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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr __workpiece_posesub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr __agv_posesub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _path_pub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _optimize_srv;

    geometry_msgs::msg::PoseStamped _workpiece_pose;
    geometry_msgs::msg::PoseStamped _agv_pose;
    bool _has_workpiece_pose = false;
    bool _has_agv_pose = false;
};

}  // namespace path_planner

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planner::PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
