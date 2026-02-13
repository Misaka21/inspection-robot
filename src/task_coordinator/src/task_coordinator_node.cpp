#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <inspection_interface/msg/system_state.hpp>
#include <inspection_interface/srv/start_inspection.hpp>
#include <inspection_interface/srv/stop_inspection.hpp>

namespace task_coordinator {

using inspection_interface::msg::SystemState;

class TaskCoordinatorNode : public rclcpp::Node {
public:
    TaskCoordinatorNode()
        : Node("task_coordinator_node"),
          current_phase_(SystemState::PHASE_IDLE) {
        RCLCPP_INFO(this->get_logger(), "Starting Task Coordinator Node");

        // 发布系统状态
        state_pub_ = this->create_publisher<inspection_interface::msg::SystemState>(
            "/inspection/state", 10);

        // 发布 AGV 目标点
        agv_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/inspection/agv/goal_pose", 10);

        // 订阅 AGV 状态
        agv_status_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/inspection/agv/current_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                (void)msg;
                agv_arrived_ = true;
            });

        // 订阅机械臂状态
        arm_status_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                (void)msg;
                arm_arrived_ = true;
            });

        // 服务
        start_srv_ = this->create_service<inspection_interface::srv::StartInspection>(
            "~/start",
            [this](const inspection_interface::srv::StartInspection::Request::SharedPtr req,
                   inspection_interface::srv::StartInspection::Response::SharedPtr res) {
                (void)req;
                start_inspection(res);
            });

        stop_srv_ = this->create_service<inspection_interface::srv::StopInspection>(
            "~/stop",
            [this](const inspection_interface::srv::StopInspection::Request::SharedPtr req,
                   inspection_interface::srv::StopInspection::Response::SharedPtr res) {
                (void)req;
                stop_inspection(res);
            });

        // 定时发布状态
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publish_state(); });
    }

private:
    void start_inspection(const inspection_interface::srv::StartInspection::Response::SharedPtr& res) {
        current_phase_ = SystemState::PHASE_LOCALIZING;
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Inspection started");
    }

    void stop_inspection(const inspection_interface::srv::StopInspection::Response::SharedPtr& res) {
        current_phase_ = SystemState::PHASE_IDLE;
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Inspection stopped");
    }

    void publish_state() {
        inspection_interface::msg::SystemState state;
        state.phase = current_phase_;
        state.progress_percent = 0.0f;
        state_pub_->publish(state);
    }

    rclcpp::Publisher<inspection_interface::msg::SystemState>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr agv_goal_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr agv_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_status_sub_;

    rclcpp::Service<inspection_interface::srv::StartInspection>::SharedPtr start_srv_;
    rclcpp::Service<inspection_interface::srv::StopInspection>::SharedPtr stop_srv_;

    rclcpp::TimerBase::SharedPtr timer_;

    uint8_t current_phase_;
    bool agv_arrived_ = false;
    bool arm_arrived_ = false;
};

}  // namespace task_coordinator

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<task_coordinator::TaskCoordinatorNode>());
    rclcpp::shutdown();
    return 0;
}
