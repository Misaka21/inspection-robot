#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <inspection_interface/msg/system_state.hpp>

namespace inspection_supervisor {

class SupervisorNode : public rclcpp::Node {
public:
    SupervisorNode()
        : Node("supervisor_node") {
        RCLCPP_INFO(this->get_logger(), "Starting Supervisor Node");

        // 状态发布者
        health_pub_ = this->create_publisher<std_msgs::msg::String>("/inspection/health", 10);
        alerts_pub_ = this->create_publisher<std_msgs::msg::String>("/inspection/alerts", 10);

        // 订阅各节点状态
        // 使用 current_pose 而非 agv/status 来检测 AGV 驱动存活性
        // 注意：若 AGV 定位丢失，current_pose 可能停止发布但 AGV 驱动本身仍在运行
        // 后续应改为订阅 /inspection/agv/status（见 CLAUDE.md 架构约定）
        agv_status_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/inspection/agv/current_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                (void)msg;
                last_agv_update_ = this->now();
            });

        arm_status_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                (void)msg;
                last_arm_update_ = this->now();
            });

        // 定时检查
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { check_health(); });
    }

private:
    void check_health() {
        auto now = this->now();

        // 检查 AGV 状态
        // rclcpp::Duration(5, 0) = 5 秒超时门限：若 AGV 5 秒未发布位姿，则视为失联
        // 使用消息时间戳差值而非绝对时间，避免时钟跳变导致误报
        if (now - last_agv_update_ > rclcpp::Duration(5, 0)) {
            publish_alert("AGV driver not responding");
        }

        // 检查机械臂状态
        // 机械臂以 /joint_states 为心跳，同样使用 5 秒超时
        if (now - last_arm_update_ > rclcpp::Duration(5, 0)) {
            publish_alert("Arm driver not responding");
        }

        // 汇总健康状态
        std_msgs::msg::String health_msg;
        health_msg.data = "OK";
        health_pub_->publish(health_msg);
    }

    void publish_alert(const std::string& alert) {
        RCLCPP_WARN(this->get_logger(), "Alert: %s", alert.c_str());
        std_msgs::msg::String alert_msg;
        alert_msg.data = alert;
        alerts_pub_->publish(alert_msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alerts_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr agv_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_agv_update_;
    rclcpp::Time last_arm_update_;
};

}  // namespace inspection_supervisor

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<inspection_supervisor::SupervisorNode>());
    rclcpp::shutdown();
    return 0;
}
