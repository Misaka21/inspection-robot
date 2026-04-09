#ifndef TASK_COORDINATOR_NODE_HPP
#define TASK_COORDINATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <inspection_interface/msg/agv_status.hpp>
#include <inspection_interface/msg/arm_status.hpp>
#include <inspection_interface/msg/system_state.hpp>
#include <inspection_interface/srv/start_inspection.hpp>
#include <inspection_interface/srv/stop_inspection.hpp>
#include <inspection_interface/srv/pause_inspection.hpp>
#include <inspection_interface/srv/resume_inspection.hpp>
#include <inspection_interface/srv/get_inspection_status.hpp>
#include <inspection_interface/srv/move_to_joints.hpp>
#include <inspection_interface/srv/move_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace task_coordinator {

using inspection_interface::msg::SystemState;

// YAML 配置中每个巡检站位的描述
struct StationConfig {
    std::string name;
    double agv_x{0}, agv_y{0}, agv_z{0}, agv_yaw{0};
    std::vector<double> arm_joints;
    double target_distance{0.30};
    double distance_tolerance{0.02};
    std::string adjust_axis{"z"};
};

class TaskCoordinatorNode : public rclcpp::Node {
public:
    explicit TaskCoordinatorNode(const rclcpp::NodeOptions& options);

private:
    // 状态机
    void run_state_machine();
    void handle_moving_to_station();
    void handle_arm_preset();
    void handle_depth_adjust();
    void handle_capturing();

    // 服务回调
    void start_inspection(
        const inspection_interface::srv::StartInspection::Request::SharedPtr req,
        inspection_interface::srv::StartInspection::Response::SharedPtr res);
    void stop_inspection(
        const inspection_interface::srv::StopInspection::Request::SharedPtr req,
        inspection_interface::srv::StopInspection::Response::SharedPtr res);
    void pause_inspection(
        const inspection_interface::srv::PauseInspection::Request::SharedPtr req,
        inspection_interface::srv::PauseInspection::Response::SharedPtr res);
    void resume_inspection(
        const inspection_interface::srv::ResumeInspection::Request::SharedPtr req,
        inspection_interface::srv::ResumeInspection::Response::SharedPtr res);
    void get_status(
        const inspection_interface::srv::GetInspectionStatus::Request::SharedPtr req,
        inspection_interface::srv::GetInspectionStatus::Response::SharedPtr res);

    // 辅助函数
    void set_phase(uint8_t phase);
    float calculate_progress();
    void publish_state();
    std::string get_current_action_string();
    bool check_timeout(const std::string& step_name, double timeout_sec);
    bool load_stations(const std::string& yaml_path);

    // 深度图处理
    double get_center_depth();

    // 站位配置
    std::vector<StationConfig> _stations;

    // 状态机
    uint8_t _current_phase;
    uint8_t _previous_phase;
    int _current_station_index;
    int _total_stations;

    // 跨阶段状态标志
    bool _agv_arrived;
    bool _arm_preset_done;
    bool _arm_preset_triggered;
    bool _depth_adjust_triggered;
    int _depth_adjust_retries;
    bool _detection_done;

    // AGV/Arm 最新状态
    inspection_interface::msg::AgvStatus _last_agv_status;
    inspection_interface::msg::ArmStatus _last_arm_status;
    bool _has_agv_status{false};
    bool _has_arm_status{false};

    // 深度图缓存
    sensor_msgs::msg::Image::SharedPtr _latest_depth;
    std::mutex _depth_mutex;

    // 超时参数
    double _agv_timeout_sec;
    double _arm_timeout_sec;
    double _detection_timeout_sec;
    double _depth_adjust_timeout_sec;
    int _max_depth_retries;

    // 超时检查
    rclcpp::Time _step_start_time;
    std::string _current_step_name;
    std::string _error_message;

    // Publishers
    rclcpp::Publisher<SystemState>::SharedPtr _state_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _agv_goal_pub;

    // Subscriptions
    rclcpp::Subscription<inspection_interface::msg::AgvStatus>::SharedPtr _agv_status_sub;
    rclcpp::Subscription<inspection_interface::msg::ArmStatus>::SharedPtr _arm_status_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _depth_sub;

    // Service clients
    rclcpp::Client<inspection_interface::srv::MoveToJoints>::SharedPtr _move_to_joints_client;
    rclcpp::Client<inspection_interface::srv::MoveToPose>::SharedPtr _move_to_pose_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _defect_detect_client;

    // Service servers
    rclcpp::Service<inspection_interface::srv::StartInspection>::SharedPtr _start_srv;
    rclcpp::Service<inspection_interface::srv::StopInspection>::SharedPtr _stop_srv;
    rclcpp::Service<inspection_interface::srv::PauseInspection>::SharedPtr _pause_srv;
    rclcpp::Service<inspection_interface::srv::ResumeInspection>::SharedPtr _resume_srv;
    rclcpp::Service<inspection_interface::srv::GetInspectionStatus>::SharedPtr _status_srv;

    // Timers
    rclcpp::TimerBase::SharedPtr _state_machine_timer;
    rclcpp::TimerBase::SharedPtr _state_publish_timer;
};

}  // namespace task_coordinator

#endif  // TASK_COORDINATOR_NODE_HPP
