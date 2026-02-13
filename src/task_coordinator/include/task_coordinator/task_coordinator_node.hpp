#ifndef TASK_COORDINATOR_NODE_HPP
#define TASK_COORDINATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <inspection_interface/msg/system_state.hpp>
#include <inspection_interface/srv/start_inspection.hpp>
#include <inspection_interface/srv/stop_inspection.hpp>
#include <inspection_interface/srv/pause_inspection.hpp>
#include <inspection_interface/srv/resume_inspection.hpp>
#include <inspection_interface/srv/get_inspection_status.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace task_coordinator {

using inspection_interface::msg::SystemState;

class TaskCoordinatorNode : public rclcpp::Node {
public:
    explicit TaskCoordinatorNode(const rclcpp::NodeOptions& options);

private:
    // 状态机
    void run_state_machine();
    void handle_localizing();
    void handle_planning();
    void handle_executing();
    void execute_current_waypoint();

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

    // 成员变量
    uint8_t _current_phase;
    uint8_t _previous_phase;
    int _current_waypoint_index;
    int _total_waypoints;
    int _execution_step;
    bool _last_step_done;

    geometry_msgs::msg::PoseStamped _last_agv_pose;
    sensor_msgs::msg::JointState _last_arm_joints;
    geometry_msgs::msg::PoseStamped _detected_pose;

    bool _pose_detected;
    bool _path_planned;
    bool _agv_arrived;
    bool _arm_arrived;
    bool _detection_done;
    bool _localizing_triggered;
    bool _planning_triggered;

    double _agv_timeout_sec;
    double _arm_timeout_sec;
    double _detection_timeout_sec;

    rclcpp::Publisher<SystemState>::SharedPtr _state_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _agv_goal_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm_goal_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _arm_joint_pub;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _agv_status_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm_status_sub;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _pose_detect_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _plan_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _defect_detect_client;

    rclcpp::Service<inspection_interface::srv::StartInspection>::SharedPtr _start_srv;
    rclcpp::Service<inspection_interface::srv::StopInspection>::SharedPtr _stop_srv;
    rclcpp::Service<inspection_interface::srv::PauseInspection>::SharedPtr _pause_srv;
    rclcpp::Service<inspection_interface::srv::ResumeInspection>::SharedPtr _resume_srv;
    rclcpp::Service<inspection_interface::srv::GetInspectionStatus>::SharedPtr _status_srv;

    rclcpp::TimerBase::SharedPtr _state_machine_timer;
    rclcpp::TimerBase::SharedPtr _state_publish_timer;
};

}  // namespace task_coordinator

#endif  // TASK_COORDINATOR_NODE_HPP
