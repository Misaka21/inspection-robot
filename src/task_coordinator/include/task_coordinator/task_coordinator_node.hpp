#ifndef TASK_COORDINATOR_NODE_HPP
#define TASK_COORDINATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <inspection_interface/msg/agv_status.hpp>
#include <inspection_interface/msg/arm_status.hpp>
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
    // _current_phase：当前状态机阶段，取值来自 SystemState 的 phase 常量
    //   （IDLE/LOCALIZING/PLANNING/EXECUTING/PAUSED/ERROR 等）
    uint8_t _current_phase;
    // _previous_phase：记录上一个阶段，用于 pause/resume 时恢复到正确状态
    uint8_t _previous_phase;
    // _current_waypoint_index：当前正在执行的站位点序号（从 0 开始）
    int _current_waypoint_index;
    int _total_waypoints;   // 总站位点数量，用于计算进度百分比
    // _execution_step：当前站位点内的执行步骤序号（AGV移动/等待/机械臂移动/拍照等）
    int _execution_step;
    bool _last_step_done;   // 当前步骤的异步操作是否完成（避免重复触发）

    inspection_interface::msg::AgvStatus _last_agv_status;
    inspection_interface::msg::ArmStatus _last_arm_status;
    // _has_agv_status/_has_arm_status：防止在驱动上线前使用默认值的守卫标志
    // 状态机执行前必须检查这两个标志，避免以"未初始化"状态做联锁判断
    bool _has_agv_status{false};
    bool _has_arm_status{false};

    std::vector<geometry_msgs::msg::Pose> _waypoints;
    std::string _waypoints_frame_id{"map"};
    geometry_msgs::msg::PoseStamped _detected_pose;

    // 以下布尔标志是状态机的跨函数共享条件，统一放这里而非散落 if/else 里
    bool _pose_detected;       // pose_detector 已返回有效检测结果
    bool _path_planned;        // path_planner 已返回规划后的 waypoints
    bool _agv_arrived;         // AGV arrived 标志（来自 agv/status.arrived）
    bool _arm_arrived;         // 机械臂 arrived 标志（来自 arm/status.arrived）
    bool _detection_done;      // 缺陷检测已完成（当前站位点的 capture+detect 流程结束）
    bool _localizing_triggered; // 已触发定位（防止重复触发）
    bool _planning_triggered;   // 已触发规划（防止重复触发）

    // 超时参数：各步骤的最大等待时间，超出后状态机进入 ERROR 状态或重试
    double _agv_timeout_sec;       // AGV 到位等待超时（秒）
    double _arm_timeout_sec;       // 机械臂到位等待超时（秒）
    double _detection_timeout_sec; // 缺陷检测完成等待超时（秒）

    rclcpp::Publisher<SystemState>::SharedPtr _state_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _agv_goal_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm_goal_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _arm_joint_pub;

    rclcpp::Subscription<inspection_interface::msg::AgvStatus>::SharedPtr _agv_status_sub;
    rclcpp::Subscription<inspection_interface::msg::ArmStatus>::SharedPtr _arm_status_sub;
    // _waypoints_sub：接收 path_planner 规划结果（PoseArray 形式的站位序列）
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr _waypoints_sub;

    // 以下 client 用于触发各算法模块的一次性计算：
    // _pose_detect_client：触发 pose_detector 做一次 6D 位姿估计
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _pose_detect_client;
    // _plan_client：触发 path_planner 用当前位姿做一次路径规划
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _plan_client;
    // _defect_detect_client：触发 defect_detector 对当前图像做一次缺陷检测
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _defect_detect_client;

    rclcpp::Service<inspection_interface::srv::StartInspection>::SharedPtr _start_srv;
    rclcpp::Service<inspection_interface::srv::StopInspection>::SharedPtr _stop_srv;
    rclcpp::Service<inspection_interface::srv::PauseInspection>::SharedPtr _pause_srv;
    rclcpp::Service<inspection_interface::srv::ResumeInspection>::SharedPtr _resume_srv;
    rclcpp::Service<inspection_interface::srv::GetInspectionStatus>::SharedPtr _status_srv;

    // _state_machine_timer：以固定频率推进状态机（run_state_machine），是控制核心
    // _state_publish_timer：以较低频率发布 SystemState 快照，与状态机 timer 分离
    //   分离的原因：状态发布频率不需要与状态机推进频率一致，可以独立配置
    rclcpp::TimerBase::SharedPtr _state_machine_timer;
    rclcpp::TimerBase::SharedPtr _state_publish_timer;
};

}  // namespace task_coordinator

#endif  // TASK_COORDINATOR_NODE_HPP
