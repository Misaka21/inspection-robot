#include <task_coordinator/task_coordinator_node.hpp>

namespace task_coordinator {

TaskCoordinatorNode::TaskCoordinatorNode(const rclcpp::NodeOptions& options)
    : Node("task_coordinator_node", options),
      _current_phase(SystemState::PHASE_IDLE),
      _previous_phase(SystemState::PHASE_IDLE),
      _current_waypoint_index(0),
      _total_waypoints(0),
      _execution_step(1),
      _last_step_done(false),
      _pose_detected(false),
      _path_planned(false),
      _agv_arrived(false),
      _arm_arrived(false),
      _detection_done(false),
      _localizing_triggered(false),
      _planning_triggered(false),
      _step_start_time(this->now()),
      _current_step_name("") {
    RCLCPP_INFO(this->get_logger(), "Starting Task Coordinator Node");

    this->declare_parameter("agv_timeout_sec", 30.0);
    this->declare_parameter("arm_timeout_sec", 30.0);
    this->declare_parameter("detection_timeout_sec", 10.0);
    this->get_parameter("agv_timeout_sec", _agv_timeout_sec);
    this->get_parameter("arm_timeout_sec", _arm_timeout_sec);
    this->get_parameter("detection_timeout_sec", _detection_timeout_sec);

    // Public ROS API: keep topic/service names stable (do not depend on node name).
    // This node is expected to run in namespace "/inspection".
    _state_pub = this->create_publisher<SystemState>("state", 10);
    _agv_goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "agv/goal_pose", 10);
    _arm_goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "arm_control/cart_goal", 10);
    _arm_joint_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        "arm_control/joint_goal", 10);

    // AGV 状态订阅：解析 arrived/stopped/connected/error_code 判断"到位"条件
    // 联锁：connected && arrived && stopped && error_code=="OK" 四条件全满足才算到位
    _agv_status_sub = this->create_subscription<inspection_interface::msg::AgvStatus>(
        "agv/status", 10,
        [this](const inspection_interface::msg::AgvStatus::SharedPtr msg) {
            _last_agv_status = *msg;
            _has_agv_status = true;
            _agv_arrived = msg->connected && msg->arrived && msg->stopped && (msg->error_code == "OK");
        });

    _arm_status_sub = this->create_subscription<inspection_interface::msg::ArmStatus>(
        "arm/status", 10,
        [this](const inspection_interface::msg::ArmStatus::SharedPtr msg) {
            _last_arm_status = *msg;
            _has_arm_status = true;
            _arm_arrived = msg->connected && msg->arrived && (msg->error_code.empty() || msg->error_code == "OK");
        });

    _waypoints_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "planning/path", 10,
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            _waypoints = msg->poses;
            _total_waypoints = static_cast<int>(_waypoints.size());
            _waypoints_frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
            RCLCPP_INFO(this->get_logger(), "收到路径点(PoseArray)，数量: %d frame_id=%s", _total_waypoints, _waypoints_frame_id.c_str());
        });

    // 订阅详细路径信息（包含机械臂关节角）
    _path_detail_sub = this->create_subscription<inspection_interface::msg::InspectionPath>(
        "planning/path_detail", 10,
        [this](const inspection_interface::msg::InspectionPath::SharedPtr msg) {
            _inspection_waypoints = msg->waypoints;
            RCLCPP_INFO(this->get_logger(), "收到详细路径(InspectionPath)，waypoints: %zu", msg->waypoints.size());
        });

    // 订阅检测到的工件位姿
    _detected_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "perception/detected_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            _detected_workpiece_pose = msg->pose;
            _has_detected_pose = true;
            RCLCPP_INFO(this->get_logger(), "收到工件位姿: [%.3f, %.3f, %.3f]",
                        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        });

    _pose_detect_client = this->create_client<std_srvs::srv::Trigger>(
        "perception/detect");
    _plan_client = this->create_client<std_srvs::srv::Trigger>(
        "planning/optimize");
    _defect_detect_client = this->create_client<std_srvs::srv::Trigger>(
        "perception/detect_defect");

    // 发布检测点给 path_planner
    _detection_points_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "planning/detection_points", 10);

    _start_srv = this->create_service<inspection_interface::srv::StartInspection>(
        "start",
        [this](const inspection_interface::srv::StartInspection::Request::SharedPtr req,
               inspection_interface::srv::StartInspection::Response::SharedPtr res) {
            start_inspection(req, res);
        });

    _stop_srv = this->create_service<inspection_interface::srv::StopInspection>(
        "stop",
        [this](const inspection_interface::srv::StopInspection::Request::SharedPtr req,
               inspection_interface::srv::StopInspection::Response::SharedPtr res) {
            stop_inspection(req, res);
        });

    _pause_srv = this->create_service<inspection_interface::srv::PauseInspection>(
        "pause",
        [this](const inspection_interface::srv::PauseInspection::Request::SharedPtr req,
               inspection_interface::srv::PauseInspection::Response::SharedPtr res) {
            pause_inspection(req, res);
        });

    _resume_srv = this->create_service<inspection_interface::srv::ResumeInspection>(
        "resume",
        [this](const inspection_interface::srv::ResumeInspection::Request::SharedPtr req,
               inspection_interface::srv::ResumeInspection::Response::SharedPtr res) {
            resume_inspection(req, res);
        });

    _status_srv = this->create_service<inspection_interface::srv::GetInspectionStatus>(
        "get_status",
        [this](const inspection_interface::srv::GetInspectionStatus::Request::SharedPtr req,
               inspection_interface::srv::GetInspectionStatus::Response::SharedPtr res) {
            get_status(req, res);
        });

    // 状态机定时器（100ms）：每 100ms 驱动一次状态机推进
    _state_machine_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { run_state_machine(); });

    // 状态发布定时器（1s）：定期向 ROS 发布 SystemState 供网关/HMI 订阅
    _state_publish_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { publish_state(); });

    RCLCPP_INFO(this->get_logger(), "Task Coordinator initialized");
}

void TaskCoordinatorNode::run_state_machine() {
    switch (_current_phase) {
        case SystemState::PHASE_IDLE:
            break;
        case SystemState::PHASE_LOCALIZING:
            handle_localizing();
            break;
        case SystemState::PHASE_PLANNING:
            handle_planning();
            break;
        case SystemState::PHASE_EXECUTING:
            handle_executing();
            break;
        case SystemState::PHASE_PAUSED:
        case SystemState::PHASE_COMPLETED:
        case SystemState::PHASE_FAILED:
        case SystemState::PHASE_STOPPED:
            break;
    }
}

// LOCALIZING 阶段：调用 perception/detect 服务触发工件位姿检测。
// _localizing_triggered 防止重复发送服务请求（异步回调尚未返回时）。
// 检测成功后自动进入 PLANNING 阶段。
void TaskCoordinatorNode::handle_localizing() {
    if (!_localizing_triggered) {
        RCLCPP_INFO(this->get_logger(), "触发工件位姿检测...");
        if (_pose_detect_client->wait_for_service(std::chrono::seconds(1))) {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto callback = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    _pose_detected = true;
                    RCLCPP_INFO(this->get_logger(), "位姿检测服务返回成功");
                } else {
                    RCLCPP_WARN(this->get_logger(), "位姿检测服务返回失败: %s", response->message.c_str());
                }
            };
            _pose_detect_client->async_send_request(req, callback);
            _localizing_triggered = true;
        }
    }

    if (_pose_detected) {
        RCLCPP_INFO(this->get_logger(), "位姿检测完成，进入规划阶段");
        set_phase(SystemState::PHASE_PLANNING);
        _pose_detected = false;
        _localizing_triggered = false;
    }
}

// PLANNING 阶段：调用 planning/optimize 服务触发路径规划。
// 规划结果通过 /inspection/planning/path 话题回调更新 _waypoints。
// 必须 _path_planned=true 且 _total_waypoints>0 才推进到 EXECUTING。
void TaskCoordinatorNode::handle_planning() {
    if (!_planning_triggered) {
        RCLCPP_INFO(this->get_logger(), "触发路径规划...");

        // 先发布检测点给 path_planner
        if (_has_detected_pose) {
            geometry_msgs::msg::PoseArray detection_points;
            detection_points.header.stamp = this->now();
            detection_points.header.frame_id = "map";
            detection_points.poses.push_back(_detected_workpiece_pose);
            _detection_points_pub->publish(detection_points);
            RCLCPP_INFO(this->get_logger(), "发布检测点给 path_planner");
        } else {
            RCLCPP_WARN(this->get_logger(), "无检测点位姿，将发布空检测点");
            geometry_msgs::msg::PoseArray detection_points;
            detection_points.header.stamp = this->now();
            detection_points.header.frame_id = "map";
            _detection_points_pub->publish(detection_points);
        }

        if (_plan_client->wait_for_service(std::chrono::seconds(1))) {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto callback = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    _path_planned = true;
                    RCLCPP_INFO(this->get_logger(), "路径规划服务返回成功");
                } else {
                    RCLCPP_WARN(this->get_logger(), "路径规划服务返回失败: %s", response->message.c_str());
                }
            };
            _plan_client->async_send_request(req, callback);
            _planning_triggered = true;
        }
    }

    if (_path_planned && _total_waypoints > 0) {
        RCLCPP_INFO(this->get_logger(), "路径规划完成，开始执行，共 %d 个路径点", _total_waypoints);
        _current_waypoint_index = 0;
        set_phase(SystemState::PHASE_EXECUTING);
        _path_planned = false;
        _planning_triggered = false;
    }
}

void TaskCoordinatorNode::handle_executing() {
    if (_current_waypoint_index >= _total_waypoints) {
        RCLCPP_INFO(this->get_logger(), "所有检测点执行完成");
        set_phase(SystemState::PHASE_COMPLETED);
        return;
    }
    execute_current_waypoint();
}

void TaskCoordinatorNode::execute_current_waypoint() {
    switch (_execution_step) {
        case 1:
            RCLCPP_INFO(
                this->get_logger(),
                "步骤1: 下发 AGV 目标 (waypoint %d/%d)",
                _current_waypoint_index + 1,
                _total_waypoints);
            _agv_arrived = false;
            if (_current_waypoint_index >= 0 &&
                _current_waypoint_index < static_cast<int>(_waypoints.size())) {
                geometry_msgs::msg::PoseStamped goal;
                goal.header.stamp = this->now();
                goal.header.frame_id = _waypoints_frame_id;
                goal.pose = _waypoints[static_cast<size_t>(_current_waypoint_index)];
                _agv_goal_pub->publish(goal);
            } else {
                RCLCPP_WARN(this->get_logger(), "当前 waypoint pose 不可用，跳过下发");
            }
            _step_start_time = this->now();
            _current_step_name = "AGV移动";
            _execution_step = 2;
            break;
        case 2:
            if (_agv_arrived) {
                RCLCPP_INFO(this->get_logger(), "AGV 已到位");
                _execution_step = 3;
                _agv_arrived = false;
            } else if (check_timeout("AGV移动", _agv_timeout_sec)) {
                RCLCPP_ERROR(this->get_logger(), "AGV 移动超时");
                set_phase(SystemState::PHASE_FAILED);
                return;
            }
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "步骤3: 下发机械臂目标");
            _arm_arrived = false;
            if (_current_waypoint_index >= 0 &&
                _current_waypoint_index < static_cast<int>(_inspection_waypoints.size())) {
                const auto& wp = _inspection_waypoints[_current_waypoint_index];
                if (!wp.joint_angles.empty()) {
                    sensor_msgs::msg::JointState joint_goal;
                    joint_goal.header.stamp = this->now();
                    // 使用标准关节名称（应与 robot_description 一致）
                    for (size_t i = 0; i < wp.joint_angles.size(); ++i) {
                        joint_goal.name.push_back("joint_" + std::to_string(i + 1));
                        joint_goal.position.push_back(wp.joint_angles[i]);
                    }
                    _arm_joint_pub->publish(joint_goal);
                    RCLCPP_INFO(this->get_logger(), "下发关节角目标，%zu 个关节", wp.joint_angles.size());
                } else {
                    RCLCPP_WARN(this->get_logger(), "当前 waypoint 无关节角数据，跳过下发");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "当前 waypoint 索引无效，跳过下发");
            }
            _step_start_time = this->now();
            _current_step_name = "机械臂移动";
            _execution_step = 4;
            break;
        case 4:
            if (_arm_arrived) {
                RCLCPP_INFO(this->get_logger(), "机械臂已到位");
                _execution_step = 5;
                _arm_arrived = false;
            } else if (check_timeout("机械臂移动", _arm_timeout_sec)) {
                RCLCPP_ERROR(this->get_logger(), "机械臂移动超时");
                set_phase(SystemState::PHASE_FAILED);
                return;
            }
            break;
        case 5:
            RCLCPP_INFO(this->get_logger(), "步骤5: 触发缺陷检测");
            if (_defect_detect_client->wait_for_service(std::chrono::seconds(1))) {
                auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto callback = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success) {
                        _detection_done = true;
                        RCLCPP_INFO(this->get_logger(), "缺陷检测服务返回成功");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "缺陷检测服务返回失败: %s", response->message.c_str());
                    }
                };
                _defect_detect_client->async_send_request(req, callback);
            }
            _step_start_time = this->now();
            _current_step_name = "缺陷检测";
            _execution_step = 6;
            break;
        case 6:
            if (_detection_done) {
                RCLCPP_INFO(this->get_logger(), "检测完成，进入下一路径点");
                _current_waypoint_index++;
                _detection_done = false;
                _execution_step = 1;
            } else if (check_timeout("缺陷检测", _detection_timeout_sec)) {
                RCLCPP_ERROR(this->get_logger(), "缺陷检测超时");
                set_phase(SystemState::PHASE_FAILED);
                return;
            }
            break;
    }
}

void TaskCoordinatorNode::start_inspection(
    const inspection_interface::srv::StartInspection::Request::SharedPtr req,
    inspection_interface::srv::StartInspection::Response::SharedPtr res) {
    (void)req;

    if (_current_phase != SystemState::PHASE_IDLE &&
        _current_phase != SystemState::PHASE_COMPLETED &&
        _current_phase != SystemState::PHASE_FAILED &&
        _current_phase != SystemState::PHASE_STOPPED) {
        res->success = false;
        res->message = "当前状态不允许启动";
        return;
    }

    _current_waypoint_index = 0;
    _total_waypoints = 0;
    _pose_detected = false;
    _path_planned = false;
    _agv_arrived = false;
    _arm_arrived = false;
    _detection_done = false;
    _localizing_triggered = false;
    _planning_triggered = false;
    _execution_step = 1;
    _last_step_done = false;
    _waypoints.clear();
    _waypoints_frame_id = "map";
    _inspection_waypoints.clear();

    set_phase(SystemState::PHASE_LOCALIZING);
    res->success = true;
    res->message = "检测任务已启动";
    RCLCPP_INFO(this->get_logger(), "检测任务已启动");
}

void TaskCoordinatorNode::stop_inspection(
    const inspection_interface::srv::StopInspection::Request::SharedPtr req,
    inspection_interface::srv::StopInspection::Response::SharedPtr res) {
    (void)req;
    set_phase(SystemState::PHASE_STOPPED);
    res->success = true;
    res->message = "检测任务已停止";
    RCLCPP_INFO(this->get_logger(), "检测任务已停止");
}

void TaskCoordinatorNode::pause_inspection(
    const inspection_interface::srv::PauseInspection::Request::SharedPtr req,
    inspection_interface::srv::PauseInspection::Response::SharedPtr res) {
    (void)req;

    if (_current_phase != SystemState::PHASE_EXECUTING) {
        res->success = false;
        res->message = "只有执行阶段才能暂停";
        return;
    }

    set_phase(SystemState::PHASE_PAUSED);
    res->success = true;
    res->message = "检测任务已暂停";
    RCLCPP_INFO(this->get_logger(), "检测任务已暂停");
}

void TaskCoordinatorNode::resume_inspection(
    const inspection_interface::srv::ResumeInspection::Request::SharedPtr req,
    inspection_interface::srv::ResumeInspection::Response::SharedPtr res) {
    (void)req;

    if (_current_phase != SystemState::PHASE_PAUSED) {
        res->success = false;
        res->message = "只有暂停状态才能恢复";
        return;
    }

    set_phase(SystemState::PHASE_EXECUTING);
    res->success = true;
    res->message = "检测任务已恢复";
    RCLCPP_INFO(this->get_logger(), "检测任务已恢复");
}

void TaskCoordinatorNode::get_status(
    const inspection_interface::srv::GetInspectionStatus::Request::SharedPtr req,
    inspection_interface::srv::GetInspectionStatus::Response::SharedPtr res) {
    (void)req;
    res->success = true;
    res->message = "ok";
    res->status = get_current_action_string();
    res->progress = calculate_progress();
    res->state.header.stamp = this->now();
    res->state.phase = _current_phase;
    res->state.progress_percent = calculate_progress();
    res->state.current_action = get_current_action_string();
    if (_has_agv_status) {
        res->state.agv = _last_agv_status;
    }
    if (_has_arm_status) {
        res->state.arm = _last_arm_status;
    }
}

void TaskCoordinatorNode::set_phase(uint8_t phase) {
    RCLCPP_INFO(this->get_logger(), "状态切换: %d -> %d", _current_phase, phase);
    _previous_phase = _current_phase;
    _current_phase = phase;
}

// 进度计算：百分比表示任务完成度。
// LOCALIZING=10%, PLANNING=20%, EXECUTING 按 waypoint 进度线性插值到 80%。
float TaskCoordinatorNode::calculate_progress() {
    if (_current_phase == SystemState::PHASE_IDLE ||
        _current_phase == SystemState::PHASE_STOPPED ||
        _current_phase == SystemState::PHASE_FAILED) {
        return 0.0f;
    }
    if (_current_phase == SystemState::PHASE_COMPLETED) {
        return 100.0f;
    }
    if (_total_waypoints == 0) {
        return 0.0f;
    }
    float phase_progress = 0.0f;
    if (_current_phase == SystemState::PHASE_LOCALIZING) {
        phase_progress = 10.0f;
    } else if (_current_phase == SystemState::PHASE_PLANNING) {
        phase_progress = 20.0f;
    } else if (_current_phase == SystemState::PHASE_EXECUTING) {
        phase_progress = 20.0f + (_current_waypoint_index * 60.0f) / _total_waypoints;
    }
    return phase_progress;
}

void TaskCoordinatorNode::publish_state() {
    SystemState state;
    state.header.stamp = this->now();
    state.phase = _current_phase;
    state.progress_percent = calculate_progress();
    state.current_action = get_current_action_string();
    if (_current_phase == SystemState::PHASE_FAILED) {
        state.error_message = _error_message;
    }
    if (_has_agv_status) {
        state.agv = _last_agv_status;
    }
    if (_has_arm_status) {
        state.arm = _last_arm_status;
    }
    _state_pub->publish(state);
}

std::string TaskCoordinatorNode::get_current_action_string() {
    switch (_current_phase) {
        case SystemState::PHASE_IDLE:
            return "等待任务";
        case SystemState::PHASE_LOCALIZING:
            return "检测工件位姿";
        case SystemState::PHASE_PLANNING:
            return "规划检测路径";
        case SystemState::PHASE_EXECUTING:
            return "执行检测: " + std::to_string(_current_waypoint_index + 1) +
                   "/" + std::to_string(_total_waypoints) + " (" + _current_step_name + ")";
        case SystemState::PHASE_PAUSED:
            return "任务已暂停";
        case SystemState::PHASE_COMPLETED:
            return "检测完成";
        case SystemState::PHASE_FAILED:
            return "检测失败: " + _error_message;
        case SystemState::PHASE_STOPPED:
            return "任务已停止";
        default:
            return "未知状态";
    }
}

// 检查是否超时，超时返回 true 并设置错误信息
bool TaskCoordinatorNode::check_timeout(const std::string& step_name, double timeout_sec) {
    if (timeout_sec <= 0.0) {
        return false;  // 超时未启用
    }
    auto elapsed = (this->now() - _step_start_time).seconds();
    if (elapsed > timeout_sec) {
        _error_message = step_name + "超时(" + std::to_string(static_cast<int>(elapsed)) + "s)";
        return true;
    }
    return false;
}

}  // namespace task_coordinator

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto coordinator = std::make_shared<task_coordinator::TaskCoordinatorNode>(
        rclcpp::NodeOptions());
    rclcpp::spin(coordinator);
    rclcpp::shutdown();
    return 0;
}
