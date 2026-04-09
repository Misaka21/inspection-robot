#include <task_coordinator/task_coordinator_node.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace task_coordinator {

TaskCoordinatorNode::TaskCoordinatorNode(const rclcpp::NodeOptions& options)
    : Node("task_coordinator_node", options),
      _current_phase(SystemState::PHASE_IDLE),
      _previous_phase(SystemState::PHASE_IDLE),
      _current_station_index(0),
      _total_stations(0),
      _agv_arrived(false),
      _arm_preset_done(false),
      _arm_preset_triggered(false),
      _depth_adjust_triggered(false),
      _depth_adjust_retries(0),
      _detection_done(false),
      _step_start_time(this->now()) {
    RCLCPP_INFO(this->get_logger(), "Starting Task Coordinator Node");

    // 参数声明
    this->declare_parameter("agv_timeout_sec", 60.0);
    this->declare_parameter("arm_timeout_sec", 30.0);
    this->declare_parameter("detection_timeout_sec", 10.0);
    this->declare_parameter("depth_adjust_timeout_sec", 15.0);
    this->declare_parameter("max_depth_retries", 3);
    this->declare_parameter("stations_file", "");

    this->get_parameter("agv_timeout_sec", _agv_timeout_sec);
    this->get_parameter("arm_timeout_sec", _arm_timeout_sec);
    this->get_parameter("detection_timeout_sec", _detection_timeout_sec);
    this->get_parameter("depth_adjust_timeout_sec", _depth_adjust_timeout_sec);
    this->get_parameter("max_depth_retries", _max_depth_retries);

    // 加载站位配置
    std::string stations_file;
    this->get_parameter("stations_file", stations_file);
    if (!stations_file.empty()) {
        if (!load_stations(stations_file)) {
            RCLCPP_ERROR(this->get_logger(), "加载站位配置失败: %s", stations_file.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "未指定 stations_file 参数，需通过参数加载站位配置");
    }

    // Publishers
    _state_pub = this->create_publisher<SystemState>("state", 10);
    _agv_goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "agv/goal_pose", 10);

    // Subscriptions
    _agv_status_sub = this->create_subscription<inspection_interface::msg::AgvStatus>(
        "agv/status", 10,
        [this](const inspection_interface::msg::AgvStatus::SharedPtr msg) {
            _last_agv_status = *msg;
            _has_agv_status = true;
            _agv_arrived = msg->connected && msg->arrived && msg->stopped &&
                           (msg->error_code == "OK");
        });

    _arm_status_sub = this->create_subscription<inspection_interface::msg::ArmStatus>(
        "arm/status", 10,
        [this](const inspection_interface::msg::ArmStatus::SharedPtr msg) {
            _last_arm_status = *msg;
            _has_arm_status = true;
        });

    // 深度图订阅（RealSense aligned depth）
    _depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/inspection/realsense/d435/aligned_depth_to_color/image_raw", 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(_depth_mutex);
            _latest_depth = msg;
        });

    // Service clients
    _move_to_joints_client = this->create_client<inspection_interface::srv::MoveToJoints>(
        "arm_control/move_to_joints");
    _move_to_pose_client = this->create_client<inspection_interface::srv::MoveToPose>(
        "arm_control/move_to_pose");
    _defect_detect_client = this->create_client<std_srvs::srv::Trigger>(
        "perception/detect_defect");

    // Service servers
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

    // 状态机定时器（100ms）
    _state_machine_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { run_state_machine(); });

    // 状态发布定时器（1s）
    _state_publish_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { publish_state(); });

    RCLCPP_INFO(this->get_logger(), "Task Coordinator initialized, %d stations loaded",
                static_cast<int>(_stations.size()));
}

bool TaskCoordinatorNode::load_stations(const std::string& yaml_path) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (!config["stations"]) {
            RCLCPP_ERROR(this->get_logger(), "YAML 缺少 'stations' 字段");
            return false;
        }

        _stations.clear();
        for (const auto& s : config["stations"]) {
            StationConfig sc;
            sc.name = s["name"].as<std::string>("unnamed");

            if (s["agv_pose"]) {
                sc.agv_x = s["agv_pose"]["x"].as<double>(0.0);
                sc.agv_y = s["agv_pose"]["y"].as<double>(0.0);
                sc.agv_z = s["agv_pose"]["z"].as<double>(0.0);
                sc.agv_yaw = s["agv_pose"]["yaw"].as<double>(0.0);
            }

            if (s["arm_joints"]) {
                for (const auto& j : s["arm_joints"]) {
                    sc.arm_joints.push_back(j.as<double>());
                }
            }

            sc.target_distance = s["target_distance"].as<double>(0.30);
            sc.distance_tolerance = s["distance_tolerance"].as<double>(0.02);
            sc.adjust_axis = s["adjust_axis"].as<std::string>("z");

            _stations.push_back(sc);
            RCLCPP_INFO(this->get_logger(), "加载站位: %s (agv=[%.2f,%.2f] joints=%zu)",
                        sc.name.c_str(), sc.agv_x, sc.agv_y, sc.arm_joints.size());
        }

        _total_stations = static_cast<int>(_stations.size());
        return _total_stations > 0;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YAML 解析错误: %s", e.what());
        return false;
    }
}

// ─── 状态机 ───────────────────────────────────────────────

void TaskCoordinatorNode::run_state_machine() {
    switch (_current_phase) {
        case SystemState::PHASE_IDLE:
        case SystemState::PHASE_COMPLETED:
        case SystemState::PHASE_FAILED:
        case SystemState::PHASE_STOPPED:
        case SystemState::PHASE_PAUSED:
            break;
        case SystemState::PHASE_MOVING_TO_STATION:
            handle_moving_to_station();
            break;
        case SystemState::PHASE_ARM_PRESET:
            handle_arm_preset();
            break;
        case SystemState::PHASE_DEPTH_ADJUST:
            handle_depth_adjust();
            break;
        case SystemState::PHASE_CAPTURING:
            handle_capturing();
            break;
    }
}

// MOVING_TO_STATION：发送 AGV 目标位姿，等待 arrived
void TaskCoordinatorNode::handle_moving_to_station() {
    // 首次进入：发送目标
    if (!_agv_arrived && _current_step_name != "AGV移动") {
        const auto& st = _stations[_current_station_index];
        RCLCPP_INFO(this->get_logger(), "AGV 导航到站位 %s (%d/%d)",
                    st.name.c_str(), _current_station_index + 1, _total_stations);

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = st.agv_x;
        goal.pose.position.y = st.agv_y;
        goal.pose.position.z = st.agv_z;
        // yaw -> quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, st.agv_yaw);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();

        _agv_goal_pub->publish(goal);
        _agv_arrived = false;
        _step_start_time = this->now();
        _current_step_name = "AGV移动";
    }

    if (_agv_arrived) {
        RCLCPP_INFO(this->get_logger(), "AGV 已到达站位 %s",
                    _stations[_current_station_index].name.c_str());
        _agv_arrived = false;
        _current_step_name = "";
        set_phase(SystemState::PHASE_ARM_PRESET);
    } else if (check_timeout("AGV移动", _agv_timeout_sec)) {
        RCLCPP_ERROR(this->get_logger(), "AGV 移动超时");
        set_phase(SystemState::PHASE_FAILED);
    }
}

// ARM_PRESET：调用 MoveToJoints 服务送臂到预设位姿
void TaskCoordinatorNode::handle_arm_preset() {
    if (!_arm_preset_triggered) {
        const auto& st = _stations[_current_station_index];
        RCLCPP_INFO(this->get_logger(), "机械臂移动到预设位姿 (%zu joints)",
                    st.arm_joints.size());

        if (!_move_to_joints_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "MoveToJoints 服务不可用，等待中...");
            return;
        }

        auto req = std::make_shared<inspection_interface::srv::MoveToJoints::Request>();
        req->target_joints = st.arm_joints;
        req->speed = 0.5f;

        auto callback = [this](
            rclcpp::Client<inspection_interface::srv::MoveToJoints>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                _arm_preset_done = true;
                RCLCPP_INFO(this->get_logger(), "机械臂预设位姿到达");
            } else {
                RCLCPP_ERROR(this->get_logger(), "机械臂预设位姿失败: %s",
                             response->message.c_str());
                _error_message = "机械臂预设位姿失败: " + response->message;
                set_phase(SystemState::PHASE_FAILED);
            }
        };

        _move_to_joints_client->async_send_request(req, callback);
        _arm_preset_triggered = true;
        _step_start_time = this->now();
        _current_step_name = "机械臂预设";
    }

    if (_arm_preset_done) {
        _arm_preset_done = false;
        _arm_preset_triggered = false;
        _current_step_name = "";
        set_phase(SystemState::PHASE_DEPTH_ADJUST);
    } else if (check_timeout("机械臂预设", _arm_timeout_sec)) {
        RCLCPP_ERROR(this->get_logger(), "机械臂预设位姿超时");
        set_phase(SystemState::PHASE_FAILED);
    }
}

// DEPTH_ADJUST：读 RealSense 深度，算偏移，调 MoveToPose 微调
void TaskCoordinatorNode::handle_depth_adjust() {
    if (!_depth_adjust_triggered) {
        const auto& st = _stations[_current_station_index];
        double current_depth = get_center_depth();

        if (current_depth <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "深度数据无效，等待有效深度...");
            if (!_current_step_name.empty()) {
                // 已经在等待了，检查超时
                if (check_timeout("深度测距", _depth_adjust_timeout_sec)) {
                    RCLCPP_ERROR(this->get_logger(), "深度测距超时，无有效数据");
                    set_phase(SystemState::PHASE_FAILED);
                }
            } else {
                _step_start_time = this->now();
                _current_step_name = "深度测距";
            }
            return;
        }

        double delta = st.target_distance - current_depth;
        RCLCPP_INFO(this->get_logger(),
                    "深度测距: 当前=%.4fm 目标=%.4fm 偏差=%.4fm 容差=%.4fm",
                    current_depth, st.target_distance, delta, st.distance_tolerance);

        if (std::abs(delta) <= st.distance_tolerance) {
            RCLCPP_INFO(this->get_logger(), "深度在容差范围内，进入拍照阶段");
            _depth_adjust_retries = 0;
            _current_step_name = "";
            set_phase(SystemState::PHASE_CAPTURING);
            return;
        }

        if (_depth_adjust_retries >= _max_depth_retries) {
            RCLCPP_ERROR(this->get_logger(), "深度调整超过最大重试次数 %d", _max_depth_retries);
            _error_message = "深度调整失败: 超过最大重试次数";
            set_phase(SystemState::PHASE_FAILED);
            return;
        }

        // 调用 MoveToPose 沿 adjust_axis 方向偏移 delta
        if (!_move_to_pose_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "MoveToPose 服务不可用，等待中...");
            return;
        }

        // 获取当前臂末端位姿（从 arm_status）
        auto req = std::make_shared<inspection_interface::srv::MoveToPose::Request>();

        // 构建目标位姿：在当前位姿基础上沿工具坐标系 z 轴偏移
        // 注意：这里用工具坐标系 z 轴近似为相机光轴方向
        // 正 delta = 需要靠近（当前太远），负 delta = 需要远离（当前太近）
        geometry_msgs::msg::Pose target;
        target.position.x = 0.0;
        target.position.y = 0.0;
        target.position.z = delta;  // 相对偏移
        target.orientation.w = 1.0;
        req->target_pose = target;
        req->speed = 0.3f;

        RCLCPP_INFO(this->get_logger(), "发送深度微调: axis=%s delta=%.4fm (retry %d/%d)",
                    st.adjust_axis.c_str(), delta, _depth_adjust_retries + 1, _max_depth_retries);

        auto callback = [this](
            rclcpp::Client<inspection_interface::srv::MoveToPose>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "深度微调移动完成");
            } else {
                RCLCPP_WARN(this->get_logger(), "深度微调移动失败: %s",
                            response->message.c_str());
            }
            _depth_adjust_triggered = false;  // 允许下一次测量+调整
        };

        _move_to_pose_client->async_send_request(req, callback);
        _depth_adjust_triggered = true;
        _depth_adjust_retries++;
        _step_start_time = this->now();
        _current_step_name = "深度微调";
    }

    // 等待 MoveToPose 回调把 _depth_adjust_triggered 置回 false
    if (check_timeout("深度微调", _depth_adjust_timeout_sec)) {
        RCLCPP_ERROR(this->get_logger(), "深度微调超时");
        set_phase(SystemState::PHASE_FAILED);
    }
}

// CAPTURING：触发缺陷检测
void TaskCoordinatorNode::handle_capturing() {
    if (!_detection_done && _current_step_name != "缺陷检测") {
        RCLCPP_INFO(this->get_logger(), "触发缺陷检测 (站位 %d/%d)",
                    _current_station_index + 1, _total_stations);

        if (_defect_detect_client->wait_for_service(std::chrono::seconds(1))) {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto callback = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    _detection_done = true;
                    RCLCPP_INFO(this->get_logger(), "缺陷检测完成");
                } else {
                    RCLCPP_WARN(this->get_logger(), "缺陷检测失败: %s",
                                response->message.c_str());
                    _detection_done = true;  // 仍然继续到下一站
                }
            };
            _defect_detect_client->async_send_request(req, callback);
        } else {
            RCLCPP_WARN(this->get_logger(), "缺陷检测服务不可用，跳过");
            _detection_done = true;
        }
        _step_start_time = this->now();
        _current_step_name = "缺陷检测";
    }

    if (_detection_done) {
        _detection_done = false;
        _current_step_name = "";

        _current_station_index++;
        if (_current_station_index >= _total_stations) {
            RCLCPP_INFO(this->get_logger(), "所有站位巡检完成");
            set_phase(SystemState::PHASE_COMPLETED);
        } else {
            RCLCPP_INFO(this->get_logger(), "进入下一站位 %d/%d",
                        _current_station_index + 1, _total_stations);
            set_phase(SystemState::PHASE_MOVING_TO_STATION);
        }
    } else if (check_timeout("缺陷检测", _detection_timeout_sec)) {
        RCLCPP_ERROR(this->get_logger(), "缺陷检测超时");
        set_phase(SystemState::PHASE_FAILED);
    }
}

// ─── 深度图处理 ───────────────────────────────────────────

double TaskCoordinatorNode::get_center_depth() {
    std::lock_guard<std::mutex> lock(_depth_mutex);
    if (!_latest_depth) {
        return -1.0;
    }

    const auto& img = _latest_depth;

    // 支持 16UC1 (毫米) 和 32FC1 (米) 两种深度编码
    bool is_16uc1 = (img->encoding == "16UC1");
    bool is_32fc1 = (img->encoding == "32FC1");
    if (!is_16uc1 && !is_32fc1) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "不支持的深度编码: %s", img->encoding.c_str());
        return -1.0;
    }

    uint32_t w = img->width;
    uint32_t h = img->height;
    if (w == 0 || h == 0) return -1.0;

    // 取中心 ROI (20% x 20%)
    uint32_t roi_w = std::max(w / 5, 1u);
    uint32_t roi_h = std::max(h / 5, 1u);
    uint32_t cx = w / 2;
    uint32_t cy = h / 2;
    uint32_t x0 = cx - roi_w / 2;
    uint32_t y0 = cy - roi_h / 2;

    std::vector<double> depths;
    depths.reserve(roi_w * roi_h);

    for (uint32_t y = y0; y < y0 + roi_h && y < h; ++y) {
        for (uint32_t x = x0; x < x0 + roi_w && x < w; ++x) {
            double d = 0.0;
            if (is_16uc1) {
                uint16_t raw = *reinterpret_cast<const uint16_t*>(
                    &img->data[y * img->step + x * 2]);
                d = raw * 0.001;  // mm -> m
            } else {
                float raw = *reinterpret_cast<const float*>(
                    &img->data[y * img->step + x * 4]);
                d = static_cast<double>(raw);
            }
            if (d > 0.01 && d < 10.0) {  // 有效范围 1cm ~ 10m
                depths.push_back(d);
            }
        }
    }

    if (depths.empty()) return -1.0;

    // 取中值（比均值更抗噪）
    std::sort(depths.begin(), depths.end());
    return depths[depths.size() / 2];
}

// ─── 服务回调 ─────────────────────────────────────────────

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

    if (_stations.empty()) {
        res->success = false;
        res->message = "无站位配置，请先加载 stations_file";
        return;
    }

    // 重置所有状态
    _current_station_index = 0;
    _total_stations = static_cast<int>(_stations.size());
    _agv_arrived = false;
    _arm_preset_done = false;
    _arm_preset_triggered = false;
    _depth_adjust_triggered = false;
    _depth_adjust_retries = 0;
    _detection_done = false;
    _current_step_name = "";
    _error_message = "";

    set_phase(SystemState::PHASE_MOVING_TO_STATION);
    res->success = true;
    res->message = "巡检任务已启动，共 " + std::to_string(_total_stations) + " 个站位";
    RCLCPP_INFO(this->get_logger(), "巡检任务已启动，共 %d 个站位", _total_stations);
}

void TaskCoordinatorNode::stop_inspection(
    const inspection_interface::srv::StopInspection::Request::SharedPtr req,
    inspection_interface::srv::StopInspection::Response::SharedPtr res) {
    (void)req;
    set_phase(SystemState::PHASE_STOPPED);
    res->success = true;
    res->message = "巡检任务已停止";
    RCLCPP_INFO(this->get_logger(), "巡检任务已停止");
}

void TaskCoordinatorNode::pause_inspection(
    const inspection_interface::srv::PauseInspection::Request::SharedPtr req,
    inspection_interface::srv::PauseInspection::Response::SharedPtr res) {
    (void)req;

    // 只有在执行相关阶段才能暂停
    if (_current_phase != SystemState::PHASE_MOVING_TO_STATION &&
        _current_phase != SystemState::PHASE_ARM_PRESET &&
        _current_phase != SystemState::PHASE_DEPTH_ADJUST &&
        _current_phase != SystemState::PHASE_CAPTURING) {
        res->success = false;
        res->message = "当前状态不允许暂停";
        return;
    }

    set_phase(SystemState::PHASE_PAUSED);
    res->success = true;
    res->message = "巡检任务已暂停";
    RCLCPP_INFO(this->get_logger(), "巡检任务已暂停");
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

    // 恢复到暂停前的阶段
    set_phase(_previous_phase);
    // 重置步骤计时，避免恢复后立即超时
    _step_start_time = this->now();
    res->success = true;
    res->message = "巡检任务已恢复";
    RCLCPP_INFO(this->get_logger(), "巡检任务已恢复到阶段 %d", _previous_phase);
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
    if (_current_phase == SystemState::PHASE_FAILED) {
        res->state.error_message = _error_message;
    }
    if (_has_agv_status) {
        res->state.agv = _last_agv_status;
    }
    if (_has_arm_status) {
        res->state.arm = _last_arm_status;
    }
}

// ─── 辅助函数 ─────────────────────────────────────────────

void TaskCoordinatorNode::set_phase(uint8_t phase) {
    RCLCPP_INFO(this->get_logger(), "状态切换: %d -> %d", _current_phase, phase);
    _previous_phase = _current_phase;
    _current_phase = phase;
}

float TaskCoordinatorNode::calculate_progress() {
    if (_current_phase == SystemState::PHASE_IDLE ||
        _current_phase == SystemState::PHASE_STOPPED ||
        _current_phase == SystemState::PHASE_FAILED) {
        return 0.0f;
    }
    if (_current_phase == SystemState::PHASE_COMPLETED) {
        return 100.0f;
    }
    if (_total_stations == 0) {
        return 0.0f;
    }

    // 每个站位占比 = 100% / total_stations
    // 站位内4个子阶段各占 25%
    float station_pct = 100.0f / _total_stations;
    float base = _current_station_index * station_pct;

    float sub_pct = 0.0f;
    if (_current_phase == SystemState::PHASE_MOVING_TO_STATION) {
        sub_pct = 0.0f;
    } else if (_current_phase == SystemState::PHASE_ARM_PRESET) {
        sub_pct = 0.25f;
    } else if (_current_phase == SystemState::PHASE_DEPTH_ADJUST) {
        sub_pct = 0.50f;
    } else if (_current_phase == SystemState::PHASE_CAPTURING) {
        sub_pct = 0.75f;
    }

    return base + sub_pct * station_pct;
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
    std::string station_info = "";
    if (_total_stations > 0 && _current_station_index < _total_stations) {
        station_info = " [" + _stations[_current_station_index].name +
                       " " + std::to_string(_current_station_index + 1) +
                       "/" + std::to_string(_total_stations) + "]";
    }

    switch (_current_phase) {
        case SystemState::PHASE_IDLE:
            return "等待任务";
        case SystemState::PHASE_MOVING_TO_STATION:
            return "AGV导航到站位" + station_info;
        case SystemState::PHASE_ARM_PRESET:
            return "机械臂移动到预设位姿" + station_info;
        case SystemState::PHASE_DEPTH_ADJUST:
            return "深度测距微调" + station_info;
        case SystemState::PHASE_CAPTURING:
            return "拍照检测" + station_info;
        case SystemState::PHASE_PAUSED:
            return "任务已暂停" + station_info;
        case SystemState::PHASE_COMPLETED:
            return "巡检完成";
        case SystemState::PHASE_FAILED:
            return "巡检失败: " + _error_message;
        case SystemState::PHASE_STOPPED:
            return "任务已停止";
        default:
            return "未知状态";
    }
}

bool TaskCoordinatorNode::check_timeout(const std::string& step_name, double timeout_sec) {
    if (timeout_sec <= 0.0) {
        return false;
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
