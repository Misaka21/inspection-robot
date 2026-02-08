#ifndef HIKVISION_DRIVER_HPP
#define HIKVISION_DRIVER_HPP

// HKSDK
#include <MvCameraControl.h>

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mutex>
#include <thread>
#include <atomic>

namespace hikvision_driver {

class HikvisionDriverNode : public rclcpp::Node {
public:
    explicit HikvisionDriverNode(const rclcpp::NodeOptions& options);
    ~HikvisionDriverNode() override;

private:
    std::string frame_id;
    int nRet = MV_OK;
    void* camera_handle = nullptr;
    std::thread capture_thread;
    std::thread monitor_thread;
    
    // 图像发布
    image_transport::CameraPublisher image_pub;
    
    // 触发服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service;
    std::atomic<bool> trigger_flag{false};

    MV_IMAGE_BASIC_INFO img_info;
    MV_CC_PIXEL_CONVERT_PARAM convert_param;
    
    // 相机信息管理
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
    sensor_msgs::msg::CameraInfo camera_info_msg;

    rclcpp::AsyncParametersClient::SharedPtr param_client;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub;

    int fail_cnt = 0;
    bool rotate_180 = false;
    std::atomic<bool> grab_on{false};
    std::atomic<bool> monitor_on{false};
    std::atomic<bool> camera_failed{false};
    std::atomic<bool> param_changed{false};
    
    // 触发模式
    bool use_trigger_mode = false;

    void declare_params();
    void init_camera();
    void reset();
    void open_device();
    void close_device();
    void start_grab();
    void stop_grab();
    void set_hk_params();
    void set_grab_params(int offset_x, int offset_y, int roi_width, int roi_height);
    void grab();
    void monitor();
    void trigger_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    std::pair<int, int> get_sensor_height_width();
};

class CameraException : public std::exception {
public:
    std::string info;
    explicit CameraException(const std::string&& _info) : info{_info} {}
    const char* what() const noexcept override { return info.c_str(); }
};

}  // namespace hikvision_driver

#endif  // HIKVISION_DRIVER_HPP
