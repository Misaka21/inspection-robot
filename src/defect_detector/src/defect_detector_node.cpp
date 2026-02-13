#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <inspection_interface/msg/defect_info.hpp>

namespace defect_detector {

class DefectDetectorNode : public rclcpp::Node {
public:
    DefectDetectorNode()
        : Node("defect_detector_node") {
        RCLCPP_INFO(this->get_logger(), "Starting Defect Detector Node");

        // 订阅工业相机图像
        _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/inspection/hikvision/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                process_image(msg);
            });

        // 发布检测结果
        _result_pub = this->create_publisher<inspection_interface::msg::DefectInfo>(
            "~/result", 10);

        // 声明参数
        this->declare_parameter("confidence_threshold", 0.7);
        this->declare_parameter("nms_threshold", 0.5);

        // 创建服务
        _detect_srv = this->create_service<std_srvs::srv::Trigger>(
            "~/detect_defect",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                RCLCPP_INFO(this->get_logger(), "Received detect request");
                response->success = true;
                response->message = "Defect detection triggered";
            });
    }

private:
    void process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        // TODO: 实现缺陷检测算法
        // 1. 将 sensor_msgs::Image 转换为 OpenCV 图像 (使用 cv_bridge)
        // 2. 运行缺陷检测模型
        // 3. NMS 后处理
        // 4. 发布结果

        inspection_interface::msg::DefectInfo result;
        result.defect_id = 0;
        result.defect_type = "none";
        result.confidence = 0.0;
        _result_pub->publish(result);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Publisher<inspection_interface::msg::DefectInfo>::SharedPtr _result_pub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _detect_srv;
};

}  // namespace defect_detector

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<defect_detector::DefectDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
