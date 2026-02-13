#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace pose_detector {

class PoseDetectorNode : public rclcpp::Node {
public:
    PoseDetectorNode()
        : Node("pose_detector_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {
        RCLCPP_INFO(this->get_logger(), "Starting Pose Detector Node");

        // 订阅点云
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/inspection/realsense/depth/color/points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                process_pointcloud(msg);
            });

        // 发布检测到的位姿
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/detected_pose", 10);

        // 发布置信度
        confidence_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "~/confidence", 10);

        // 声明参数
        this->declare_parameter("algorithm", "icp");
        this->declare_parameter("voxel_size", 0.01);
        this->declare_parameter("max_correspondence_dist", 0.05);
    }

private:
    void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // TODO: 实现位姿检测算法
        // 1. 点云预处理（降采样）
        // 2. 特征提取 (FPFH) 或 ICP 配准
        // 3. 输出位姿

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.orientation.w = 1.0;  // 默认朝上
        pose_pub_->publish(pose);

        std_msgs::msg::Float32 conf;
        conf.data = 1.0;
        confidence_pub_->publish(conf);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

}  // namespace pose_detector

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_detector::PoseDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
