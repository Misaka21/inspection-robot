#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace pose_detector {

class PoseDetectorNode : public rclcpp::Node {
public:
    PoseDetectorNode()
        : Node("pose_detector_node"),
          _tf_buffer(this->get_clock()),
          _tf_listener(_tf_buffer) {
        RCLCPP_INFO(this->get_logger(), "Starting Pose Detector Node");

        // 订阅点云
        _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/inspection/realsense/d435/depth/color/points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                process_pointcloud(msg);
            });

        // 发布检测到的位姿
        _pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "detected_pose", 10);

        // 发布置信度
        _confidence_pub = this->create_publisher<std_msgs::msg::Float32>(
            "confidence", 10);

        // 声明参数
        // algorithm：配准算法选择，"icp" = 迭代最近点，"fpfh" = 快速点特征直方图初始配准
        this->declare_parameter("algorithm", "icp");
        // voxel_size：体素下采样的格子大小（米），决定处理速度与精度的权衡
        this->declare_parameter("voxel_size", 0.01);
        // max_correspondence_dist：ICP 最大对应点距离，超过此距离的点对不计入配准
        this->declare_parameter("max_correspondence_dist", 0.05);

        // 创建服务
        _detect_srv = this->create_service<std_srvs::srv::Trigger>(
            "detect",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                RCLCPP_INFO(this->get_logger(), "Received detect request");
                // 触发一次检测（当前是自动检测，服务只是响应确认）
                response->success = true;
                response->message = "Detection triggered";
            });
    }

private:
    void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // TODO: 实现位姿检测算法
        // 1. 点云预处理（降采样）- voxel_size 控制采样密度，值越大越稀疏但越快
        // 2. 特征提取 (FPFH) 或 ICP 配准
        //    - ICP（Iterative Closest Point）：迭代最近点，需要初始估计较近时收敛
        //    - FPFH（Fast Point Feature Histograms）：局部几何特征，适合全局初始配准
        // 3. 输出位姿
        // 注意：耗时算法不应在此回调直接执行，应缓存点云后由 service 触发（见 CLAUDE.md）

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        // 骨架占位：quaternion.w=1.0 表示单位四元数（无旋转），等待算法实现后替换
        pose.pose.orientation.w = 1.0;  // 默认朝上
        _pose_pub->publish(pose);

        std_msgs::msg::Float32 conf;
        conf.data = 1.0;  // 骨架占位：confidence=1.0，实际应由配准误差/分数计算
        _confidence_pub->publish(conf);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _confidence_pub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _detect_srv;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
};

}  // namespace pose_detector

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_detector::PoseDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
