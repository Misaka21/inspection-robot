#include "path_planner/stance_sampler.hpp"

#include <cmath>

namespace path_planner {

geometry_msgs::msg::Quaternion StanceSampler::quaternion_from_yaw(double yaw) {
    geometry_msgs::msg::Quaternion q;
    double half = yaw * 0.5;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(half);
    q.w = std::cos(half);
    return q;
}

void StanceSampler::extract_z_axis(const geometry_msgs::msg::Pose& pose,
                                   double& x, double& y, double& z) {
    // 从四元数提取旋转矩阵的Z轴
    // R = [1-2(y^2+z^2), 2(xy-zw), 2(xz+yw)]
    //     [2(xy+zw), 1-2(x^2+z^2), 2(yz-xw)]
    //     [2(xz-yw), 2(yz+xw), 1-2(x^2+y^2)]
    // Z轴是第三列
    const auto& q = pose.orientation;
    x = 2.0 * (q.x * q.z - q.y * q.w);
    y = 2.0 * (q.y * q.z + q.x * q.w);
    z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
}

geometry_msgs::msg::Pose StanceSampler::compute_look_at_pose(
    const geometry_msgs::msg::Point& from,
    const geometry_msgs::msg::Point& to) {

    geometry_msgs::msg::Pose pose;
    pose.position = from;

    // 计算从from指向to的向量
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dz = to.z - from.z;

    // 归一化
    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len > 1e-6) {
        dx /= len;
        dy /= len;
        dz /= len;
    }

    // 构造旋转矩阵，使Z轴指向目标
    // Z = (dx, dy, dz)
    // 选择X轴：在XY平面内与Z垂直
    double xX = -dy;
    double xY = dx;
    double xZ = 0.0;
    double xlen = std::sqrt(xX*xX + xY*xY);
    if (xlen > 1e-6) {
        xX /= xlen;
        xY /= xlen;
    } else {
        // Z轴垂直向上，选择X轴为(1,0,0)
        xX = 1.0;
        xY = 0.0;
    }

    // Y = Z × X
    double yX = dy * xZ - dz * xY;
    double yY = dz * xX - dx * xZ;
    double yZ = dx * xY - dy * xX;

    // 从旋转矩阵构造四元数
    // trace = Rxx + Ryy + Rzz
    double trace = xX + yY + dz;
    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        pose.orientation.w = 0.25 / s;
        pose.orientation.x = (yZ - dy) * s;
        pose.orientation.y = (dx - xZ) * s;
        pose.orientation.z = (xY - yX) * s;
    } else {
        // 处理其他情况...
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
    }

    return pose;
}

std::vector<StanceSampler::Stance> StanceSampler::sample_stances(
    const geometry_msgs::msg::Pose& workpiece_pose,
    double camera_working_dist,
    double candidate_radius,
    double yaw_step_deg) {

    std::vector<Stance> stances;

    // 提取工件表面法向（Z轴）
    double nx, ny, nz;
    extract_z_axis(workpiece_pose, nx, ny, nz);

    // 归一化法向
    double nlen = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (nlen > 1e-6) {
        nx /= nlen;
        ny /= nlen;
        nz /= nlen;
    }

    // TCP位置 = 工件位置 + 法向 × 工作距离（相机在工件"上方"）
    geometry_msgs::msg::Point tcp_point;
    tcp_point.x = workpiece_pose.position.x + nx * camera_working_dist;
    tcp_point.y = workpiece_pose.position.y + ny * camera_working_dist;
    tcp_point.z = workpiece_pose.position.z + nz * camera_working_dist;

    // 在圆上均匀采样AGV站位
    for (double yaw_deg = 0.0; yaw_deg < 360.0; yaw_deg += yaw_step_deg) {
        double yaw = yaw_deg * M_PI / 180.0;

        Stance stance;

        // AGV位置：以工件为中心，candidate_radius为半径的圆上
        // 注意：这里假设工件在地面投影为中心，AGV在水平面上移动
        // 如果工件有高度，AGV仍在地面（z=0或当前高度）
        stance.agv_pose.position.x = workpiece_pose.position.x + candidate_radius * std::cos(yaw);
        stance.agv_pose.position.y = workpiece_pose.position.y + candidate_radius * std::sin(yaw);
        stance.agv_pose.position.z = 0.0;  // 假设AGV在地面

        // AGV朝向：朝向工件中心
        double agv_yaw = yaw + M_PI;  // 朝向圆心
        stance.agv_pose.orientation = quaternion_from_yaw(agv_yaw);

        // TCP目标位姿
        stance.tcp_pose.position = tcp_point;
        geometry_msgs::msg::Pose look_at_pose = compute_look_at_pose(tcp_point, workpiece_pose.position);
        stance.tcp_pose.orientation = look_at_pose.orientation;

        stance.distance_to_workpiece = candidate_radius;

        stances.push_back(stance);
    }

    return stances;
}

}  // namespace path_planner
