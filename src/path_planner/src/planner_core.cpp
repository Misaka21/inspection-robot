#include "path_planner/planner_core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <limits>

namespace path_planner {

bool PlannerCore::init(rclcpp::Node::SharedPtr node) {
    node_ = node;

    if (!ik_adapter_.init(node, "elfin_arm")) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize IK adapter");
        return false;
    }

    initialized_ = true;
    RCLCPP_INFO(node->get_logger(), "PlannerCore initialized");
    return true;
}

std::vector<PlannerCore::Waypoint> PlannerCore::generate_candidates_for_point(
    const geometry_msgs::msg::Pose& workpiece_pose,
    int point_id,
    const PlanningRequest& req) {

    std::vector<Waypoint> candidates;

    // 1. 采样AGV站位
    auto stances = stance_sampler_.sample_stances(
        workpiece_pose,
        req.camera_working_dist,
        req.candidate_radius,
        req.yaw_step_deg);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Point %d: sampled %zu stances",
                 point_id, stances.size());

    // 2. 对每个站位做IK检查
    for (const auto& stance : stances) {
        // 快速距离检查（在IK之前过滤明显不可达的）
        double dx = stance.tcp_pose.position.x - stance.agv_pose.position.x - ik_adapter_.get_joint_names().empty() ? 0 : 0;
        double dy = stance.tcp_pose.position.y - stance.agv_pose.position.y;
        double dz = stance.tcp_pose.position.z - stance.agv_pose.position.z;
        double tcp_dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (tcp_dist < req.arm_reach_min || tcp_dist > req.arm_reach_max) {
            continue;  // 快速过滤
        }

        // IK求解
        auto ik_result = ik_adapter_.check_reachability(
            stance.agv_pose, stance.tcp_pose);

        if (!ik_result.reachable) {
            continue;
        }

        // 创建候选waypoint
        Waypoint wp;
        wp.agv_pose = stance.agv_pose;
        wp.joint_angles = ik_result.joint_angles;
        wp.detection_point_id = point_id;
        wp.manipulability = ik_result.manipulability;

        candidates.push_back(wp);
    }

    // 3. 按可操作性排序，保留top-k
    std::sort(candidates.begin(), candidates.end(),
              [](const Waypoint& a, const Waypoint& b) {
                  return a.manipulability > b.manipulability;
              });

    if (candidates.size() > static_cast<size_t>(req.max_candidates_per_point)) {
        candidates.resize(req.max_candidates_per_point);
    }

    RCLCPP_INFO(node_->get_logger(),
                "Point %d: %zu/%zu candidates reachable (kept top %d)",
                point_id, candidates.size(), stances.size(),
                req.max_candidates_per_point);

    return candidates;
}

double PlannerCore::compute_distance(const geometry_msgs::msg::Pose& a,
                                      const geometry_msgs::msg::Pose& b) {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<PlannerCore::Waypoint> PlannerCore::select_stances_and_order(
    const std::vector<std::vector<Waypoint>>& candidates_per_point,
    const geometry_msgs::msg::Pose& start_pose) {

    std::vector<Waypoint> route;
    geometry_msgs::msg::Pose current = start_pose;
    std::vector<bool> visited(candidates_per_point.size(), false);

    // 贪心：每次选最近的未访问点的最近候选
    for (size_t step = 0; step < candidates_per_point.size(); ++step) {
        double best_dist = std::numeric_limits<double>::max();
        int best_point_idx = -1;
        Waypoint best_wp;

        // 遍历所有未访问的检测点
        for (size_t i = 0; i < candidates_per_point.size(); ++i) {
            if (visited[i]) continue;
            if (candidates_per_point[i].empty()) continue;

            // 找这个点到当前位置最近的候选
            for (const auto& wp : candidates_per_point[i]) {
                double d = compute_distance(current, wp.agv_pose);
                if (d < best_dist) {
                    best_dist = d;
                    best_point_idx = static_cast<int>(i);
                    best_wp = wp;
                }
            }
        }

        if (best_point_idx < 0) {
            RCLCPP_ERROR(node_->get_logger(),
                         "No reachable candidate found for remaining points");
            return route;  // 部分结果
        }

        visited[best_point_idx] = true;
        best_wp.distance_to_current = best_dist;
        route.push_back(best_wp);
        current = best_wp.agv_pose;
    }

    return route;
}

PlannerCore::PlanningResult PlannerCore::plan(const PlanningRequest& req) {
    PlanningResult result;

    if (!initialized_) {
        result.error_msg = "PlannerCore not initialized";
        return result;
    }

    if (req.detection_points.empty()) {
        result.error_msg = "No detection points provided";
        return result;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Planning for %zu detection points",
                req.detection_points.size());

    // 1. 为每个检测点生成候选站位
    std::vector<std::vector<Waypoint>> candidates_per_point;
    for (size_t i = 0; i < req.detection_points.size(); ++i) {
        auto candidates = generate_candidates_for_point(
            req.detection_points[i], static_cast<int>(i), req);

        if (candidates.empty()) {
            result.error_msg = "No reachable stance for detection point " + std::to_string(i);
            RCLCPP_ERROR(node_->get_logger(), "%s", result.error_msg.c_str());
            return result;
        }

        candidates_per_point.push_back(candidates);
    }

    // 2. 选择站位并排序
    result.waypoints = select_stances_and_order(
        candidates_per_point, req.current_agv_pose);

    // 3. 计算总距离
    result.total_distance = 0.0;
    geometry_msgs::msg::Pose prev = req.current_agv_pose;
    for (const auto& wp : result.waypoints) {
        result.total_distance += compute_distance(prev, wp.agv_pose);
        prev = wp.agv_pose;
    }

    result.success = true;
    RCLCPP_INFO(node_->get_logger(),
                "Planning success: %zu waypoints, total distance %.2fm",
                result.waypoints.size(), result.total_distance);

    return result;
}

}  // namespace path_planner
