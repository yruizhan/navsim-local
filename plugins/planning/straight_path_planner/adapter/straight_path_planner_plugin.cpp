#include "straight_path_planner_plugin.hpp"
#include <iostream>

namespace straight_path_planner {
namespace adapter {

StraightPathPlannerPlugin::StraightPathPlannerPlugin() {
  planner_ = std::make_unique<algorithm::StraightPathPlanner>();
}

navsim::plugin::PlannerPluginMetadata StraightPathPlannerPlugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "StraightPathPlanner";
  metadata.version = "1.0.0";
  metadata.author = "NavSim Team";
  metadata.description = "Simple straight-line path planner for validation";
  metadata.requires_occupancy_grid = false;  // TODO: 根据需要修改
  metadata.requires_esdf_map = false;        // TODO: 根据需要修改
  metadata.requires_lane_lines = false;      // TODO: 根据需要修改
  return metadata;
}

bool StraightPathPlannerPlugin::initialize(const nlohmann::json& config) {
  try {
    // 从 JSON 加载配置
    config_ = algorithm::StraightPathPlanner::Config::fromJson(config);
    
    // 设置算法配置
    planner_->setConfig(config_);
    
    // 打印配置信息
    std::cout << "[StraightPathPlanner] Initialized with config:" << std::endl;
    std::cout << "  - max_velocity: " << config_.max_velocity << " m/s" << std::endl;
    std::cout << "  - max_acceleration: " << config_.max_acceleration << " m/s²" << std::endl;
    std::cout << "  - step_size: " << config_.step_size << " m" << std::endl;
    std::cout << "  - max_iterations: " << config_.max_iterations << std::endl;
    
    initialized_ = true;
    return true;
    
  } catch (const std::exception& e) {
    std::cerr << "[StraightPathPlanner] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool StraightPathPlannerPlugin::plan(
    const navsim::planning::PlanningContext& context,
    navsim::plugin::PlanningResult& result) {
  
  if (!initialized_) {
    result.success = false;
    result.failure_reason = "Plugin not initialized";
    return false;
  }
  
  // 转换输入数据
  Eigen::Vector3d start = convertPose(context.ego.pose);
  Eigen::Vector3d goal = convertPose(context.task.goal_pose);
  
  // 调用算法
  auto algo_result = planner_->plan(start, goal);
  
  // 转换输出数据
  result.success = algo_result.success;
  result.failure_reason = algo_result.failure_reason;
  result.computation_time_ms = algo_result.computation_time_ms;
  
  if (algo_result.success) {
    result.trajectory = convertTrajectory(algo_result.path);
  }
  
  return result.success;
}

void StraightPathPlannerPlugin::reset() {
  if (planner_) {
    planner_->reset();
  }
}

// ========== 私有辅助函数 ==========

Eigen::Vector3d StraightPathPlannerPlugin::convertPose(
    const navsim::planning::Pose2d& pose) const {
  return Eigen::Vector3d(pose.x, pose.y, pose.yaw);
}

navsim::planning::Trajectory StraightPathPlannerPlugin::convertTrajectory(
    const std::vector<algorithm::StraightPathPlanner::Waypoint>& path) const {
  
  navsim::planning::Trajectory trajectory;
  trajectory.points.reserve(path.size());
  
  for (const auto& wp : path) {
    navsim::planning::TrajectoryPoint point;
    point.pose.x = wp.position.x();
    point.pose.y = wp.position.y();
    point.pose.yaw = wp.position.z();
    point.velocity = wp.velocity;
    point.timestamp = wp.timestamp;
    
    trajectory.points.push_back(point);
  }
  
  return trajectory;
}

} // namespace adapter
} // namespace straight_path_planner

