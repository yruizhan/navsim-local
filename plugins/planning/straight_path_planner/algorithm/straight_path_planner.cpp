#include "straight_path_planner.hpp"
#include <chrono>
#include <cmath>

namespace straight_path_planner {
namespace algorithm {

// ========== Config ==========

StraightPathPlanner::Config StraightPathPlanner::Config::fromJson(const nlohmann::json& json) {
  Config config;
  
  if (json.contains("max_velocity")) {
    config.max_velocity = json["max_velocity"].get<double>();
  }
  if (json.contains("max_acceleration")) {
    config.max_acceleration = json["max_acceleration"].get<double>();
  }
  if (json.contains("step_size")) {
    config.step_size = json["step_size"].get<double>();
  }
  if (json.contains("max_iterations")) {
    config.max_iterations = json["max_iterations"].get<int>();
  }
  
  return config;
}

// ========== StraightPathPlanner ==========

StraightPathPlanner::StraightPathPlanner(const Config& config)
    : config_(config) {
}

void StraightPathPlanner::setConfig(const Config& config) {
  config_ = config;
}

StraightPathPlanner::Result StraightPathPlanner::plan(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal) {
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  Result result;
  
  // TODO: 实现您的规划算法
  // 这里是一个简单的示例：生成直线路径
  
  // 计算距离
  double distance = (goal.head<2>() - start.head<2>()).norm();
  
  // 生成路径点
  int num_points = static_cast<int>(distance / config_.step_size) + 2;
  result.path.reserve(num_points);
  
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    
    Waypoint wp;
    wp.position = start + t * (goal - start);
    wp.velocity = config_.max_velocity;
    wp.timestamp = i * config_.step_size / config_.max_velocity;
    
    result.path.push_back(wp);
  }
  
  result.success = true;
  
  // 计算耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  result.computation_time_ms = duration.count() / 1000.0;
  
  return result;
}

void StraightPathPlanner::reset() {
  // TODO: 重置算法状态
}

} // namespace algorithm
} // namespace straight_path_planner

