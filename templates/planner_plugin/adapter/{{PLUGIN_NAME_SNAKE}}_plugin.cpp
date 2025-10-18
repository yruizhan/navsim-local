#include "{{PLUGIN_NAME_SNAKE}}_plugin.hpp"
#include <iostream>

namespace {{NAMESPACE}} {
namespace adapter {

{{PLUGIN_NAME}}Plugin::{{PLUGIN_NAME}}Plugin() {
  planner_ = std::make_unique<algorithm::{{PLUGIN_NAME}}>();
}

navsim::plugin::PlannerPluginMetadata {{PLUGIN_NAME}}Plugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "{{PLUGIN_NAME}}";
  metadata.version = "1.0.0";
  metadata.author = "{{AUTHOR}}";
  metadata.description = "{{DESCRIPTION}}";
  metadata.type = "search";  // TODO: 修改为实际类型 ("search", "optimization", "sampling", "learning")

  // TODO: 根据需要声明必需的感知数据
  // metadata.required_perception_data = {"occupancy_grid"};  // 需要栅格地图
  // metadata.required_perception_data = {"esdf_map"};        // 需要 ESDF 地图
  // metadata.required_perception_data = {};                  // 不需要感知数据

  metadata.can_be_fallback = false;  // TODO: 是否可以作为降级规划器

  return metadata;
}

bool {{PLUGIN_NAME}}Plugin::initialize(const nlohmann::json& config) {
  if (initialized_) {
    std::cerr << "[{{PLUGIN_NAME}}] Already initialized" << std::endl;
    return false;
  }

  try {
    // 加载配置
    if (!loadConfig(config)) {
      std::cerr << "[{{PLUGIN_NAME}}] Failed to load config" << std::endl;
      return false;
    }

    // 验证配置
    if (!validateConfig()) {
      std::cerr << "[{{PLUGIN_NAME}}] Invalid config" << std::endl;
      return false;
    }

    // 设置算法配置
    planner_->setConfig(config_);

    initialized_ = true;

    if (verbose_) {
      std::cout << "[{{PLUGIN_NAME}}] Initialized with config:" << std::endl;
      std::cout << "  - max_velocity: " << config_.max_velocity << " m/s" << std::endl;
      std::cout << "  - max_acceleration: " << config_.max_acceleration << " m/s²" << std::endl;
      std::cout << "  - step_size: " << config_.step_size << " m" << std::endl;
      std::cout << "  - max_iterations: " << config_.max_iterations << std::endl;
    }

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[{{PLUGIN_NAME}}] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool {{PLUGIN_NAME}}Plugin::plan(
    const navsim::planning::PlanningContext& context,
    std::chrono::milliseconds deadline,
    navsim::plugin::PlanningResult& result) {

  // TODO: 取消注释以启用统计
  // total_plans_++;

  if (!initialized_) {
    result.success = false;
    result.failure_reason = "Plugin not initialized";
    // failed_plans_++;
    return false;
  }

  // TODO: 如果需要感知数据，在这里检查并获取
  // 示例：获取 ESDF 地图
  // esdf_map_ = context.getCustomData<navsim::perception::ESDFMap>("perception_esdf_map");
  // if (!esdf_map_) {
  //   result.success = false;
  //   result.failure_reason = "ESDF map not available in context";
  //   failed_plans_++;
  //   return false;
  // }

  // 转换输入数据
  Eigen::Vector3d start = convertPose(context.ego.pose);
  Eigen::Vector3d goal = convertPose(context.task.goal_pose);

  if (verbose_) {
    std::cout << "[{{PLUGIN_NAME}}] Planning from (" << start.transpose()
              << ") to (" << goal.transpose() << ")" << std::endl;
  }

  // 调用算法
  auto start_time = std::chrono::high_resolution_clock::now();
  auto algo_result = planner_->plan(start, goal);
  auto end_time = std::chrono::high_resolution_clock::now();

  double planning_time_ms = std::chrono::duration<double, std::milli>(
      end_time - start_time).count();

  if (verbose_) {
    std::cout << "[{{PLUGIN_NAME}}] Planning "
              << (algo_result.success ? "succeeded" : "failed")
              << " in " << planning_time_ms << " ms" << std::endl;
  }

  // 转换输出数据
  if (!convertAlgorithmOutputToResult(algo_result, result)) {
    result.success = false;
    result.failure_reason = "Failed to convert algorithm output";
    // failed_plans_++;
    return false;
  }

  result.computation_time_ms = planning_time_ms;
  result.planner_name = "{{PLUGIN_NAME}}";

  // TODO: 取消注释以启用统计
  // if (result.success) {
  //   successful_plans_++;
  //   total_planning_time_ms_ += planning_time_ms;
  // } else {
  //   failed_plans_++;
  // }

  return result.success;
}

std::pair<bool, std::string> {{PLUGIN_NAME}}Plugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {

  if (!initialized_) {
    return {false, "Plugin not initialized"};
  }

  // TODO: 根据需要检查必需的感知数据
  // 示例：检查 ESDF 地图
  // auto esdf_map = context.getCustomData<navsim::perception::ESDFMap>("perception_esdf_map");
  // if (!esdf_map) {
  //   return {false, "ESDF map not available in context"};
  // }

  // 示例：检查栅格地图
  // if (!context.occupancy_grid) {
  //   return {false, "Occupancy grid not available"};
  // }

  // 简单的规划器不需要任何感知数据
  return {true, ""};
}

void {{PLUGIN_NAME}}Plugin::reset() {
  if (verbose_) {
    std::cout << "[{{PLUGIN_NAME}}] Resetting plugin..." << std::endl;
  }

  // TODO: 取消注释以重置统计
  // total_plans_ = 0;
  // successful_plans_ = 0;
  // failed_plans_ = 0;
  // total_planning_time_ms_ = 0.0;

  if (planner_) {
    planner_->reset();
  }
}

nlohmann::json {{PLUGIN_NAME}}Plugin::getStatistics() const {
  nlohmann::json stats;

  // TODO: 取消注释以返回统计信息
  // stats["total_plans"] = total_plans_;
  // stats["successful_plans"] = successful_plans_;
  // stats["failed_plans"] = failed_plans_;
  // stats["success_rate"] = (total_plans_ > 0)
  //     ? static_cast<double>(successful_plans_) / total_plans_
  //     : 0.0;
  // stats["avg_planning_time_ms"] = (total_plans_ > 0)
  //     ? total_planning_time_ms_ / total_plans_
  //     : 0.0;

  return stats;
}

// ========== 私有辅助函数 ==========

bool {{PLUGIN_NAME}}Plugin::loadConfig(const nlohmann::json& config) {
  try {
    // 从 JSON 解析配置参数
    // TODO: 根据您的算法配置修改以下代码
    if (config.contains("max_velocity")) {
      config_.max_velocity = config["max_velocity"].get<double>();
    }
    if (config.contains("max_acceleration")) {
      config_.max_acceleration = config["max_acceleration"].get<double>();
    }
    if (config.contains("step_size")) {
      config_.step_size = config["step_size"].get<double>();
    }
    if (config.contains("max_iterations")) {
      config_.max_iterations = config["max_iterations"].get<int>();
    }
    if (config.contains("verbose")) {
      verbose_ = config["verbose"].get<bool>();
    }

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[{{PLUGIN_NAME}}] Failed to parse config: " << e.what() << std::endl;
    return false;
  }
}

bool {{PLUGIN_NAME}}Plugin::validateConfig() const {
  // TODO: 根据您的算法需求验证配置
  // 示例：
  if (config_.max_velocity <= 0.0) {
    std::cerr << "[{{PLUGIN_NAME}}] Invalid max_velocity: " << config_.max_velocity << std::endl;
    return false;
  }
  if (config_.max_acceleration <= 0.0) {
    std::cerr << "[{{PLUGIN_NAME}}] Invalid max_acceleration: " << config_.max_acceleration << std::endl;
    return false;
  }
  if (config_.step_size <= 0.0) {
    std::cerr << "[{{PLUGIN_NAME}}] Invalid step_size: " << config_.step_size << std::endl;
    return false;
  }
  if (config_.max_iterations <= 0) {
    std::cerr << "[{{PLUGIN_NAME}}] Invalid max_iterations: " << config_.max_iterations << std::endl;
    return false;
  }

  return true;
}

Eigen::Vector3d {{PLUGIN_NAME}}Plugin::convertPose(
    const navsim::planning::Pose2d& pose) const {
  return Eigen::Vector3d(pose.x, pose.y, pose.yaw);
}

bool {{PLUGIN_NAME}}Plugin::convertAlgorithmOutputToResult(
    const algorithm::{{PLUGIN_NAME}}::Result& algo_result,
    navsim::plugin::PlanningResult& result) const {

  result.success = algo_result.success;
  result.failure_reason = algo_result.failure_reason;

  if (!algo_result.success) {
    return true;  // 失败也是有效的结果
  }

  // TODO: 根据您的算法输出修改以下代码
  // 转换路径为轨迹点
  result.trajectory.clear();
  result.trajectory.reserve(algo_result.path.size());

  for (const auto& wp : algo_result.path) {
    navsim::plugin::TrajectoryPoint point;
    point.pose.x = wp.position.x();
    point.pose.y = wp.position.y();
    point.pose.yaw = wp.position.z();
    point.twist.vx = wp.velocity;
    point.twist.vy = 0.0;
    point.twist.omega = 0.0;
    point.acceleration = 0.0;
    point.time_from_start = wp.timestamp;

    result.trajectory.push_back(point);
  }

  // 设置元数据
  result.metadata["path_length"] = static_cast<double>(algo_result.path.size());

  return true;
}

} // namespace adapter
} // namespace {{NAMESPACE}}

