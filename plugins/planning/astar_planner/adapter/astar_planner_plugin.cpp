#include "astar_planner_plugin.hpp"
#include <iostream>

namespace astar_planner {
namespace adapter {

// ========== GridMapAdapter ==========

/**
 * @brief 将平台的 OccupancyGrid 适配到算法层的 GridMapInterface
 *
 * 这是适配器模式的典型应用：
 * - 平台层使用 navsim::planning::OccupancyGrid
 * - 算法层使用 algorithm::GridMapInterface
 * - 适配器负责转换
 */
class AstarPlannerPlugin::GridMapAdapter : public algorithm::GridMapInterface {
public:
  explicit GridMapAdapter(const navsim::planning::OccupancyGrid* grid)
      : grid_(grid) {}

  bool isOccupied(double x, double y) const override {
    if (!grid_) {
      return true;  // 没有地图时，认为所有位置都被占用
    }

    // 转换为栅格坐标（手动实现，避免链接问题）
    int gx = static_cast<int>((x - grid_->config.origin.x) / grid_->config.resolution);
    int gy = static_cast<int>((y - grid_->config.origin.y) / grid_->config.resolution);

    // 检查是否在地图范围内
    if (gx < 0 || gx >= grid_->config.width ||
        gy < 0 || gy >= grid_->config.height) {
      return true;  // 超出地图范围，认为被占用
    }

    // 检查占用状态（阈值 50）
    int index = gy * grid_->config.width + gx;
    return grid_->data[index] >= 50;
  }

  void getBounds(double& min_x, double& max_x,
                double& min_y, double& max_y) const override {
    if (!grid_) {
      min_x = max_x = min_y = max_y = 0.0;
      return;
    }

    min_x = grid_->config.origin.x;
    min_y = grid_->config.origin.y;
    max_x = min_x + grid_->config.width * grid_->config.resolution;
    max_y = min_y + grid_->config.height * grid_->config.resolution;
  }

  double getResolution() const override {
    if (!grid_) {
      return 0.1;  // 默认分辨率
    }
    return grid_->config.resolution;
  }

private:
  const navsim::planning::OccupancyGrid* grid_;
};

// ========== AstarPlannerPlugin ==========

AstarPlannerPlugin::AstarPlannerPlugin() {
  planner_ = std::make_unique<algorithm::AstarPlanner>();
}

navsim::plugin::PlannerPluginMetadata AstarPlannerPlugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "AstarPlanner";
  metadata.version = "1.0.0";
  metadata.author = "NavSim Team";
  metadata.description = "A* path planner with occupancy grid map";
  metadata.type = "search";

  // 声明需要栅格地图
  metadata.required_perception_data = {"occupancy_grid"};

  metadata.can_be_fallback = true;  // 可以作为降级规划器

  return metadata;
}

bool AstarPlannerPlugin::initialize(const nlohmann::json& config) {
  if (initialized_) {
    std::cerr << "[AstarPlanner] Already initialized" << std::endl;
    return false;
  }

  try {
    // 加载配置
    if (!loadConfig(config)) {
      std::cerr << "[AstarPlanner] Failed to load config" << std::endl;
      return false;
    }

    // 验证配置
    if (!validateConfig()) {
      std::cerr << "[AstarPlanner] Invalid config" << std::endl;
      return false;
    }

    // 设置算法配置
    planner_->setConfig(config_);

    initialized_ = true;

    if (verbose_) {
      std::cout << "[AstarPlanner] Initialized with config:" << std::endl;
      std::cout << "  - max_velocity: " << config_.max_velocity << " m/s" << std::endl;
      std::cout << "  - max_acceleration: " << config_.max_acceleration << " m/s²" << std::endl;
      std::cout << "  - step_size: " << config_.step_size << " m" << std::endl;
      std::cout << "  - max_iterations: " << config_.max_iterations << std::endl;
    }

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[AstarPlanner] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool AstarPlannerPlugin::plan(
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

  // 获取栅格地图
  if (!context.occupancy_grid) {
    result.success = false;
    result.failure_reason = "Occupancy grid not available in context";
    // failed_plans_++;
    return false;
  }

  // 创建地图适配器并设置到算法
  grid_map_adapter_ = std::make_shared<GridMapAdapter>(context.occupancy_grid.get());
  planner_->setMap(grid_map_adapter_);

  // 转换输入数据
  Eigen::Vector3d start = convertPose(context.ego.pose);
  Eigen::Vector3d goal = convertPose(context.task.goal_pose);

  if (verbose_) {
    std::cout << "[AstarPlanner] Planning from (" << start.transpose()
              << ") to (" << goal.transpose() << ")" << std::endl;
    std::cout << "[AstarPlanner] Grid map: " << context.occupancy_grid->config.width
              << "x" << context.occupancy_grid->config.height
              << " @ " << context.occupancy_grid->config.resolution << " m/cell" << std::endl;
  }

  // 调用算法
  auto start_time = std::chrono::high_resolution_clock::now();
  auto algo_result = planner_->plan(start, goal);
  auto end_time = std::chrono::high_resolution_clock::now();

  double planning_time_ms = std::chrono::duration<double, std::milli>(
      end_time - start_time).count();

  if (verbose_) {
    std::cout << "[AstarPlanner] Planning "
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
  result.planner_name = "AstarPlanner";

  // TODO: 取消注释以启用统计
  // if (result.success) {
  //   successful_plans_++;
  //   total_planning_time_ms_ += planning_time_ms;
  // } else {
  //   failed_plans_++;
  // }

  return result.success;
}

std::pair<bool, std::string> AstarPlannerPlugin::isAvailable(
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

void AstarPlannerPlugin::reset() {
  if (verbose_) {
    std::cout << "[AstarPlanner] Resetting plugin..." << std::endl;
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

nlohmann::json AstarPlannerPlugin::getStatistics() const {
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

bool AstarPlannerPlugin::loadConfig(const nlohmann::json& config) {
  try {
    // 从 JSON 解析配置参数
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
    if (config.contains("heuristic_weight")) {
      config_.heuristic_weight = config["heuristic_weight"].get<double>();
    }
    if (config.contains("allow_diagonal")) {
      config_.allow_diagonal = config["allow_diagonal"].get<bool>();
    }
    if (config.contains("goal_tolerance")) {
      config_.goal_tolerance = config["goal_tolerance"].get<double>();
    }
    if (config.contains("obstacle_inflation")) {
      config_.obstacle_inflation = config["obstacle_inflation"].get<double>();
    }
    if (config.contains("verbose")) {
      verbose_ = config["verbose"].get<bool>();
    }

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[AstarPlanner] Failed to parse config: " << e.what() << std::endl;
    return false;
  }
}

bool AstarPlannerPlugin::validateConfig() const {
  // TODO: 根据您的算法需求验证配置
  // 示例：
  if (config_.max_velocity <= 0.0) {
    std::cerr << "[AstarPlanner] Invalid max_velocity: " << config_.max_velocity << std::endl;
    return false;
  }
  if (config_.max_acceleration <= 0.0) {
    std::cerr << "[AstarPlanner] Invalid max_acceleration: " << config_.max_acceleration << std::endl;
    return false;
  }
  if (config_.step_size <= 0.0) {
    std::cerr << "[AstarPlanner] Invalid step_size: " << config_.step_size << std::endl;
    return false;
  }
  if (config_.max_iterations <= 0) {
    std::cerr << "[AstarPlanner] Invalid max_iterations: " << config_.max_iterations << std::endl;
    return false;
  }

  return true;
}

Eigen::Vector3d AstarPlannerPlugin::convertPose(
    const navsim::planning::Pose2d& pose) const {
  return Eigen::Vector3d(pose.x, pose.y, pose.yaw);
}

bool AstarPlannerPlugin::convertAlgorithmOutputToResult(
    const algorithm::AstarPlanner::Result& algo_result,
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
} // namespace astar_planner

