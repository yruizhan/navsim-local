#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "plugin/data/planning_result.hpp"
#include "core/planning_context.hpp"
#include "../algorithm/astar_planner.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

namespace astar_planner {
namespace adapter {

/**
 * @brief AstarPlanner 插件适配器
 * 
 * 这是适配器层，负责：
 * 1. 实现平台插件接口
 * 2. 转换平台数据结构 ↔ 算法数据结构
 * 3. 处理配置和错误
 * 
 * 设计原则：
 * - 薄适配层，逻辑尽量放在 algorithm 层
 * - 只做数据转换，不做算法逻辑
 */
class AstarPlannerPlugin : public navsim::plugin::PlannerPluginInterface {
public:
  /**
   * @brief 构造函数
   */
  AstarPlannerPlugin();
  
  /**
   * @brief 析构函数
   */
  ~AstarPlannerPlugin() override = default;
  
  // ========== 插件接口实现 ==========
  
  /**
   * @brief 获取插件元数据
   */
  navsim::plugin::PlannerPluginMetadata getMetadata() const override;
  
  /**
   * @brief 初始化插件
   * 
   * @param config 配置参数（JSON 格式）
   *   - max_velocity: 最大速度 (m/s)
   *   - max_acceleration: 最大加速度 (m/s²)
   *   - step_size: 步长 (m)
   *   - max_iterations: 最大迭代次数
   */
  bool initialize(const nlohmann::json& config) override;
  
  /**
   * @brief 执行规划
   *
   * @param context 规划上下文（包含自车状态、目标、地图等）
   * @param deadline 截止时间（规划器应该在此时间前完成）
   * @param result 规划结果（输出）
   * @return 规划是否成功
   */
  bool plan(
      const navsim::planning::PlanningContext& context,
      std::chrono::milliseconds deadline,
      navsim::plugin::PlanningResult& result) override;

  /**
   * @brief 检查规划器是否可用
   *
   * @param context 规划上下文
   * @return {是否可用, 不可用原因}
   */
  std::pair<bool, std::string> isAvailable(
      const navsim::planning::PlanningContext& context) const override;

  /**
   * @brief 重置插件状态
   */
  void reset() override;

  /**
   * @brief 获取统计信息
   */
  nlohmann::json getStatistics() const override;

private:
  // ========== 算法实例 ==========
  // TODO: 将下面的示例替换为您的算法类
  // 示例：如果您的算法类是 JPS::JPSPlanner，则：
  //   std::unique_ptr<JPS::JPSPlanner> planner_;
  std::unique_ptr<algorithm::AstarPlanner> planner_;

  // ========== 算法配置 ==========
  // TODO: 将下面的示例替换为您的算法配置结构
  // 示例：如果您的算法配置是 JPS::JPSConfig，则：
  //   JPS::JPSConfig config_;
  algorithm::AstarPlanner::Config config_;

  // ========== 感知数据（如果需要）==========
  // 栅格地图适配器（将平台的 OccupancyGrid 适配到算法层的 GridMapInterface）
  class GridMapAdapter;
  std::shared_ptr<GridMapAdapter> grid_map_adapter_;

  // ========== 状态标志 ==========
  bool initialized_ = false;
  bool verbose_ = false;

  // ========== 统计信息（可选）==========
  // TODO: 如果需要统计信息，取消注释以下代码
  // mutable int total_plans_ = 0;
  // mutable int successful_plans_ = 0;
  // mutable int failed_plans_ = 0;
  // mutable double total_planning_time_ms_ = 0.0;

  // ========== 辅助方法 ==========

  /**
   * @brief 加载配置
   */
  bool loadConfig(const nlohmann::json& config);

  /**
   * @brief 验证配置
   */
  bool validateConfig() const;

  /**
   * @brief 转换平台数据 → 算法数据
   *
   * TODO: 根据您的算法需求修改此方法
   * 示例：如果需要转换起点和终点：
   *   bool convertContextToAlgorithmInput(
   *       const navsim::planning::PlanningContext& context,
   *       Eigen::Vector3d& start,
   *       Eigen::Vector3d& goal) const;
   */
  Eigen::Vector3d convertPose(const navsim::planning::Pose2d& pose) const;

  /**
   * @brief 转换算法输出 → 平台数据
   *
   * TODO: 根据您的算法输出修改此方法
   * 示例：如果算法返回路径点列表：
   *   bool convertAlgorithmOutputToResult(
   *       const std::vector<YourWaypoint>& path,
   *       navsim::plugin::PlanningResult& result) const;
   */
  bool convertAlgorithmOutputToResult(
      const algorithm::AstarPlanner::Result& algo_result,
      navsim::plugin::PlanningResult& result) const;
};

/**
 * @brief 注册 AstarPlanner 插件
 *
 * 这个函数会被 plugin_loader.cpp 调用，确保插件被注册到插件注册表。
 */
void registerAstarPlannerPlugin();

} // namespace adapter
} // namespace astar_planner

