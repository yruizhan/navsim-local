#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "plugin/data/planning_result.hpp"
#include "core/planning_context.hpp"
#include "../algorithm/{{PLUGIN_NAME_SNAKE}}.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

namespace {{NAMESPACE}} {
namespace adapter {

/**
 * @brief {{PLUGIN_NAME}} 插件适配器
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
class {{PLUGIN_NAME}}Plugin : public navsim::plugin::PlannerPluginInterface {
public:
  /**
   * @brief 构造函数
   */
  {{PLUGIN_NAME}}Plugin();
  
  /**
   * @brief 析构函数
   */
  ~{{PLUGIN_NAME}}Plugin() override = default;
  
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
   * @param result 规划结果（输出）
   * @return 规划是否成功
   */
  bool plan(
      const navsim::planning::PlanningContext& context,
      navsim::plugin::PlanningResult& result) override;
  
  /**
   * @brief 重置插件状态
   */
  void reset() override;
  
private:
  // 算法核心实例
  std::unique_ptr<algorithm::{{PLUGIN_NAME}}> planner_;
  
  // 配置
  algorithm::{{PLUGIN_NAME}}::Config config_;
  
  // 是否已初始化
  bool initialized_ = false;
  
  /**
   * @brief 转换平台数据 → 算法数据
   */
  Eigen::Vector3d convertPose(const navsim::planning::Pose2d& pose) const;
  
  /**
   * @brief 转换算法数据 → 平台数据
   */
  navsim::planning::Trajectory convertTrajectory(
      const std::vector<algorithm::{{PLUGIN_NAME}}::Waypoint>& path) const;
};

} // namespace adapter
} // namespace {{NAMESPACE}}

