#pragma once

#include "plugin/framework/perception_plugin_interface.hpp"
#include "plugin/data/perception_input.hpp"
#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <memory>
#include <cmath>
#include <limits>
#include <algorithm>

namespace navsim {
namespace plugins {
namespace perception {

/**
 * @brief ESDF (Euclidean Signed Distance Field) 地图构建插件
 * 
 * 功能：
 * - 从 BEV 障碍物和动态障碍物构建占据栅格
 * - 使用 Felzenszwalb 距离变换算法计算 ESDF
 * - 支持双线性插值和梯度计算
 * 
 * 输入：
 * - context.bev_obstacles (静态障碍物：圆形、矩形、多边形)
 * - context.dynamic_obstacles (动态障碍物)
 * 
 * 输出：
 * - context.esdf_map (ESDF 距离场)
 */
class ESDFBuilderPlugin : public plugin::PerceptionPluginInterface {
public:
  ESDFBuilderPlugin() = default;
  ~ESDFBuilderPlugin() override = default;

  /**
   * @brief 获取插件元数据
   */
  plugin::PerceptionPluginMetadata getMetadata() const override;

  /**
   * @brief 初始化插件
   * @param config 配置参数（JSON 格式）
   *   - resolution: 栅格分辨率 (m/cell)
   *   - map_width: 地图宽度 (m)
   *   - map_height: 地图高度 (m)
   *   - max_distance: 最大距离 (m)
   *   - include_dynamic: 是否包含动态障碍物
   */
  bool initialize(const nlohmann::json& config) override;

  /**
   * @brief 处理感知数据
   * @param input 感知输入（包含 BEV 障碍物、动态障碍物等）
   * @param context 规划上下文（将 ESDF 地图存储到 context.esdf_map）
   */
  bool process(const plugin::PerceptionInput& input, planning::PlanningContext& context) override;

private:
  // ========== 配置参数 ==========
  
  double resolution_ = 0.1;        // 栅格分辨率 (m/cell)
  double map_width_ = 30.0;        // 地图宽度 (m)
  double map_height_ = 30.0;       // 地图高度 (m)
  double max_distance_ = 5.0;      // 最大距离 (m)
  bool include_dynamic_ = true;    // 是否包含动态障碍物

  // ========== 内部数据 ==========
  
  std::vector<uint8_t> occupancy_grid_;  // 临时占据栅格 (0=自由, 100=占据)
  int grid_width_ = 0;                   // 栅格宽度 (cells)
  int grid_height_ = 0;                  // 栅格高度 (cells)

  // ========== 核心算法 ==========
  
  /**
   * @brief 从 BEV 障碍物构建占据栅格
   * @param input 感知输入
   * @param origin 地图原点
   */
  void buildOccupancyGrid(const plugin::PerceptionInput& input, const planning::Point2d& origin);
  
  /**
   * @brief 计算 ESDF 距离场
   * @param esdf_map 输出的 ESDF 地图
   */
  void computeESDF(planning::ESDFMap& esdf_map);
  
  /**
   * @brief Felzenszwalb 距离变换算法（1D）
   * 
   * 这是一个高效的 1D 距离变换算法，时间复杂度 O(n)
   * 
   * @param f_get_val 获取输入值的函数
   * @param f_set_val 设置输出值的函数
   * @param start 起始索引
   * @param end 结束索引
   * @param dim_size 维度大小
   */
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size);
  
  // ========== 辅助函数 ==========
  
  /**
   * @brief 检查点是否在圆形障碍物内
   */
  bool isInCircle(double x, double y, const planning::BEVObstacles::Circle& circle) const;

  /**
   * @brief 检查点是否在矩形障碍物内
   */
  bool isInRectangle(double x, double y, const planning::BEVObstacles::Rectangle& rect) const;

  /**
   * @brief 检查点是否在多边形障碍物内（射线法）
   */
  bool isInPolygon(double x, double y, const planning::BEVObstacles::Polygon& polygon) const;
  
  /**
   * @brief 检查点是否在动态障碍物内
   */
  bool isInDynamicObstacle(double x, double y, const planning::DynamicObstacle& obs) const;
};

} // namespace perception
} // namespace plugins
} // namespace navsim

