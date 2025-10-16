/**
 * @file jps_planner_plugin.hpp
 * @brief JPS Planner Plugin - Plugin interface layer
 *
 * This is the plugin interface layer that connects the JPS algorithm
 * to the NavSim plugin system.
 */

#ifndef JPS_PLANNER_PLUGIN_HPP
#define JPS_PLANNER_PLUGIN_HPP

#include "plugin/framework/planner_plugin_interface.hpp"
#include "core/planning_context.hpp"
#include "esdf_map.hpp"
#include "jps_planner.hpp"
#include "jps_data_structures.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <chrono>

namespace navsim {
namespace plugins {
namespace planning {

/**
 * @brief JPS Planner Plugin
 *
 * This plugin provides Jump Point Search (JPS) path planning functionality.
 * It uses the ESDFMap from the ESDFBuilderPlugin for collision checking.
 *
 * Architecture:
 * - Plugin Interface Layer (this class): Handles plugin lifecycle, config, data conversion
 * - Core Algorithm Layer (JPS::JPSPlanner): Pure algorithm implementation
 * - Data Structure Layer (JPS::*): Data structures and utilities
 */
class JPSPlannerPlugin : public plugin::PlannerPluginInterface {
 public:
  JPSPlannerPlugin() = default;
  ~JPSPlannerPlugin() override = default;

  /**
   * @brief Get plugin metadata
   */
  plugin::PlannerPluginMetadata getMetadata() const override;

  /**
   * @brief Initialize the plugin
   * @param config JSON configuration
   * @return true if initialization succeeded
   */
  bool initialize(const nlohmann::json& config) override;

  /**
   * @brief Plan a path
   * @param context Planning context containing start, goal, and map
   * @param deadline Planning deadline
   * @param result Planning result (output)
   * @return true if planning succeeded
   */
  bool plan(const navsim::planning::PlanningContext& context,
            std::chrono::milliseconds deadline,
            plugin::PlanningResult& result) override;

  /**
   * @brief Check if planner is available
   * @param context Planning context
   * @return Pair of (is_available, reason)
   */
  std::pair<bool, std::string> isAvailable(const navsim::planning::PlanningContext& context) const override;

  /**
   * @brief Reset the planner
   */
  void reset() override;

  /**
   * @brief Get statistics
   */
  nlohmann::json getStatistics() const override;

 private:
  // ========== Configuration ==========

  /**
   * @brief Load configuration from JSON
   * @param config JSON configuration
   * @return true if loading succeeded
   */
  bool loadConfig(const nlohmann::json& config);

  /**
   * @brief Validate configuration
   * @return true if configuration is valid
   */
  bool validateConfig() const;

  // ========== Data Conversion ==========

  /**
   * @brief Convert PlanningContext to JPS input
   * @param context Planning context
   * @param start Output start state (x, y, yaw)
   * @param goal Output goal state (x, y, yaw)
   * @return true if conversion succeeded
   */
  bool convertContextToJPSInput(const navsim::planning::PlanningContext& context,
                                 Eigen::Vector3d& start,
                                 Eigen::Vector3d& goal) const;

  /**
   * @brief Convert JPS output to PlanningResult
   * @param jps_planner JPS planner instance
   * @param result Output planning result
   * @return true if conversion succeeded
   */
  bool convertJPSOutputToResult(const JPS::JPSPlanner& jps_planner,
                                 plugin::PlanningResult& result) const;

  // ========== Member Variables ==========

  // Core algorithm object
  std::unique_ptr<JPS::JPSPlanner> jps_planner_;

  // ESDFMap (from PlanningContext)
  std::shared_ptr<navsim::perception::ESDFMap> esdf_map_;

  // Configuration
  JPS::JPSConfig jps_config_;

  // Plugin state
  bool initialized_ = false;

  // Statistics
  mutable int total_plans_ = 0;
  mutable int successful_plans_ = 0;
  mutable int failed_plans_ = 0;
  mutable double total_planning_time_ms_ = 0.0;

  // Configuration parameters
  bool verbose_ = false;
};

}  // namespace planning
}  // namespace plugins
}  // namespace navsim

#endif  // JPS_PLANNER_PLUGIN_HPP

