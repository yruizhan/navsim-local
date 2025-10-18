/**
 * @file register.cpp
 * @brief Plugin registration for JpsPlanner
 *
 * This file implements the dual registration mechanism:
 * 1. Dynamic registration (for dlsym loading)
 * 2. Static registration (for static linking)
 */

#include "jps_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <memory>

namespace navsim {
namespace plugins {
namespace planning {

/**
 * @brief Register JpsPlanner plugin
 * 
 * This function is called by both dynamic and static registration mechanisms.
 */
void registerJPSPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "JpsPlanner",
        []() -> std::shared_ptr<navsim::plugin::PlannerPluginInterface> {
          return std::make_shared<JPSPlannerPlugin>();
        });
    registered = true;
  }
}

}  // namespace planning
}  // namespace plugins
}  // namespace navsim

// ========== Dynamic Registration (for dlsym) ==========

extern "C" {
  /**
   * @brief Dynamic registration function
   * 
   * This function is called by DynamicPluginLoader::loadPlugin()
   * when the plugin is loaded via dlsym.
   * 
   * Function name must match: register{PluginName}Plugin
   * Example: JpsPlanner -> registerJpsPlannerPlugin
   */
  void registerJpsPlannerPlugin() {
    navsim::plugins::planning::registerJPSPlannerPlugin();
  }
}

// ========== Static Registration (for static linking) ==========

namespace {
  /**
   * @brief Static registration initializer
   * 
   * This struct's constructor is called before main() when the plugin
   * is statically linked. It serves as a fallback registration mechanism.
   */
  struct JPSPlannerPluginInitializer {
    JPSPlannerPluginInitializer() {
      navsim::plugins::planning::registerJPSPlannerPlugin();
    }
  };
  
  // Global instance triggers registration before main()
  static JPSPlannerPluginInitializer g_jps_planner_initializer;
}
