/**
 * @file register.cpp
 * @brief Plugin registration implementation for JPS Planner
 */

#include "jps_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"

namespace navsim {
namespace plugins {
namespace planning {

// 插件自注册函数
void registerJpsPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "JpsPlanner",
        []() -> std::shared_ptr<plugin::PlannerPluginInterface> {
          return std::make_shared<JPSPlannerPlugin>();
        });
    registered = true;
  }
}

} // namespace planning
} // namespace plugins
} // namespace navsim

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerJpsPlannerPlugin() {
    navsim::plugins::planning::registerJpsPlannerPlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
struct JpsPlannerPluginInitializer {
  JpsPlannerPluginInitializer() {
    navsim::plugins::planning::registerJpsPlannerPlugin();
  }
};
static JpsPlannerPluginInitializer g_jps_planner_initializer;
}

