#include "astar_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"

namespace navsim {
namespace plugins {
namespace planning {

// 插件自注册函数
void registerAStarPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "AStarPlanner",
        []() -> std::shared_ptr<plugin::PlannerPluginInterface> {
          return std::make_shared<AStarPlannerPlugin>();
        });
    registered = true;
  }
}

} // namespace planning
} // namespace plugins
} // namespace navsim

// 静态初始化器 - 确保在程序启动时注册
namespace {
struct AStarPlannerPluginInitializer {
  AStarPlannerPluginInitializer() {
    navsim::plugins::planning::registerAStarPlannerPlugin();
  }
};
static AStarPlannerPluginInitializer g_astar_planner_initializer;
}

