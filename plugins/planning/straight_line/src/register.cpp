#include "straight_line_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"

namespace navsim {
namespace plugins {
namespace planning {

// 插件自注册函数
void registerStraightLinePlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "StraightLinePlanner",
        []() -> std::shared_ptr<plugin::PlannerPluginInterface> {
          return std::make_shared<StraightLinePlannerPlugin>();
        });
    registered = true;
  }
}

} // namespace planning
} // namespace plugins
} // namespace navsim

// 静态初始化器 - 确保在程序启动时注册
namespace {
struct StraightLinePlannerPluginInitializer {
  StraightLinePlannerPluginInitializer() {
    navsim::plugins::planning::registerStraightLinePlannerPlugin();
  }
};
static StraightLinePlannerPluginInitializer g_straight_line_planner_initializer;
}

