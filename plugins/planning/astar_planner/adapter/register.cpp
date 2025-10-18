#include "astar_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <memory>

namespace astar_planner {
namespace adapter {

// 插件自注册函数
void registerAstarPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering AstarPlanner plugin..." << std::endl;
    navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "AstarPlanner",
        []() -> std::shared_ptr<navsim::plugin::PlannerPluginInterface> {
          return std::make_shared<AstarPlannerPlugin>();
        });
    registered = true;
    std::cout << "[DEBUG] AstarPlanner plugin registered successfully" << std::endl;
  }
}

} // namespace adapter
} // namespace astar_planner

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerAstarPlannerPlugin() {
    astar_planner::adapter::registerAstarPlannerPlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct AstarPlannerPluginInitializer {
    AstarPlannerPluginInitializer() {
      astar_planner::adapter::registerAstarPlannerPlugin();
    }
  };
  static AstarPlannerPluginInitializer g_astar_planner_initializer;
}

