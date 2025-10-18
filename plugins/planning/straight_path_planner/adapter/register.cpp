#include "straight_path_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <memory>

namespace straight_path_planner {
namespace adapter {

// 插件自注册函数
void registerStraightPathPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering StraightPathPlanner plugin..." << std::endl;
    navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "StraightPathPlanner",
        []() -> std::shared_ptr<navsim::plugin::PlannerPluginInterface> {
          return std::make_shared<StraightPathPlannerPlugin>();
        });
    registered = true;
    std::cout << "[DEBUG] StraightPathPlanner plugin registered successfully" << std::endl;
  }
}

} // namespace adapter
} // namespace straight_path_planner

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerStraightPathPlannerPlugin() {
    straight_path_planner::adapter::registerStraightPathPlannerPlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct StraightPathPlannerPluginInitializer {
    StraightPathPlannerPluginInitializer() {
      straight_path_planner::adapter::registerStraightPathPlannerPlugin();
    }
  };
  static StraightPathPlannerPluginInitializer g_straight_path_planner_initializer;
}

