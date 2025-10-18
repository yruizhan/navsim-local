#include "jps_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <memory>

namespace jps_planner {
namespace adapter {

// 插件自注册函数
void registerJpsPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering JpsPlanner plugin..." << std::endl;
    navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "JpsPlanner",
        []() -> std::shared_ptr<navsim::plugin::PlannerPluginInterface> {
          return std::make_shared<JpsPlannerPlugin>();
        });
    registered = true;
    std::cout << "[DEBUG] JpsPlanner plugin registered successfully" << std::endl;
  }
}

} // namespace adapter
} // namespace jps_planner

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerJpsPlannerPlugin() {
    jps_planner::adapter::registerJpsPlannerPlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct JpsPlannerPluginInitializer {
    JpsPlannerPluginInitializer() {
      jps_planner::adapter::registerJpsPlannerPlugin();
    }
  };
  static JpsPlannerPluginInitializer g_jps_planner_initializer;
}

