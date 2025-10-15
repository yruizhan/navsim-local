#include "esdf_builder_plugin_register.hpp"
#include "esdf_builder_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <iostream>

namespace navsim {
namespace plugins {
namespace perception {

// 插件自注册函数
void registerESDFBuilderPlugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering ESDFBuilder plugin..." << std::endl;
    plugin::PerceptionPluginRegistry::getInstance().registerPlugin(
        "ESDFBuilder",
        []() -> std::unique_ptr<plugin::PerceptionPluginInterface> {
          return std::make_unique<ESDFBuilderPlugin>();
        });
    registered = true;
    std::cout << "[DEBUG] ESDFBuilder plugin registered successfully" << std::endl;
  }
}

} // namespace perception
} // namespace plugins
} // namespace navsim

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerESDFBuilderPlugin() {
    navsim::plugins::perception::registerESDFBuilderPlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct ESDFBuilderPluginInitializer {
    ESDFBuilderPluginInitializer() {
      navsim::plugins::perception::registerESDFBuilderPlugin();
    }
  };
  static ESDFBuilderPluginInitializer esdf_builder_plugin_initializer;
}

