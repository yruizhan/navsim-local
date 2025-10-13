#include "plugin/framework/plugin_registry.hpp"

namespace navsim {
namespace plugin {

/**
 * @brief 初始化所有插件
 *
 * 这个函数现在是空的。插件通过 loadAllBuiltinPlugins() 函数注册，
 * 该函数由 AlgorithmManager 在初始化时调用。
 */
void initializeAllPlugins() {
  // 空实现 - 插件通过 loadAllBuiltinPlugins() 注册
}

} // namespace plugin
} // namespace navsim

