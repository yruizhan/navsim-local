#pragma once

namespace navsim {
namespace plugins {

/**
 * @brief 加载所有内置插件
 * 
 * 这个函数由插件系统调用，用于注册所有内置插件。
 * 实现在 plugins/plugin_loader.cpp 中。
 */
void loadAllBuiltinPlugins();

} // namespace plugins
} // namespace navsim

