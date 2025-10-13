/**
 * @file plugin_loader.cpp
 * @brief 插件加载器 - 强制引用所有插件的注册函数
 * 
 * 这个文件的目的是确保链接器包含所有插件的静态注册器。
 * 通过显式调用每个插件的注册函数，我们强制链接器包含这些符号。
 */

// 包含所有插件的注册函数声明
#include "perception/grid_map_builder/include/grid_map_builder_plugin_register.hpp"
#include "planning/straight_line/include/straight_line_planner_plugin_register.hpp"
#include "planning/astar/include/astar_planner_plugin_register.hpp"

namespace navsim {
namespace plugins {

/**
 * @brief 加载所有内置插件
 * 
 * 这个函数会被 plugin_init.cpp 调用，确保所有插件被注册。
 */
void loadAllBuiltinPlugins() {
  // 调用所有插件的注册函数
  perception::registerGridMapBuilderPlugin();
  planning::registerStraightLinePlannerPlugin();
  planning::registerAStarPlannerPlugin();
}

} // namespace plugins
} // namespace navsim

