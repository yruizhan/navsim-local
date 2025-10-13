# NavSim Built-in Plugins
#
# This file manages all built-in plugins as independent sub-projects.
# Users can enable/disable specific plugins via CMake options.

message(STATUS "=== Configuring NavSim Built-in Plugins ===")

# ========== 插件选项 ==========
option(BUILD_PERCEPTION_PLUGINS "Build perception plugins" ON)
option(BUILD_PLANNING_PLUGINS "Build planning plugins" ON)

# ========== 感知插件 ==========
if(BUILD_PERCEPTION_PLUGINS)
    message(STATUS "Building perception plugins...")
    add_subdirectory(perception)
else()
    message(STATUS "Skipping perception plugins")
    # 创建空的接口库
    add_library(navsim_perception_plugins INTERFACE)
endif()

# ========== 规划器插件 ==========
if(BUILD_PLANNING_PLUGINS)
    message(STATUS "Building planning plugins...")
    add_subdirectory(planning)
else()
    message(STATUS "Skipping planning plugins")
    # 创建空的接口库
    add_library(navsim_planning_plugins INTERFACE)
endif()

# ========== 统一的插件库 ==========
# 聚合所有插件，供主程序链接
add_library(navsim_builtin_plugins INTERFACE)

target_link_libraries(navsim_builtin_plugins INTERFACE
    navsim_perception_plugins
    navsim_planning_plugins)

message(STATUS "=== Built-in Plugins Configuration Complete ===")

