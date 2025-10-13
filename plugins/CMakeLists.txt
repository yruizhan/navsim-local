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
# 创建一个静态库来强制引用所有插件
add_library(navsim_builtin_plugins STATIC
    plugin_loader.cpp)

target_include_directories(navsim_builtin_plugins
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(navsim_builtin_plugins
    PUBLIC
        navsim_perception_plugins
        navsim_planning_plugins)

target_compile_features(navsim_builtin_plugins PUBLIC cxx_std_17)

message(STATUS "=== Built-in Plugins Configuration Complete ===")

