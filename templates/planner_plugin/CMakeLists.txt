# {{PLUGIN_NAME}} Planner Plugin
#
# {{DESCRIPTION}}

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
#
# TODO: 如果您的算法有多个源文件，在这里添加
# 示例：如果您复制了 JPS 算法，可能需要添加：
#   algorithm/graph_search.cpp
#   algorithm/jps_data_structures.cpp
#   algorithm/other_helper.cpp
add_library({{PLUGIN_NAME_SNAKE}}_plugin SHARED
    algorithm/{{PLUGIN_NAME_SNAKE}}.cpp
    adapter/{{PLUGIN_NAME_SNAKE}}_plugin.cpp
    adapter/register.cpp)

# 设置动态库输出名称和版本
set_target_properties({{PLUGIN_NAME_SNAKE}}_plugin PROPERTIES
    OUTPUT_NAME "{{PLUGIN_NAME_SNAKE}}_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories({{PLUGIN_NAME_SNAKE}}_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/navsim_plugins/{{PLUGIN_NAME_SNAKE}}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/adapter
        ${CMAKE_SOURCE_DIR}/platform/include  # Platform headers (required)
)

# TODO: 如果您的算法需要感知数据，取消注释相应的行
# 示例：如果需要 ESDF 地图（如 JPS 算法）
# target_include_directories({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE
#     ${CMAKE_SOURCE_DIR}/plugins/perception/esdf_builder/include)

# 示例：如果需要栅格地图（如 A* 算法）
# target_include_directories({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE
#     ${CMAKE_SOURCE_DIR}/plugins/perception/grid_map_builder/include)

# ========== 依赖项 ==========
# TODO: 如果您的算法需要额外的库，取消注释相应的行

# 示例：如果需要 Boost（如 JPS 算法使用优先队列）
# find_package(Boost REQUIRED)

# 示例：如果需要 OpenCV（图像处理）
# find_package(OpenCV REQUIRED)

# 示例：如果需要 PCL（点云处理）
# find_package(PCL REQUIRED)

target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen
)

# TODO: 如果您的算法需要额外的库，取消注释相应的行

# 示例：如果需要 Boost
# target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE Boost::boost)

# 示例：如果需要 OpenCV
# target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE ${OpenCV_LIBS})

# 示例：如果需要 PCL
# target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE ${PCL_LIBRARIES})

# 示例：如果需要 ESDF 地图（如 JPS 算法）
# target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE esdf_builder_plugin)

# 示例：如果需要栅格地图（如 A* 算法）
# target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE grid_map_builder_plugin)

# ========== 编译选项 ==========
target_compile_features({{PLUGIN_NAME_SNAKE}}_plugin PUBLIC cxx_std_17)

# 定义宏
target_compile_definitions({{PLUGIN_NAME_SNAKE}}_plugin
    PRIVATE
        PLUGIN_NAME="{{PLUGIN_NAME}}"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
if(INSTALL_PLUGINS)
    install(TARGETS {{PLUGIN_NAME_SNAKE}}_plugin
        EXPORT {{PLUGIN_NAME}}PluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins)

    install(DIRECTORY algorithm/ adapter/
        DESTINATION include/navsim_plugins/{{PLUGIN_NAME_SNAKE}}
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT {{PLUGIN_NAME}}PluginTargets
        FILE {{PLUGIN_NAME}}PluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    {{PLUGIN_NAME}} planner plugin configured")

