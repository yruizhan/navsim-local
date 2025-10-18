# AstarPlanner Planner Plugin
#
# A* path planner with occupancy grid map

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
#
# TODO: 如果您的算法有多个源文件，在这里添加
# 示例：如果您复制了 JPS 算法，可能需要添加：
#   algorithm/graph_search.cpp
#   algorithm/jps_data_structures.cpp
#   algorithm/other_helper.cpp
add_library(astar_planner_plugin SHARED
    algorithm/astar_planner.cpp
    adapter/astar_planner_plugin.cpp
    adapter/register.cpp)

# 设置动态库输出名称和版本
set_target_properties(astar_planner_plugin PROPERTIES
    OUTPUT_NAME "astar_planner_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories(astar_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/navsim_plugins/astar_planner>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/adapter
        ${CMAKE_SOURCE_DIR}/platform/include  # Platform headers (required)
)

# TODO: 如果您的算法需要感知数据，取消注释相应的行
# 示例：如果需要 ESDF 地图（如 JPS 算法）
# target_include_directories(astar_planner_plugin PRIVATE
#     ${CMAKE_SOURCE_DIR}/plugins/perception/esdf_builder/include)

# 示例：如果需要栅格地图（如 A* 算法）
# target_include_directories(astar_planner_plugin PRIVATE
#     ${CMAKE_SOURCE_DIR}/plugins/perception/grid_map_builder/include)

# ========== 依赖项 ==========
# TODO: 如果您的算法需要额外的库，取消注释相应的行

# 示例：如果需要 Boost（如 JPS 算法使用优先队列）
# find_package(Boost REQUIRED)

# 示例：如果需要 OpenCV（图像处理）
# find_package(OpenCV REQUIRED)

# 示例：如果需要 PCL（点云处理）
# find_package(PCL REQUIRED)

target_link_libraries(astar_planner_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen
)

# TODO: 如果您的算法需要额外的库，取消注释相应的行

# 示例：如果需要 Boost
# target_link_libraries(astar_planner_plugin PRIVATE Boost::boost)

# 示例：如果需要 OpenCV
# target_link_libraries(astar_planner_plugin PRIVATE ${OpenCV_LIBS})

# 示例：如果需要 PCL
# target_link_libraries(astar_planner_plugin PRIVATE ${PCL_LIBRARIES})

# 示例：如果需要 ESDF 地图（如 JPS 算法）
# target_link_libraries(astar_planner_plugin PRIVATE esdf_builder_plugin)

# 示例：如果需要栅格地图（如 A* 算法）
# target_link_libraries(astar_planner_plugin PRIVATE grid_map_builder_plugin)

# ========== 编译选项 ==========
target_compile_features(astar_planner_plugin PUBLIC cxx_std_17)

# 定义宏
target_compile_definitions(astar_planner_plugin
    PRIVATE
        PLUGIN_NAME="AstarPlanner"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
if(INSTALL_PLUGINS)
    install(TARGETS astar_planner_plugin
        EXPORT AstarPlannerPluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins)

    install(DIRECTORY algorithm/ adapter/
        DESTINATION include/navsim_plugins/astar_planner
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT AstarPlannerPluginTargets
        FILE AstarPlannerPluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    AstarPlanner planner plugin configured")

