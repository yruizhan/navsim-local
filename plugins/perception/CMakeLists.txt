# NavSim Perception Plugins
#
# Each perception plugin is an independent sub-project.
# Users can enable/disable specific plugins.

message(STATUS "--- Configuring Perception Plugins ---")

# ========== 插件选项 ==========
option(BUILD_GRID_MAP_BUILDER_PLUGIN "Build GridMapBuilder plugin" ON)
option(BUILD_ESDF_BUILDER_PLUGIN "Build ESDF Builder plugin" OFF)

# ========== 插件列表 ==========
set(PERCEPTION_PLUGIN_LIBS "")

# GridMapBuilder 插件
if(BUILD_GRID_MAP_BUILDER_PLUGIN)
    message(STATUS "  [+] GridMapBuilder plugin")
    add_subdirectory(grid_map_builder)
    list(APPEND PERCEPTION_PLUGIN_LIBS grid_map_builder_plugin)
else()
    message(STATUS "  [-] GridMapBuilder plugin (disabled)")
endif()

# ESDF Builder 插件（示例 - 未实现）
if(BUILD_ESDF_BUILDER_PLUGIN)
    message(STATUS "  [+] ESDF Builder plugin")
    # add_subdirectory(esdf_builder)
    # list(APPEND PERCEPTION_PLUGIN_LIBS esdf_builder_plugin)
    message(WARNING "ESDF Builder plugin not implemented yet")
else()
    message(STATUS "  [-] ESDF Builder plugin (disabled)")
endif()

# ========== 创建感知插件聚合库 ==========
add_library(navsim_perception_plugins INTERFACE)

if(PERCEPTION_PLUGIN_LIBS)
    target_link_libraries(navsim_perception_plugins INTERFACE
        ${PERCEPTION_PLUGIN_LIBS})
    message(STATUS "Perception plugins: ${PERCEPTION_PLUGIN_LIBS}")
else()
    message(STATUS "No perception plugins enabled")
endif()

message(STATUS "--- Perception Plugins Configuration Complete ---")

