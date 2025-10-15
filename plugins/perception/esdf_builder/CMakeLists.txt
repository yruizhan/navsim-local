# ESDF Builder Plugin
add_library(esdf_builder_plugin SHARED
  src/esdf_builder_plugin.cpp
  src/register.cpp
)

target_include_directories(esdf_builder_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/navsim_plugins/esdf_builder>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(esdf_builder_plugin
  PUBLIC
    navsim_plugin_framework
)

# 设置编译选项
target_compile_features(esdf_builder_plugin PUBLIC cxx_std_17)

# 定义宏
target_compile_definitions(esdf_builder_plugin PUBLIC BUILD_ESDF_BUILDER_PLUGIN)

target_compile_definitions(esdf_builder_plugin
  PRIVATE
    PLUGIN_NAME="ESDFBuilder"
    PLUGIN_VERSION="1.0.0"
)

message(STATUS "    ESDFBuilder plugin configured")

