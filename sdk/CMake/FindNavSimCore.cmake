# FindNavSimCore.cmake
#
# 查找NavSim核心库和头文件
#
# 设置的变量：
#   NavSimCore_FOUND - 是否找到NavSim核心库
#   NavSimCore_INCLUDE_DIRS - 包含目录
#   NavSimCore_LIBRARIES - 链接库
#   NavSimCore_VERSION - 版本号

# 查找头文件
find_path(NavSimCore_INCLUDE_DIR
    NAMES plugin/framework/planner_plugin_interface.hpp
    PATHS
        ${NAVSIM_ROOT}/include
        ${CMAKE_INSTALL_PREFIX}/include
        /usr/local/include
        /usr/include
    PATH_SUFFIXES
        navsim
        navsim-local)

# 查找库文件
find_library(NavSimCore_PLUGIN_FRAMEWORK_LIBRARY
    NAMES navsim_plugin_framework
    PATHS
        ${NAVSIM_ROOT}/lib
        ${CMAKE_INSTALL_PREFIX}/lib
        /usr/local/lib
        /usr/lib
    PATH_SUFFIXES
        navsim)

find_library(NavSimCore_PROTO_LIBRARY
    NAMES navsim_proto
    PATHS
        ${NAVSIM_ROOT}/lib
        ${CMAKE_INSTALL_PREFIX}/lib
        /usr/local/lib
        /usr/lib
    PATH_SUFFIXES
        navsim)

# 查找版本信息
if(NavSimCore_INCLUDE_DIR)
    file(STRINGS "${NavSimCore_INCLUDE_DIR}/core/version.hpp"
         NavSimCore_VERSION_LINES
         REGEX "#define NAVSIM_VERSION")

    if(NavSimCore_VERSION_LINES)
        string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+"
               NavSimCore_VERSION "${NavSimCore_VERSION_LINES}")
    endif()
endif()

# 设置标准变量
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NavSimCore
    REQUIRED_VARS
        NavSimCore_INCLUDE_DIR
        NavSimCore_PLUGIN_FRAMEWORK_LIBRARY
        NavSimCore_PROTO_LIBRARY
    VERSION_VAR NavSimCore_VERSION)

if(NavSimCore_FOUND)
    set(NavSimCore_INCLUDE_DIRS ${NavSimCore_INCLUDE_DIR})
    set(NavSimCore_LIBRARIES
        ${NavSimCore_PLUGIN_FRAMEWORK_LIBRARY}
        ${NavSimCore_PROTO_LIBRARY})

    # 创建导入目标
    if(NOT TARGET NavSim::Core)
        add_library(NavSim::Core INTERFACE IMPORTED)
        set_target_properties(NavSim::Core PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${NavSimCore_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${NavSimCore_LIBRARIES}")
    endif()
endif()

mark_as_advanced(
    NavSimCore_INCLUDE_DIR
    NavSimCore_PLUGIN_FRAMEWORK_LIBRARY
    NavSimCore_PROTO_LIBRARY)