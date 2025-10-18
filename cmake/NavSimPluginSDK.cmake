# NavSim Plugin SDK
#
# 提供插件开发的标准化构建工具
#
# 使用方式：
# find_package(NavSimPluginSDK REQUIRED)
# navsim_add_plugin(my_plugin ...)

cmake_minimum_required(VERSION 3.16)

# 插件SDK版本
set(NAVSIM_PLUGIN_SDK_VERSION "1.0.0")

# 查找必要的依赖
find_package(PkgConfig QUIET)

# 插件元数据加载函数
function(navsim_load_plugin_metadata)
    # 读取 plugin.json 文件
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/plugin.json")
        file(READ "${CMAKE_CURRENT_SOURCE_DIR}/plugin.json" PLUGIN_JSON_CONTENT)

        # 解析基础字段（简化版JSON解析）
        string(REGEX MATCH "\"name\"[[:space:]]*:[[:space:]]*\"([^\"]+)\"" _ ${PLUGIN_JSON_CONTENT})
        if(CMAKE_MATCH_1)
            set(PLUGIN_NAME ${CMAKE_MATCH_1} PARENT_SCOPE)
            message(STATUS "Plugin name: ${CMAKE_MATCH_1}")
        endif()

        string(REGEX MATCH "\"version\"[[:space:]]*:[[:space:]]*\"([^\"]+)\"" _ ${PLUGIN_JSON_CONTENT})
        if(CMAKE_MATCH_1)
            set(PLUGIN_VERSION ${CMAKE_MATCH_1} PARENT_SCOPE)
            message(STATUS "Plugin version: ${CMAKE_MATCH_1}")
        endif()

        string(REGEX MATCH "\"type\"[[:space:]]*:[[:space:]]*\"([^\"]+)\"" _ ${PLUGIN_JSON_CONTENT})
        if(CMAKE_MATCH_1)
            set(PLUGIN_TYPE ${CMAKE_MATCH_1} PARENT_SCOPE)
            message(STATUS "Plugin type: ${CMAKE_MATCH_1}")
        endif()
    else()
        message(WARNING "plugin.json not found in ${CMAKE_CURRENT_SOURCE_DIR}")
    endif()
endfunction()

# 查找NavSim核心库
function(navsim_find_core_libraries)
    # 首先尝试查找已安装的NavSim
    find_package(NavSimCore QUIET)

    if(NOT NavSimCore_FOUND)
        # 如果没有找到，尝试在源码树中查找
        get_filename_component(NAVSIM_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
        if(EXISTS "${NAVSIM_SOURCE_DIR}/include/plugin/framework")
            message(STATUS "Found NavSim source tree at: ${NAVSIM_SOURCE_DIR}")

            # 设置包含目录
            set(NAVSIM_INCLUDE_DIRS
                "${NAVSIM_SOURCE_DIR}/include"
                "${NAVSIM_SOURCE_DIR}/third_party/nlohmann"
                PARENT_SCOPE)

            # 查找已构建的库
            find_library(NAVSIM_PLUGIN_FRAMEWORK_LIB
                NAMES navsim_plugin_framework
                PATHS "${NAVSIM_SOURCE_DIR}/build"
                NO_DEFAULT_PATH)

            find_library(NAVSIM_PROTO_LIB
                NAMES navsim_proto
                PATHS "${NAVSIM_SOURCE_DIR}/build"
                NO_DEFAULT_PATH)

            if(NAVSIM_PLUGIN_FRAMEWORK_LIB AND NAVSIM_PROTO_LIB)
                set(NAVSIM_LIBRARIES
                    ${NAVSIM_PLUGIN_FRAMEWORK_LIB}
                    ${NAVSIM_PROTO_LIB}
                    PARENT_SCOPE)
                set(NAVSIM_FOUND TRUE PARENT_SCOPE)
                message(STATUS "Found NavSim libraries: ${NAVSIM_PLUGIN_FRAMEWORK_LIB}")
            else()
                message(STATUS "NavSim libraries not built. Please build the main project first.")
                set(NAVSIM_FOUND FALSE PARENT_SCOPE)
            endif()
        else()
            message(FATAL_ERROR "NavSim source tree not found. Please set NAVSIM_ROOT or install NavSim.")
        endif()
    else()
        set(NAVSIM_FOUND TRUE PARENT_SCOPE)
    endif()
endfunction()

# 创建插件目标
function(navsim_add_plugin target_name)
    set(options STANDALONE)
    set(oneValueArgs TYPE CONFIG_FILE)
    set(multiValueArgs SOURCES HEADERS DEPENDENCIES CONFIG_FILES)

    cmake_parse_arguments(PLUGIN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT PLUGIN_SOURCES)
        message(FATAL_ERROR "navsim_add_plugin: SOURCES is required")
    endif()

    # 查找NavSim核心库
    navsim_find_core_libraries()
    if(NOT NAVSIM_FOUND)
        message(FATAL_ERROR "NavSim core libraries not found")
    endif()

    # 创建插件库
    if(PLUGIN_STANDALONE)
        # 独立插件：创建可执行文件用于测试
        add_executable(${target_name}_standalone ${PLUGIN_SOURCES})
        set(plugin_target ${target_name}_standalone)

        # 同时创建动态库用于集成
        add_library(${target_name} SHARED ${PLUGIN_SOURCES})
        set_target_properties(${target_name} PROPERTIES
            OUTPUT_NAME "${target_name}"
            VERSION "${PLUGIN_VERSION}"
            SOVERSION 1)
    else()
        # 普通插件：只创建动态库
        add_library(${target_name} SHARED ${PLUGIN_SOURCES})
        set(plugin_target ${target_name})
        set_target_properties(${target_name} PROPERTIES
            OUTPUT_NAME "${target_name}"
            VERSION "${PLUGIN_VERSION}"
            SOVERSION 1)
    endif()

    # 设置包含目录
    target_include_directories(${target_name}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${NAVSIM_INCLUDE_DIRS}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)

    # 链接依赖
    target_link_libraries(${target_name}
        PUBLIC
            ${NAVSIM_LIBRARIES}
        PRIVATE
            ${PLUGIN_DEPENDENCIES})

    # 设置编译特性
    target_compile_features(${target_name} PUBLIC cxx_std_17)

    # 定义插件宏
    target_compile_definitions(${target_name}
        PRIVATE
            PLUGIN_NAME="${PLUGIN_NAME}"
            PLUGIN_VERSION="${PLUGIN_VERSION}")

    # 如果是独立插件，为可执行文件设置相同的属性
    if(PLUGIN_STANDALONE)
        target_include_directories(${target_name}_standalone
            PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/include
                ${NAVSIM_INCLUDE_DIRS})

        target_link_libraries(${target_name}_standalone
            PRIVATE
                ${NAVSIM_LIBRARIES}
                ${PLUGIN_DEPENDENCIES})

        target_compile_features(${target_name}_standalone PRIVATE cxx_std_17)
        target_compile_definitions(${target_name}_standalone
            PRIVATE
                PLUGIN_NAME="${PLUGIN_NAME}"
                PLUGIN_VERSION="${PLUGIN_VERSION}"
                STANDALONE_BUILD)
    endif()

    # 安装配置文件
    if(PLUGIN_CONFIG_FILES)
        foreach(config_file ${PLUGIN_CONFIG_FILES})
            if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${config_file}")
                install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/${config_file}"
                    DESTINATION share/navsim/plugins/${PLUGIN_NAME}/config)
            endif()
        endforeach()
    endif()

    message(STATUS "Created plugin target: ${target_name}")
    if(PLUGIN_STANDALONE)
        message(STATUS "Created standalone target: ${target_name}_standalone")
    endif()
endfunction()

# 为插件添加测试
function(navsim_add_plugin_test target_name)
    set(oneValueArgs PLUGIN_TARGET)
    set(multiValueArgs SOURCES DEPENDENCIES)

    cmake_parse_arguments(TEST "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT TEST_SOURCES)
        message(FATAL_ERROR "navsim_add_plugin_test: SOURCES is required")
    endif()

    if(NOT TEST_PLUGIN_TARGET)
        message(FATAL_ERROR "navsim_add_plugin_test: PLUGIN_TARGET is required")
    endif()

    # 查找测试框架
    find_package(GTest QUIET)

    add_executable(${target_name} ${TEST_SOURCES})

    target_include_directories(${target_name}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/test
            ${NAVSIM_INCLUDE_DIRS})

    target_link_libraries(${target_name}
        PRIVATE
            ${TEST_PLUGIN_TARGET}
            ${NAVSIM_LIBRARIES}
            ${TEST_DEPENDENCIES})

    if(GTest_FOUND)
        target_link_libraries(${target_name} PRIVATE GTest::GTest GTest::Main)
    endif()

    target_compile_features(${target_name} PRIVATE cxx_std_17)

    # 添加到测试
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.10")
        include(GoogleTest)
        if(COMMAND gtest_discover_tests)
            gtest_discover_tests(${target_name})
        endif()
    endif()

    message(STATUS "Created plugin test: ${target_name}")
endfunction()

# 为插件添加示例
function(navsim_add_plugin_example target_name)
    set(oneValueArgs PLUGIN_TARGET)
    set(multiValueArgs SOURCES DEPENDENCIES)

    cmake_parse_arguments(EXAMPLE "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT EXAMPLE_SOURCES)
        message(FATAL_ERROR "navsim_add_plugin_example: SOURCES is required")
    endif()

    add_executable(${target_name} ${EXAMPLE_SOURCES})

    target_include_directories(${target_name}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${NAVSIM_INCLUDE_DIRS})

    target_link_libraries(${target_name}
        PRIVATE
            ${EXAMPLE_PLUGIN_TARGET}
            ${NAVSIM_LIBRARIES}
            ${EXAMPLE_DEPENDENCIES})

    target_compile_features(${target_name} PRIVATE cxx_std_17)

    message(STATUS "Created plugin example: ${target_name}")
endfunction()

# 验证插件包结构
function(navsim_validate_plugin_structure)
    set(required_files
        "plugin.json"
        "CMakeLists.txt"
        "include"
        "src")

    foreach(file ${required_files})
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
            message(WARNING "Missing required file/directory: ${file}")
        endif()
    endforeach()

    # 检查plugin.json格式
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/plugin.json")
        file(READ "${CMAKE_CURRENT_SOURCE_DIR}/plugin.json" json_content)

        set(required_fields "name" "version" "type")
        foreach(field ${required_fields})
            if(NOT json_content MATCHES "\"${field}\"")
                message(WARNING "Missing required field in plugin.json: ${field}")
            endif()
        endforeach()
    endif()
endfunction()

message(STATUS "NavSim Plugin SDK ${NAVSIM_PLUGIN_SDK_VERSION} loaded")