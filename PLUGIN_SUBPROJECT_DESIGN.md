# 插件子工程架构设计

**日期**: 2025-10-13  
**目标**: 将感知插件和规划器插件作为独立的 CMake 子工程，方便用户扩展

---

## 📋 设计目标

### 核心目标
1. **插件独立性**: 插件包可以独立编译、测试、分发
2. **用户友好**: 用户只需关注插件代码，无需修改核心系统
3. **灵活扩展**: 支持内置插件和外部插件包
4. **向后兼容**: 保持现有插件正常工作

### 使用场景
- **场景 1**: 用户开发新的规划器插件（如 RRT*）
- **场景 2**: 用户开发新的感知插件（如 ESDF Builder）
- **场景 3**: 第三方提供插件包（如商业规划器）
- **场景 4**: 插件包独立发布和版本管理

---

## 🏗️ 新的目录结构

```
navsim-local/
├── CMakeLists.txt                    # 主 CMake 文件
├── cmake/
│   ├── NavSimPluginConfig.cmake.in   # 插件开发配置模板
│   └── FindNavSimCore.cmake          # 查找核心库
├── include/
│   ├── core/                         # 核心模块（不变）
│   └── plugin/
│       ├── framework/                # 插件框架接口（SDK）
│       ├── data/                     # 数据结构（SDK）
│       └── preprocessing/            # 前置处理（SDK）
├── src/
│   ├── core/                         # 核心实现
│   └── plugin/
│       ├── framework/                # 插件框架实现
│       └── preprocessing/            # 前置处理实现
├── plugins/                          # 插件子工程目录 ⭐ 新增
│   ├── CMakeLists.txt                # 插件总入口
│   ├── perception/                   # 感知插件包 ⭐
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── grid_map_builder/
│   │   │   ├── CMakeLists.txt
│   │   │   ├── include/
│   │   │   │   └── grid_map_builder_plugin.hpp
│   │   │   └── src/
│   │   │       └── grid_map_builder_plugin.cpp
│   │   └── esdf_builder/             # 示例：新插件
│   │       ├── CMakeLists.txt
│   │       └── ...
│   └── planning/                     # 规划器插件包 ⭐
│       ├── CMakeLists.txt
│       ├── README.md
│       ├── straight_line/
│       │   ├── CMakeLists.txt
│       │   ├── include/
│       │   │   └── straight_line_planner_plugin.hpp
│       │   └── src/
│       │       └── straight_line_planner_plugin.cpp
│       ├── astar/
│       │   ├── CMakeLists.txt
│       │   └── ...
│       └── rrt_star/                 # 示例：新插件
│           ├── CMakeLists.txt
│           └── ...
├── external_plugins/                 # 外部插件包 ⭐ 新增
│   └── README.md                     # 如何添加外部插件
└── tests/
    ├── test_plugin_system.cpp
    └── plugins/                      # 插件单元测试 ⭐ 新增
        ├── test_grid_map_builder.cpp
        └── test_astar_planner.cpp
```

---

## 🔧 CMake 架构设计

### 1. 主 CMakeLists.txt（简化版）

```cmake
cmake_minimum_required(VERSION 3.16)
project(navsim_local VERSION 1.0.0 LANGUAGES CXX)

# ========== 选项 ==========
option(BUILD_PLUGINS "Build built-in plugins" ON)
option(BUILD_TESTS "Build tests" ON)

# ========== 核心库 ==========
# Protobuf, ixwebsocket, Eigen3 等依赖（保持不变）
find_package(Protobuf REQUIRED)
find_package(Eigen3 QUIET)

# Proto 库
add_library(navsim_proto STATIC ...)

# 插件框架库（不包含具体插件）
add_library(navsim_plugin_framework STATIC
    src/plugin/framework/perception_plugin_manager.cpp
    src/plugin/framework/planner_plugin_manager.cpp
    src/plugin/framework/config_loader.cpp
    src/plugin/framework/plugin_init.cpp
    src/plugin/preprocessing/bev_extractor.cpp
    src/plugin/preprocessing/dynamic_predictor.cpp
    src/plugin/preprocessing/basic_converter.cpp
    src/plugin/preprocessing/preprocessing_pipeline.cpp)

target_include_directories(navsim_plugin_framework
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${CMAKE_CURRENT_BINARY_DIR}
      third_party/nlohmann)

target_link_libraries(navsim_plugin_framework PUBLIC navsim_proto)

# 核心库
add_library(navsim_core STATIC
    src/core/planning_context.cpp
    src/core/algorithm_manager.cpp
    src/core/websocket_visualizer.cpp)

target_link_libraries(navsim_core PUBLIC navsim_plugin_framework)

# ========== 插件子工程 ==========
if(BUILD_PLUGINS)
  add_subdirectory(plugins)
endif()

# ========== 主程序 ==========
add_executable(navsim_algo
    src/core/main.cpp
    src/core/bridge.cpp)

target_link_libraries(navsim_algo
    PRIVATE
      navsim_core
      navsim_builtin_plugins  # 由 plugins/CMakeLists.txt 提供
      navsim_proto
      ixwebsocket)

# ========== 安装和导出 ==========
install(TARGETS navsim_proto navsim_plugin_framework navsim_core
    EXPORT NavSimCoreTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp")

# 导出配置，供外部插件使用
install(EXPORT NavSimCoreTargets
    FILE NavSimCoreTargets.cmake
    NAMESPACE NavSim::
    DESTINATION lib/cmake/NavSim)

configure_file(cmake/NavSimPluginConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/NavSimPluginConfig.cmake @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/NavSimPluginConfig.cmake
    DESTINATION lib/cmake/NavSim)
```

### 2. plugins/CMakeLists.txt（插件总入口）

```cmake
# 插件总入口
# 收集所有插件并创建统一的库

# 添加感知插件子目录
add_subdirectory(perception)

# 添加规划器插件子目录
add_subdirectory(planning)

# 创建统一的插件库（聚合所有插件）
add_library(navsim_builtin_plugins INTERFACE)

target_link_libraries(navsim_builtin_plugins INTERFACE
    navsim_perception_plugins
    navsim_planning_plugins)
```

### 3. plugins/perception/CMakeLists.txt（感知插件包）

```cmake
# 感知插件包

# 选项：用户可以选择编译哪些插件
option(BUILD_GRID_MAP_BUILDER "Build GridMapBuilder plugin" ON)
option(BUILD_ESDF_BUILDER "Build ESDF Builder plugin" OFF)

# 收集所有感知插件
set(PERCEPTION_PLUGIN_LIBS "")

# GridMapBuilder 插件
if(BUILD_GRID_MAP_BUILDER)
  add_subdirectory(grid_map_builder)
  list(APPEND PERCEPTION_PLUGIN_LIBS grid_map_builder_plugin)
endif()

# ESDF Builder 插件（示例）
if(BUILD_ESDF_BUILDER)
  add_subdirectory(esdf_builder)
  list(APPEND PERCEPTION_PLUGIN_LIBS esdf_builder_plugin)
endif()

# 创建感知插件聚合库
add_library(navsim_perception_plugins INTERFACE)

if(PERCEPTION_PLUGIN_LIBS)
  target_link_libraries(navsim_perception_plugins INTERFACE
      ${PERCEPTION_PLUGIN_LIBS})
endif()
```

### 4. plugins/perception/grid_map_builder/CMakeLists.txt（单个插件）

```cmake
# GridMapBuilder 插件

add_library(grid_map_builder_plugin STATIC
    src/grid_map_builder_plugin.cpp)

target_include_directories(grid_map_builder_plugin
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/plugins/perception/grid_map_builder>)

# 链接到插件框架
target_link_libraries(grid_map_builder_plugin
    PUBLIC
      NavSim::navsim_plugin_framework  # 使用导出的目标
    PRIVATE
      Eigen3::Eigen)

target_compile_features(grid_map_builder_plugin PUBLIC cxx_std_17)

# 安装（可选）
install(TARGETS grid_map_builder_plugin
    EXPORT GridMapBuilderPluginTargets
    LIBRARY DESTINATION lib/plugins
    ARCHIVE DESTINATION lib/plugins)

install(DIRECTORY include/
    DESTINATION include/plugins/perception/grid_map_builder
    FILES_MATCHING PATTERN "*.hpp")
```

### 5. plugins/planning/CMakeLists.txt（规划器插件包）

```cmake
# 规划器插件包

option(BUILD_STRAIGHT_LINE_PLANNER "Build StraightLine planner" ON)
option(BUILD_ASTAR_PLANNER "Build A* planner" ON)
option(BUILD_RRT_STAR_PLANNER "Build RRT* planner" OFF)

set(PLANNING_PLUGIN_LIBS "")

if(BUILD_STRAIGHT_LINE_PLANNER)
  add_subdirectory(straight_line)
  list(APPEND PLANNING_PLUGIN_LIBS straight_line_planner_plugin)
endif()

if(BUILD_ASTAR_PLANNER)
  add_subdirectory(astar)
  list(APPEND PLANNING_PLUGIN_LIBS astar_planner_plugin)
endif()

if(BUILD_RRT_STAR_PLANNER)
  add_subdirectory(rrt_star)
  list(APPEND PLANNING_PLUGIN_LIBS rrt_star_planner_plugin)
endif()

add_library(navsim_planning_plugins INTERFACE)

if(PLANNING_PLUGIN_LIBS)
  target_link_libraries(navsim_planning_plugins INTERFACE
      ${PLANNING_PLUGIN_LIBS})
endif()
```

---

## 📦 外部插件开发指南

### 外部插件项目结构

```
my_custom_planner/
├── CMakeLists.txt
├── include/
│   └── my_custom_planner_plugin.hpp
└── src/
    └── my_custom_planner_plugin.cpp
```

### 外部插件 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_custom_planner VERSION 1.0.0 LANGUAGES CXX)

# 查找 NavSim 核心库
find_package(NavSim REQUIRED)

# 创建插件库
add_library(my_custom_planner_plugin SHARED
    src/my_custom_planner_plugin.cpp)

target_include_directories(my_custom_planner_plugin
    PUBLIC include)

# 链接到 NavSim 插件框架
target_link_libraries(my_custom_planner_plugin
    PUBLIC NavSim::navsim_plugin_framework)

target_compile_features(my_custom_planner_plugin PUBLIC cxx_std_17)

# 安装
install(TARGETS my_custom_planner_plugin
    LIBRARY DESTINATION lib/navsim_plugins
    ARCHIVE DESTINATION lib/navsim_plugins)
```

### 编译外部插件

```bash
# 1. 安装 NavSim 核心库
cd navsim-local
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build
sudo cmake --install build

# 2. 编译外部插件
cd my_custom_planner
cmake -B build -DCMAKE_PREFIX_PATH=/usr/local
cmake --build build
sudo cmake --install build
```

---

## 🔄 迁移步骤

### 步骤 1: 重构 CMake 结构
- [ ] 创建 `plugins/` 目录结构
- [ ] 拆分 `navsim_plugin_system` 为 `navsim_plugin_framework` + 插件库
- [ ] 移动插件代码到各自的子目录

### 步骤 2: 更新插件注册机制
- [ ] 修改 `plugin_init.cpp` 为自动发现机制
- [ ] 每个插件子工程提供注册函数

### 步骤 3: 创建插件 SDK
- [ ] 导出核心库和插件框架
- [ ] 创建 `NavSimPluginConfig.cmake`
- [ ] 编写外部插件开发文档

### 步骤 4: 测试和验证
- [ ] 验证内置插件正常工作
- [ ] 创建示例外部插件
- [ ] 更新文档和示例

---

## ✅ 优势总结

| 特性 | 当前架构 | 子工程架构 |
|------|---------|-----------|
| 插件独立性 | ❌ 耦合在一起 | ✅ 完全独立 |
| 编译速度 | ❌ 修改任何插件都要全量编译 | ✅ 只编译修改的插件 |
| 用户扩展 | ⚠️ 需要修改核心 CMake | ✅ 独立 CMake 项目 |
| 插件分发 | ❌ 无法独立分发 | ✅ 可独立打包分发 |
| 版本管理 | ❌ 统一版本 | ✅ 插件独立版本 |
| 测试隔离 | ⚠️ 测试耦合 | ✅ 插件独立测试 |


