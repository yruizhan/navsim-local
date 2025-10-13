# NavSim Built-in Plugins

本目录包含 NavSim 的内置插件，每个插件都是独立的 CMake 子工程。

---

## 📁 目录结构

```
plugins/
├── CMakeLists.txt              # 插件总入口
├── perception/                 # 感知插件包
│   ├── CMakeLists.txt
│   ├── grid_map_builder/       # GridMapBuilder 插件
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── grid_map_builder_plugin.hpp
│   │   └── src/
│   │       └── grid_map_builder_plugin.cpp
│   └── esdf_builder/           # ESDF Builder 插件（示例）
│       └── ...
└── planning/                   # 规划器插件包
    ├── CMakeLists.txt
    ├── straight_line/          # StraightLine 规划器
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   └── straight_line_planner_plugin.hpp
    │   └── src/
    │       └── straight_line_planner_plugin.cpp
    ├── astar/                  # A* 规划器
    │   └── ...
    └── rrt_star/               # RRT* 规划器（示例）
        └── ...
```

---

## 🔧 编译选项

### 全局选项

```bash
# 禁用所有插件
cmake -B build -DBUILD_PLUGINS=OFF

# 只编译感知插件
cmake -B build -DBUILD_PLANNING_PLUGINS=OFF

# 只编译规划器插件
cmake -B build -DBUILD_PERCEPTION_PLUGINS=OFF
```

### 感知插件选项

```bash
# 禁用 GridMapBuilder 插件
cmake -B build -DBUILD_GRID_MAP_BUILDER_PLUGIN=OFF

# 启用 ESDF Builder 插件（需要先实现）
cmake -B build -DBUILD_ESDF_BUILDER_PLUGIN=ON
```

### 规划器插件选项

```bash
# 禁用 StraightLine 规划器
cmake -B build -DBUILD_STRAIGHT_LINE_PLANNER_PLUGIN=OFF

# 禁用 A* 规划器
cmake -B build -DBUILD_ASTAR_PLANNER_PLUGIN=OFF

# 启用 RRT* 规划器（需要先实现）
cmake -B build -DBUILD_RRT_STAR_PLANNER_PLUGIN=ON
```

---

## ➕ 添加新的内置插件

### 步骤 1: 创建插件目录

```bash
# 感知插件
mkdir -p plugins/perception/my_plugin/{include,src}

# 规划器插件
mkdir -p plugins/planning/my_planner/{include,src}
```

### 步骤 2: 创建插件 CMakeLists.txt

```cmake
# plugins/perception/my_plugin/CMakeLists.txt

add_library(my_plugin STATIC
    src/my_plugin.cpp)

target_include_directories(my_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/navsim_plugins/my_plugin>)

target_link_libraries(my_plugin
    PUBLIC navsim_plugin_framework)

target_compile_features(my_plugin PUBLIC cxx_std_17)
```

### 步骤 3: 在父 CMakeLists.txt 中添加选项

```cmake
# plugins/perception/CMakeLists.txt

option(BUILD_MY_PLUGIN "Build My Plugin" OFF)

if(BUILD_MY_PLUGIN)
    message(STATUS "  [+] My Plugin")
    add_subdirectory(my_plugin)
    list(APPEND PERCEPTION_PLUGIN_LIBS my_plugin)
endif()
```

### 步骤 4: 实现插件

参考现有插件实现：
- 感知插件: `plugins/perception/grid_map_builder/`
- 规划器插件: `plugins/planning/straight_line/`

---

## 📦 内置插件列表

### 感知插件

| 插件名称 | 状态 | 描述 | CMake 选项 |
|---------|------|------|-----------|
| GridMapBuilder | ✅ 已实现 | 构建栅格占据地图 | `BUILD_GRID_MAP_BUILDER_PLUGIN` |
| ESDF Builder | ⏳ 计划中 | 构建 ESDF 地图 | `BUILD_ESDF_BUILDER_PLUGIN` |

### 规划器插件

| 插件名称 | 状态 | 描述 | CMake 选项 |
|---------|------|------|-----------|
| StraightLine | ✅ 已实现 | 直线轨迹规划器 | `BUILD_STRAIGHT_LINE_PLANNER_PLUGIN` |
| A* | ✅ 已实现 | A* 路径规划器 | `BUILD_ASTAR_PLANNER_PLUGIN` |
| RRT* | ⏳ 计划中 | RRT* 路径规划器 | `BUILD_RRT_STAR_PLANNER_PLUGIN` |

---

## 🔍 插件依赖关系

```
navsim_builtin_plugins (INTERFACE)
├── navsim_perception_plugins (INTERFACE)
│   ├── grid_map_builder_plugin (STATIC)
│   │   └── navsim_plugin_framework
│   └── esdf_builder_plugin (STATIC)
│       └── navsim_plugin_framework
└── navsim_planning_plugins (INTERFACE)
    ├── straight_line_planner_plugin (STATIC)
    │   └── navsim_plugin_framework
    ├── astar_planner_plugin (STATIC)
    │   └── navsim_plugin_framework
    └── rrt_star_planner_plugin (STATIC)
        └── navsim_plugin_framework
```

---

## 📝 开发指南

### 感知插件开发

1. **继承接口**: `PerceptionPluginInterface`
2. **实现方法**:
   - `getMetadata()` - 返回插件元数据
   - `initialize()` - 初始化插件
   - `process()` - 处理感知数据
3. **注册插件**: 在 `plugin_init.cpp` 中注册

### 规划器插件开发

1. **继承接口**: `PlannerPluginInterface`
2. **实现方法**:
   - `getMetadata()` - 返回插件元数据
   - `initialize()` - 初始化规划器
   - `plan()` - 生成轨迹
   - `isAvailable()` - 检查可用性
3. **注册插件**: 在 `plugin_init.cpp` 中注册

---

## 🚀 快速开始

### 编译所有插件

```bash
cd navsim-local
cmake -B build
cmake --build build
```

### 只编译特定插件

```bash
# 只编译 GridMapBuilder 和 StraightLine
cmake -B build \
  -DBUILD_GRID_MAP_BUILDER_PLUGIN=ON \
  -DBUILD_ESDF_BUILDER_PLUGIN=OFF \
  -DBUILD_STRAIGHT_LINE_PLANNER_PLUGIN=ON \
  -DBUILD_ASTAR_PLANNER_PLUGIN=OFF

cmake --build build
```

### 增量编译

```bash
# 修改插件后，只重新编译该插件
cmake --build build --target grid_map_builder_plugin
```

---

## 📚 参考文档

- [插件架构设计](../docs/PLUGIN_ARCHITECTURE_DESIGN.md)
- [插件快速参考](../docs/PLUGIN_QUICK_REFERENCE.md)
- [外部插件开发指南](../external_plugins/README.md)

