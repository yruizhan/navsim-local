# 插件子工程迁移实施计划

**日期**: 2025-10-13  
**目标**: 将现有插件重构为独立的 CMake 子工程  
**预计时间**: 2-3 小时

---

## 📋 迁移概览

### 当前结构
```
navsim-local/
├── include/plugin/plugins/
│   ├── perception/grid_map_builder_plugin.hpp
│   └── planning/
│       ├── straight_line_planner_plugin.hpp
│       └── astar_planner_plugin.hpp
└── src/plugin/plugins/
    ├── perception/grid_map_builder_plugin.cpp
    └── planning/
        ├── straight_line_planner_plugin.cpp
        └── astar_planner_plugin.cpp
```

### 目标结构
```
navsim-local/
└── plugins/
    ├── CMakeLists.txt
    ├── perception/
    │   ├── CMakeLists.txt
    │   └── grid_map_builder/
    │       ├── CMakeLists.txt
    │       ├── include/grid_map_builder_plugin.hpp
    │       └── src/grid_map_builder_plugin.cpp
    └── planning/
        ├── CMakeLists.txt
        ├── straight_line/
        │   ├── CMakeLists.txt
        │   ├── include/straight_line_planner_plugin.hpp
        │   └── src/straight_line_planner_plugin.cpp
        └── astar/
            ├── CMakeLists.txt
            ├── include/astar_planner_plugin.hpp
            └── src/astar_planner_plugin.cpp
```

---

## 🔧 迁移步骤

### Phase 1: 准备工作（10 分钟）

#### 1.1 创建目录结构

```bash
cd navsim-local

# 创建插件子工程目录
mkdir -p plugins/{perception,planning}
mkdir -p plugins/perception/grid_map_builder/{include,src}
mkdir -p plugins/planning/straight_line/{include,src}
mkdir -p plugins/planning/astar/{include,src}

# 创建 CMake 配置目录
mkdir -p cmake

# 创建外部插件示例目录
mkdir -p external_plugins
```

#### 1.2 备份当前代码

```bash
# 创建备份分支
git checkout -b backup/before-plugin-subproject-migration
git add -A
git commit -m "Backup before plugin subproject migration"

# 切换到工作分支
git checkout -b feature/plugin-subproject
```

---

### Phase 2: 移动插件代码（20 分钟）

#### 2.1 移动感知插件

```bash
# GridMapBuilder 插件
mv include/plugin/plugins/perception/grid_map_builder_plugin.hpp \
   plugins/perception/grid_map_builder/include/

mv src/plugin/plugins/perception/grid_map_builder_plugin.cpp \
   plugins/perception/grid_map_builder/src/
```

#### 2.2 移动规划器插件

```bash
# StraightLine 规划器
mv include/plugin/plugins/planning/straight_line_planner_plugin.hpp \
   plugins/planning/straight_line/include/

mv src/plugin/plugins/planning/straight_line_planner_plugin.cpp \
   plugins/planning/straight_line/src/

# A* 规划器
mv include/plugin/plugins/planning/astar_planner_plugin.hpp \
   plugins/planning/astar/include/

mv src/plugin/plugins/planning/astar_planner_plugin.cpp \
   plugins/planning/astar/src/
```

#### 2.3 删除旧目录

```bash
# 删除空目录
rm -rf include/plugin/plugins
rm -rf src/plugin/plugins
```

---

### Phase 3: 更新 CMake 配置（30 分钟）

#### 3.1 创建 CMake 辅助文件

已创建的文件：
- ✅ `cmake/NavSimPluginConfig.cmake.in`
- ✅ `cmake/NavSimPluginHelpers.cmake`

#### 3.2 创建插件 CMakeLists.txt

已创建的文件：
- ✅ `plugins/CMakeLists.txt`
- ✅ `plugins/perception/CMakeLists.txt`
- ✅ `plugins/perception/grid_map_builder/CMakeLists.txt`
- ✅ `plugins/planning/CMakeLists.txt`
- ✅ `plugins/planning/straight_line/CMakeLists.txt`
- ✅ `plugins/planning/astar/CMakeLists.txt`

#### 3.3 修改主 CMakeLists.txt

需要修改的部分：

```cmake
# 旧代码（删除）
add_library(navsim_plugin_system STATIC
    # Plugin framework
    src/plugin/framework/perception_plugin_manager.cpp
    src/plugin/framework/planner_plugin_manager.cpp
    src/plugin/framework/config_loader.cpp
    src/plugin/framework/plugin_init.cpp
    # Preprocessing
    src/plugin/preprocessing/bev_extractor.cpp
    src/plugin/preprocessing/dynamic_predictor.cpp
    src/plugin/preprocessing/basic_converter.cpp
    src/plugin/preprocessing/preprocessing_pipeline.cpp
    # Concrete plugins ← 删除这部分
    src/plugin/plugins/perception/grid_map_builder_plugin.cpp
    src/plugin/plugins/planning/straight_line_planner_plugin.cpp
    src/plugin/plugins/planning/astar_planner_plugin.cpp)

# 新代码（替换）
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

# 添加插件子工程
option(BUILD_PLUGINS "Build built-in plugins" ON)
if(BUILD_PLUGINS)
    add_subdirectory(plugins)
endif()

# 更新链接
target_link_libraries(navsim_core
    PUBLIC
        navsim_proto
        navsim_plugin_framework)  # 改名

target_link_libraries(navsim_algo
    PRIVATE
        navsim_core
        navsim_builtin_plugins  # 新增
        navsim_proto
        ixwebsocket)
```

---

### Phase 4: 更新插件注册（20 分钟）

#### 4.1 修改 plugin_init.cpp

```cpp
// src/plugin/framework/plugin_init.cpp

#include "plugin/framework/plugin_registry.hpp"

// 包含插件头文件（使用新路径）
// 注意：这些头文件现在在 plugins/ 子目录中
// 需要在编译时通过 include_directories 暴露

namespace navsim {
namespace plugin {

void initializeAllPlugins() {
    // 感知插件
    PerceptionPluginRegistry::getInstance().registerPlugin(
        "GridMapBuilder",
        []() -> std::shared_ptr<PerceptionPluginInterface> {
            // 使用前向声明或动态加载
            // 暂时保持现有实现
        });

    // 规划器插件
    PlannerPluginRegistry::getInstance().registerPlugin(
        "StraightLinePlanner",
        []() -> std::shared_ptr<PlannerPluginInterface> {
            // ...
        });

    PlannerPluginRegistry::getInstance().registerPlugin(
        "AStarPlanner",
        []() -> std::shared_ptr<PlannerPluginInterface> {
            // ...
        });
}

} // namespace plugin
} // namespace navsim
```

**注意**: 由于插件现在是独立编译的，需要修改注册机制。有两种方案：

**方案 A**: 在各插件的 CMakeLists.txt 中暴露头文件
```cmake
# plugins/planning/straight_line/CMakeLists.txt
target_include_directories(straight_line_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
```

**方案 B**: 创建插件注册源文件
```cpp
// plugins/planning/straight_line/src/register.cpp
#include "straight_line_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"

namespace {
struct StraightLineRegistrar {
    StraightLineRegistrar() {
        navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
            "StraightLinePlanner",
            []() { return std::make_shared<...>(); });
    }
};
static StraightLineRegistrar g_registrar;
}
```

---

### Phase 5: 测试和验证（30 分钟）

#### 5.1 编译测试

```bash
cd navsim-local
rm -rf build
cmake -B build
cmake --build build
```

预期输出：
```
=== Configuring NavSim Built-in Plugins ===
Building perception plugins...
--- Configuring Perception Plugins ---
  [+] GridMapBuilder plugin
    GridMapBuilder plugin configured
Perception plugins: grid_map_builder_plugin
--- Perception Plugins Configuration Complete ---
Building planning plugins...
--- Configuring Planning Plugins ---
  [+] StraightLine planner plugin
    StraightLine planner plugin configured
  [+] A* planner plugin
    A* planner plugin configured
Planning plugins: straight_line_planner_plugin;astar_planner_plugin
--- Planning Plugins Configuration Complete ---
=== Built-in Plugins Configuration Complete ===
```

#### 5.2 运行测试

```bash
# 运行插件系统测试
./build/test_plugin_system

# 运行主程序
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

#### 5.3 测试选项编译

```bash
# 禁用所有插件
cmake -B build-no-plugins -DBUILD_PLUGINS=OFF
cmake --build build-no-plugins

# 只编译感知插件
cmake -B build-perception-only -DBUILD_PLANNING_PLUGINS=OFF
cmake --build build-perception-only

# 禁用特定插件
cmake -B build-custom \
  -DBUILD_GRID_MAP_BUILDER_PLUGIN=ON \
  -DBUILD_STRAIGHT_LINE_PLANNER_PLUGIN=ON \
  -DBUILD_ASTAR_PLANNER_PLUGIN=OFF
cmake --build build-custom
```

---

### Phase 6: 文档更新（20 分钟）

#### 6.1 更新主 README.md

添加插件子工程说明：

```markdown
## 🔌 插件系统

NavSim 采用插件化架构，插件作为独立的 CMake 子工程。

### 内置插件

- 感知插件: `plugins/perception/`
- 规划器插件: `plugins/planning/`

### 编译选项

```bash
# 禁用所有插件
cmake -B build -DBUILD_PLUGINS=OFF

# 禁用特定插件
cmake -B build -DBUILD_ASTAR_PLANNER_PLUGIN=OFF
```

### 开发外部插件

参考 [外部插件开发指南](external_plugins/README.md)
```

#### 6.2 创建迁移说明文档

已创建：
- ✅ `PLUGIN_SUBPROJECT_DESIGN.md`
- ✅ `plugins/README.md`
- ✅ `external_plugins/README.md`

---

## ✅ 验收标准

- [ ] 所有插件成功编译
- [ ] 插件可以独立编译（修改单个插件只重新编译该插件）
- [ ] 测试程序正常运行
- [ ] 主程序正常运行
- [ ] 可以通过 CMake 选项禁用特定插件
- [ ] 文档完整且准确

---

## 🔄 回滚计划

如果迁移失败，可以回滚到备份分支：

```bash
git checkout backup/before-plugin-subproject-migration
```

---

## 📊 预期收益

| 指标 | 迁移前 | 迁移后 | 改进 |
|------|--------|--------|------|
| 编译时间（修改单个插件） | ~30s | ~5s | 6x 提升 |
| 插件独立性 | 低 | 高 | ✅ |
| 用户扩展难度 | 高 | 低 | ✅ |
| 插件可分发性 | 否 | 是 | ✅ |

---

## 📝 后续工作

1. **动态插件加载**: 实现运行时加载 .so/.dll 插件
2. **插件市场**: 建立插件分享平台
3. **插件模板**: 提供插件项目模板生成器
4. **CI/CD**: 为每个插件配置独立的 CI 流程

