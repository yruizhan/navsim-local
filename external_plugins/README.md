# 外部插件开发指南

本指南介绍如何开发独立的 NavSim 插件包，无需修改 NavSim 核心代码。

---

## 📋 前提条件

### 1. 安装 NavSim 核心库

```bash
cd navsim-local
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build
sudo cmake --install build
```

安装后，以下文件将被安装：

```
/usr/local/
├── include/
│   ├── core/
│   └── plugin/
│       ├── framework/
│       ├── data/
│       └── preprocessing/
├── lib/
│   ├── libnavsim_proto.a
│   ├── libnavsim_plugin_framework.a
│   └── libnavsim_core.a
└── lib/cmake/NavSim/
    ├── NavSimCoreTargets.cmake
    ├── NavSimPluginConfig.cmake
    └── NavSimPluginHelpers.cmake
```

### 2. 验证安装

```bash
# 检查 CMake 配置
cmake --find-package -DNAME=NavSim -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST

# 应该输出: NavSim found.
```

---

## 🚀 快速开始

### 示例 1: 创建简单的规划器插件

#### 1.1 创建项目结构

```bash
mkdir my_custom_planner
cd my_custom_planner
mkdir -p include src
```

#### 1.2 创建头文件 `include/my_custom_planner_plugin.hpp`

```cpp
#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include <nlohmann/json.hpp>

namespace my_plugins {

class MyCustomPlannerPlugin : public navsim::plugin::PlannerPluginInterface {
public:
    MyCustomPlannerPlugin() = default;
    ~MyCustomPlannerPlugin() override = default;

    // 必须实现的方法
    navsim::plugin::PlannerPluginMetadata getMetadata() const override;
    
    bool initialize(const nlohmann::json& config) override;
    
    bool plan(const navsim::planning::PlanningContext& context,
              std::chrono::milliseconds deadline,
              navsim::plugin::PlanningResult& result) override;
    
    std::pair<bool, std::string> isAvailable(
        const navsim::planning::PlanningContext& context) const override;

private:
    // 插件配置
    double max_velocity_ = 2.0;
    double time_step_ = 0.1;
};

} // namespace my_plugins
```

#### 1.3 创建源文件 `src/my_custom_planner_plugin.cpp`

```cpp
#include "my_custom_planner_plugin.hpp"
#include <iostream>

namespace my_plugins {

navsim::plugin::PlannerPluginMetadata 
MyCustomPlannerPlugin::getMetadata() const {
    navsim::plugin::PlannerPluginMetadata metadata;
    metadata.name = "MyCustomPlanner";
    metadata.version = "1.0.0";
    metadata.description = "My custom planning algorithm";
    metadata.author = "Your Name";
    metadata.type = "custom";
    metadata.can_be_fallback = false;
    return metadata;
}

bool MyCustomPlannerPlugin::initialize(const nlohmann::json& config) {
    if (config.contains("max_velocity")) {
        max_velocity_ = config["max_velocity"].get<double>();
    }
    if (config.contains("time_step")) {
        time_step_ = config["time_step"].get<double>();
    }
    
    std::cout << "[MyCustomPlanner] Initialized with max_velocity=" 
              << max_velocity_ << std::endl;
    return true;
}

bool MyCustomPlannerPlugin::plan(
    const navsim::planning::PlanningContext& context,
    std::chrono::milliseconds deadline,
    navsim::plugin::PlanningResult& result) {
    
    // 实现您的规划算法
    // ...
    
    result.success = true;
    result.planner_name = "MyCustomPlanner";
    return true;
}

std::pair<bool, std::string> MyCustomPlannerPlugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {
    return {true, "Available"};
}

} // namespace my_plugins
```

#### 1.4 创建 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_custom_planner VERSION 1.0.0 LANGUAGES CXX)

# 查找 NavSim 核心库
find_package(NavSim REQUIRED)

# 创建插件库
add_library(my_custom_planner_plugin SHARED
    src/my_custom_planner_plugin.cpp)

target_include_directories(my_custom_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# 链接到 NavSim 插件框架
target_link_libraries(my_custom_planner_plugin
    PUBLIC NavSim::navsim_plugin_framework)

target_compile_features(my_custom_planner_plugin PUBLIC cxx_std_17)

# 安装
install(TARGETS my_custom_planner_plugin
    LIBRARY DESTINATION lib/navsim_plugins
    ARCHIVE DESTINATION lib/navsim_plugins)

install(DIRECTORY include/
    DESTINATION include/my_custom_planner
    FILES_MATCHING PATTERN "*.hpp")
```

#### 1.5 编译和安装

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/usr/local
cmake --build build
sudo cmake --install build
```

---

## 📦 示例 2: 创建感知插件

### 2.1 创建项目

```bash
mkdir my_perception_plugin
cd my_perception_plugin
mkdir -p include src
```

### 2.2 创建头文件 `include/my_perception_plugin.hpp`

```cpp
#pragma once

#include "plugin/framework/perception_plugin_interface.hpp"
#include <nlohmann/json.hpp>

namespace my_plugins {

class MyPerceptionPlugin : public navsim::plugin::PerceptionPluginInterface {
public:
    MyPerceptionPlugin() = default;
    ~MyPerceptionPlugin() override = default;

    navsim::plugin::PerceptionPluginMetadata getMetadata() const override;
    
    bool initialize(const nlohmann::json& config) override;
    
    bool process(const navsim::plugin::PerceptionInput& input,
                 navsim::planning::PlanningContext& context) override;

private:
    double resolution_ = 0.1;
};

} // namespace my_plugins
```

### 2.3 实现和编译

参考规划器插件的步骤。

---

## 🔧 使用 CMake 辅助函数

NavSim 提供了便捷的 CMake 函数来简化插件开发。

### 使用 `navsim_add_planner_plugin`

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_custom_planner VERSION 1.0.0 LANGUAGES CXX)

find_package(NavSim REQUIRED)

# 使用辅助函数创建插件
navsim_add_planner_plugin(
    NAME my_custom_planner_plugin
    SOURCES src/my_custom_planner_plugin.cpp
    HEADERS include/my_custom_planner_plugin.hpp
    DEPENDENCIES Eigen3::Eigen  # 可选的额外依赖
)

# 安装插件
navsim_install_plugin(NAME my_custom_planner_plugin)
```

### 使用 `navsim_add_perception_plugin`

```cmake
navsim_add_perception_plugin(
    NAME my_perception_plugin
    SOURCES src/my_perception_plugin.cpp
    HEADERS include/my_perception_plugin.hpp
)

navsim_install_plugin(NAME my_perception_plugin)
```

---

## 📝 插件注册

### 方法 1: 在 NavSim 核心中注册（需要重新编译核心）

修改 `navsim-local/src/plugin/framework/plugin_init.cpp`:

```cpp
#include "my_custom_planner_plugin.hpp"

void initializeAllPlugins() {
    // ... 现有插件 ...
    
    // 注册外部插件
    PlannerPluginRegistry::getInstance().registerPlugin(
        "MyCustomPlanner",
        []() -> std::shared_ptr<PlannerPluginInterface> {
            return std::make_shared<my_plugins::MyCustomPlannerPlugin>();
        });
}
```

### 方法 2: 动态加载（推荐 - 需要实现动态加载功能）

未来版本将支持运行时动态加载插件。

---

## 🧪 测试插件

### 创建测试程序

```cpp
#include "my_custom_planner_plugin.hpp"
#include <iostream>

int main() {
    auto plugin = std::make_shared<my_plugins::MyCustomPlannerPlugin>();
    
    // 初始化
    nlohmann::json config = {
        {"max_velocity", 3.0},
        {"time_step", 0.1}
    };
    
    if (!plugin->initialize(config)) {
        std::cerr << "Failed to initialize plugin" << std::endl;
        return 1;
    }
    
    // 测试元数据
    auto metadata = plugin->getMetadata();
    std::cout << "Plugin: " << metadata.name << std::endl;
    std::cout << "Version: " << metadata.version << std::endl;
    
    return 0;
}
```

---

## 📚 完整示例项目

参考 `navsim-local/plugins/` 目录下的内置插件：

- **感知插件**: `plugins/perception/grid_map_builder/`
- **规划器插件**: `plugins/planning/straight_line/`

---

## ❓ 常见问题

### Q1: 找不到 NavSim 包

```bash
# 设置 CMAKE_PREFIX_PATH
cmake -B build -DCMAKE_PREFIX_PATH=/usr/local

# 或设置环境变量
export CMAKE_PREFIX_PATH=/usr/local
```

### Q2: 链接错误

确保链接到正确的库：

```cmake
target_link_libraries(my_plugin
    PUBLIC NavSim::navsim_plugin_framework  # 使用命名空间
)
```

### Q3: 如何调试插件

```bash
# 使用 Debug 模式编译
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# 使用 gdb 调试
gdb ./build/my_test_program
```

---

## 📞 获取帮助

- 查看内置插件示例
- 阅读 [插件架构设计文档](../docs/PLUGIN_ARCHITECTURE_DESIGN.md)
- 查看 [插件快速参考](../docs/PLUGIN_QUICK_REFERENCE.md)

