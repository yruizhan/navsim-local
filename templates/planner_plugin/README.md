# {{PLUGIN_NAME}} Planner Plugin

{{DESCRIPTION}}

## 📋 概述

- **插件名称**: {{PLUGIN_NAME}}
- **插件类型**: 规划器 (Planner)
- **版本**: 1.0.0
- **作者**: {{AUTHOR}}

## 🏗️ 架构

本插件采用三层解耦架构：

```
{{PLUGIN_NAME_SNAKE}}_plugin/
├── algorithm/              # 算法层（纯算法，无平台依赖）
│   ├── {{PLUGIN_NAME_SNAKE}}.hpp
│   └── {{PLUGIN_NAME_SNAKE}}.cpp
├── adapter/                # 适配器层（平台接口适配）
│   ├── {{PLUGIN_NAME_SNAKE}}_plugin.hpp
│   ├── {{PLUGIN_NAME_SNAKE}}_plugin.cpp
│   └── register.cpp
├── tests/                  # 测试（可选）
│   └── test_{{PLUGIN_NAME_SNAKE}}.cpp
├── CMakeLists.txt
└── README.md
```

### 算法层 (algorithm/)

- **职责**: 实现核心规划算法
- **依赖**: 仅依赖 Eigen 和 STL
- **特点**: 可复用到其他项目

### 适配器层 (adapter/)

- **职责**: 实现平台插件接口，转换数据格式
- **依赖**: 依赖平台 API 和算法层
- **特点**: 薄适配层，逻辑在算法层

## 🚀 快速开始

### 编译

```bash
cd navsim-local
mkdir -p build && cd build
cmake ..
make {{PLUGIN_NAME_SNAKE}}_plugin
```

### 使用

```bash
# 使用本地调试工具测试
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner {{PLUGIN_NAME}}
```

## ⚙️ 配置参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_velocity` | double | 2.0 | 最大速度 (m/s) |
| `max_acceleration` | double | 2.0 | 最大加速度 (m/s²) |
| `step_size` | double | 0.1 | 步长 (m) |
| `max_iterations` | int | 1000 | 最大迭代次数 |

### 配置示例

```json
{
  "{{PLUGIN_NAME}}": {
    "max_velocity": 3.0,
    "max_acceleration": 2.5,
    "step_size": 0.05,
    "max_iterations": 5000
  }
}
```

## 📊 性能

TODO: 添加性能测试结果

## 🧪 测试

```bash
# 运行单元测试
cd build
ctest -R {{PLUGIN_NAME_SNAKE}}
```

## 📝 开发指南

### 如何集成已有算法

如果您已经有算法代码（如从其他项目复制），按以下步骤集成：

#### 1. 复制算法文件到 `algorithm/` 目录

```bash
# 示例：复制 JPS 算法文件
cp /path/to/old_plugin/jps_planner.{hpp,cpp} algorithm/
cp /path/to/old_plugin/graph_search.{hpp,cpp} algorithm/
cp /path/to/old_plugin/jps_data_structures.hpp algorithm/
```

#### 2. 更新适配层头文件 (`adapter/{{PLUGIN_NAME_SNAKE}}_plugin.hpp`)

```cpp
// 2.1 包含您的算法头文件
#include "../algorithm/your_algorithm.hpp"

// 2.2 添加算法实例作为成员变量
private:
  std::unique_ptr<YourAlgorithm> algorithm_;
  YourAlgorithmConfig config_;

  // 如果需要感知数据
  std::shared_ptr<navsim::perception::ESDFMap> esdf_map_;
```

#### 3. 更新适配层实现 (`adapter/{{PLUGIN_NAME_SNAKE}}_plugin.cpp`)

```cpp
// 3.1 在 loadConfig() 中解析配置
bool {{PLUGIN_NAME}}Plugin::loadConfig(const nlohmann::json& config) {
  config_.your_param = config.value("your_param", default_value);
  // ...
}

// 3.2 在 plan() 中调用算法
bool {{PLUGIN_NAME}}Plugin::plan(...) {
  // 获取感知数据（如果需要）
  esdf_map_ = context.getCustomData<navsim::perception::ESDFMap>("perception_esdf_map");

  // 调用算法
  bool success = algorithm_->plan(start, goal);

  // 转换输出
  convertAlgorithmOutputToResult(...);
}
```

#### 4. 更新 CMakeLists.txt

```cmake
# 4.1 添加算法源文件
add_library({{PLUGIN_NAME_SNAKE}}_plugin SHARED
    algorithm/your_algorithm.cpp
    algorithm/helper_module.cpp  # 如果有多个文件
    adapter/{{PLUGIN_NAME_SNAKE}}_plugin.cpp
    adapter/register.cpp)

# 4.2 添加依赖（如果需要）
find_package(Boost REQUIRED)  # 如果算法使用 Boost

target_include_directories({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE
    ${CMAKE_SOURCE_DIR}/plugins/perception/esdf_builder/include)  # 如果需要 ESDF

target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE
    Boost::boost
    esdf_builder_plugin)
```

#### 5. 编译和测试

```bash
cd build
cmake ..
make {{PLUGIN_NAME_SNAKE}}_plugin -j4
./navsim_local_debug --planner {{PLUGIN_NAME}} --scenario scenarios/simple_corridor.json
```

### 从零开发新算法

如果您要从零开发新算法：

#### 1. 编辑算法层

编辑 `algorithm/{{PLUGIN_NAME_SNAKE}}.cpp` 中的 `plan()` 方法：

```cpp
bool {{PLUGIN_NAME}}::plan(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& goal) {
  // TODO: 实现您的算法
  // 1. 初始化数据结构
  // 2. 执行搜索/优化
  // 3. 生成路径
  // 4. 返回结果
}
```

#### 2. 添加配置参数

在 `algorithm/{{PLUGIN_NAME_SNAKE}}.hpp` 的 `Config` 结构体中添加参数：

```cpp
struct Config {
  double max_velocity = 2.0;
  double your_new_param = 1.0;  // 新参数
};
```

在 `adapter/{{PLUGIN_NAME_SNAKE}}_plugin.cpp` 的 `loadConfig()` 中解析：

```cpp
if (config.contains("your_new_param")) {
  config_.your_new_param = config["your_new_param"].get<double>();
}
```

### 添加感知数据依赖

如果您的算法需要感知数据（如栅格地图、ESDF 等）：

#### 1. 在 `getMetadata()` 中声明依赖

```cpp
navsim::plugin::PlannerPluginMetadata {{PLUGIN_NAME}}Plugin::getMetadata() const {
  metadata.required_perception_data = {"esdf_map"};  // 或 {"occupancy_grid"}
  return metadata;
}
```

#### 2. 在 `plan()` 中获取感知数据

```cpp
// 获取 ESDF 地图
esdf_map_ = context.getCustomData<navsim::perception::ESDFMap>("perception_esdf_map");
if (!esdf_map_) {
  result.failure_reason = "ESDF map not available";
  return false;
}

// 使用地图
algorithm_->setMap(esdf_map_);
```

#### 3. 在 CMakeLists.txt 中添加依赖

```cmake
target_include_directories({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE
    ${CMAKE_SOURCE_DIR}/plugins/perception/esdf_builder/include)

target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin PRIVATE
    esdf_builder_plugin)
```

## 📚 相关文档

- [插件开发指南](../../docs/PLUGIN_DEVELOPMENT.md)
- [三层架构说明](../../REFACTORING_PROPOSAL.md)
- [本地调试模式](../../docs/LOCAL_DEBUG_MODE.md)

## 📄 许可证

TODO: 添加许可证信息

