# ESDF 地图构建插件适配方案

## 📋 概述

本文档描述如何将基于 ROS 的 ESDF (Euclidean Signed Distance Field) 地图构建模块适配到 navsim-local 项目中。

---

## 🔍 原始代码分析

### 核心功能

原始 ESDF 代码（位于 `plugins/perception/esdf_map/`）实现了以下功能：

1. **占据栅格地图构建**：
   - 从点云数据构建占据栅格地图
   - 使用 Raycast 算法进行概率更新
   - 支持动态更新和离群点移除

2. **ESDF 计算**：
   - 使用 2D 距离变换算法（Felzenszwalb 算法）
   - 计算正距离场（自由空间到障碍物的距离）
   - 计算负距离场（障碍物内部到自由空间的距离）
   - 支持双线性插值和梯度计算

3. **可视化**：
   - 发布占据栅格地图（PointCloud2）
   - 发布 ESDF 地图（PointCloud2）
   - 发布 ESDF 梯度（MarkerArray）

### ROS 依赖项

**头文件依赖**：
- `ros/ros.h` - ROS 核心
- `sensor_msgs/PointCloud2.h` - 点云消息
- `visualization_msgs/MarkerArray.h` - 可视化消息
- `tf2_ros/transform_listener.h` - TF 变换
- `pcl/point_cloud.h` - PCL 点云库
- `pcl_conversions/pcl_conversions.h` - PCL-ROS 转换

**运行时依赖**：
- `ros::NodeHandle` - 参数服务器、发布订阅
- `ros::Timer` - 定时器回调
- `ros::Subscriber` - 点云订阅
- `ros::Publisher` - 地图发布
- `tf2_ros::Buffer` - TF 变换查询

---

## 🎯 适配策略

### 策略 1：简化版 ESDF（推荐）

**核心思路**：
- **不使用点云和 Raycast**：直接从 `bev_obstacles` 和 `dynamic_obstacles` 构建占据栅格
- **保留 ESDF 计算核心**：使用原始的距离变换算法
- **移除 ROS 依赖**：使用项目现有的数据结构和接口

**优点**：
- ✅ 实现简单，代码量少
- ✅ 无需移植复杂的 Raycast 逻辑
- ✅ 与现有 GridMapBuilder 插件一致
- ✅ 性能更好（直接从几何障碍物生成）

**缺点**：
- ❌ 失去了基于传感器的概率更新能力
- ❌ 无法处理未知区域

### 策略 2：完整移植（复杂）

**核心思路**：
- 完整移植 Raycast 和概率更新逻辑
- 需要模拟点云数据（从 BEV 障碍物生成）
- 保留所有原始功能

**优点**：
- ✅ 保留完整功能
- ✅ 支持概率更新和未知区域

**缺点**：
- ❌ 实现复杂，代码量大
- ❌ 需要模拟点云数据（不自然）
- ❌ 性能开销大

**推荐**：使用**策略 1（简化版）**

---

## 🔧 简化版 ESDF 实现方案

### 1. 数据结构定义

已在 `include/core/planning_context.hpp` 中添加：

```cpp
struct ESDFMap {
  struct Config {
    Point2d origin;      // 地图原点 (m)
    double resolution;   // 栅格分辨率 (m/cell)
    int width;           // 宽度 (cells)
    int height;          // 高度 (cells)
    double max_distance; // 最大距离 (m)
  } config;

  std::vector<double> data;  // 距离场数据 (m)

  // 工具函数
  double getDistance(int x, int y) const;
  double getDistanceInterpolated(const Point2d& point) const;
  double getDistanceWithGradient(const Point2d& point, Point2d& gradient) const;
  bool isWithinSafeDistance(int x, int y, double safe_distance) const;
  Point2d cellToWorld(int x, int y) const;
  std::pair<int, int> worldToCell(const Point2d& point) const;
};
```

### 2. 插件接口

创建 `plugins/perception/esdf_builder/include/esdf_builder_plugin.hpp`：

```cpp
#pragma once

#include "plugin/perception_plugin_interface.hpp"
#include "core/planning_context.hpp"
#include <vector>
#include <memory>

namespace navsim {
namespace plugin {

class ESDFBuilderPlugin : public PerceptionPluginInterface {
public:
  ESDFBuilderPlugin() = default;
  ~ESDFBuilderPlugin() override = default;

  std::string getName() const override { return "ESDFBuilder"; }

  bool initialize(const std::map<std::string, std::any>& params) override;

  bool process(const PerceptionInput& input, PerceptionOutput& output) override;

private:
  // 配置参数
  double resolution_ = 0.1;        // 栅格分辨率 (m/cell)
  double map_width_ = 30.0;        // 地图宽度 (m)
  double map_height_ = 30.0;       // 地图高度 (m)
  double max_distance_ = 5.0;      // 最大距离 (m)
  bool include_dynamic_ = true;    // 是否包含动态障碍物

  // 内部数据
  std::vector<uint8_t> occupancy_grid_;  // 临时占据栅格
  int grid_width_ = 0;
  int grid_height_ = 0;

  // 核心算法
  void buildOccupancyGrid(const PerceptionInput& input, const planning::Point2d& origin);
  void computeESDF(planning::ESDFMap& esdf_map);
  
  // 距离变换算法（Felzenszwalb 算法）
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size);
};

} // namespace plugin
} // namespace navsim
```

### 3. 核心算法实现

#### 3.1 从 BEV 障碍物构建占据栅格

```cpp
void ESDFBuilderPlugin::buildOccupancyGrid(
    const PerceptionInput& input, 
    const planning::Point2d& origin) {
  
  // 清空栅格
  std::fill(occupancy_grid_.begin(), occupancy_grid_.end(), 0);
  
  // 处理静态障碍物
  for (const auto& obs : input.bev_obstacles.circles) {
    // 遍历圆形障碍物覆盖的栅格
    int min_x = std::max(0, static_cast<int>((obs.center.x - obs.radius - origin.x) / resolution_));
    int max_x = std::min(grid_width_ - 1, static_cast<int>((obs.center.x + obs.radius - origin.x) / resolution_));
    int min_y = std::max(0, static_cast<int>((obs.center.y - obs.radius - origin.y) / resolution_));
    int max_y = std::min(grid_height_ - 1, static_cast<int>((obs.center.y + obs.radius - origin.y) / resolution_));
    
    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        // 计算栅格中心到圆心的距离
        double cell_x = origin.x + (x + 0.5) * resolution_;
        double cell_y = origin.y + (y + 0.5) * resolution_;
        double dx = cell_x - obs.center.x;
        double dy = cell_y - obs.center.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist <= obs.radius) {
          occupancy_grid_[y * grid_width_ + x] = 100;  // 占据
        }
      }
    }
  }
  
  // 类似处理矩形和多边形障碍物...
  // 参考 GridMapBuilder 的实现
}
```

#### 3.2 ESDF 距离变换算法

```cpp
void ESDFBuilderPlugin::computeESDF(planning::ESDFMap& esdf_map) {
  int size = grid_width_ * grid_height_;
  std::vector<double> tmp_buffer(size, 0.0);
  std::vector<double> distance_buffer_pos(size, 0.0);
  std::vector<double> distance_buffer_neg(size, 0.0);
  
  // ========== 计算正距离场（自由空间到障碍物的距离） ==========
  
  // X 方向扫描
  for (int x = 0; x < grid_width_; ++x) {
    fillESDF(
      [&](int y) {
        return occupancy_grid_[y * grid_width_ + x] >= 50 ?
          0.0 : std::numeric_limits<double>::max();
      },
      [&](int y, double val) { tmp_buffer[y * grid_width_ + x] = val; },
      0, grid_height_ - 1, grid_height_
    );
  }
  
  // Y 方向扫描
  for (int y = 0; y < grid_height_; ++y) {
    fillESDF(
      [&](int x) { return tmp_buffer[y * grid_width_ + x]; },
      [&](int x, double val) {
        distance_buffer_pos[y * grid_width_ + x] = resolution_ * std::sqrt(val);
      },
      0, grid_width_ - 1, grid_width_
    );
  }
  
  // ========== 计算负距离场（障碍物内部到自由空间的距离） ==========
  
  // X 方向扫描
  for (int x = 0; x < grid_width_; ++x) {
    fillESDF(
      [&](int y) {
        return occupancy_grid_[y * grid_width_ + x] < 50 ?
          0.0 : std::numeric_limits<double>::max();
      },
      [&](int y, double val) { tmp_buffer[y * grid_width_ + x] = val; },
      0, grid_height_ - 1, grid_height_
    );
  }
  
  // Y 方向扫描
  for (int y = 0; y < grid_height_; ++y) {
    fillESDF(
      [&](int x) { return tmp_buffer[y * grid_width_ + x]; },
      [&](int x, double val) {
        distance_buffer_neg[y * grid_width_ + x] = resolution_ * std::sqrt(val);
      },
      0, grid_width_ - 1, grid_width_
    );
  }
  
  // ========== 合并正负距离场 ==========
  for (int i = 0; i < size; ++i) {
    esdf_map.data[i] = distance_buffer_pos[i];
    if (distance_buffer_neg[i] > 0.0) {
      esdf_map.data[i] += (-distance_buffer_neg[i] + resolution_);
    }
    // 截断到最大距离
    esdf_map.data[i] = std::min(esdf_map.data[i], max_distance_);
  }
}
```

#### 3.3 Felzenszwalb 距离变换算法

```cpp
template <typename F_get_val, typename F_set_val>
void ESDFBuilderPlugin::fillESDF(
    F_get_val f_get_val, 
    F_set_val f_set_val, 
    int start, 
    int end, 
    int dim_size) {
  
  std::vector<int> v(dim_size);
  std::vector<double> z(dim_size + 1);

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; ++q) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;
  for (int q = start; q <= end; ++q) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}
```

### 4. 插件注册

在 `plugins/perception/esdf_builder/src/esdf_builder_plugin.cpp` 中：

```cpp
#include "esdf_builder_plugin.hpp"
#include "plugin/plugin_factory.hpp"

// 注册插件
REGISTER_PERCEPTION_PLUGIN(ESDFBuilder, navsim::plugin::ESDFBuilderPlugin)
```

### 5. CMakeLists.txt

创建 `plugins/perception/esdf_builder/CMakeLists.txt`：

```cmake
# ESDF Builder Plugin
add_library(esdf_builder_plugin SHARED
  src/esdf_builder_plugin.cpp
)

target_include_directories(esdf_builder_plugin
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(esdf_builder_plugin
  PRIVATE
    navsim_core
)

# 安装到插件目录
install(TARGETS esdf_builder_plugin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/plugins/perception
)
```

### 6. 配置文件

在 `config/default.json` 中添加：

```json
{
  "perception": {
    "plugins": [
      {
        "name": "GridMapBuilder",
        "enabled": true,
        "priority": 100,
        "params": {
          "resolution": 0.1,
          "map_width": 30.0,
          "map_height": 30.0,
          "obstacle_cost": 100,
          "inflation_radius": 0.0
        }
      },
      {
        "name": "ESDFBuilder",
        "enabled": true,
        "priority": 90,
        "params": {
          "resolution": 0.1,
          "map_width": 30.0,
          "map_height": 30.0,
          "max_distance": 5.0,
          "include_dynamic": true
        }
      }
    ]
  }
}
```

---

## 📊 数据流

```
WorldTick
    ↓
BEV Obstacles + Dynamic Obstacles
    ↓
ESDFBuilderPlugin::process()
    ↓
buildOccupancyGrid()  ← 从几何障碍物构建占据栅格
    ↓
computeESDF()         ← 计算距离场
    ↓
ESDFMap (存储到 PerceptionOutput)
    ↓
规划器使用 (梯度优化、碰撞检测等)
```

---

## 🎯 使用场景

### 1. 基于梯度的轨迹优化

```cpp
// 在优化规划器中使用 ESDF
if (context.esdf_map) {
  for (const auto& point : trajectory) {
    planning::Point2d grad;
    double dist = context.esdf_map->getDistanceWithGradient(point, grad);
    
    if (dist < safe_distance) {
      // 添加排斥力，沿梯度方向远离障碍物
      repulsive_force = k_rep * (1.0 / dist - 1.0 / safe_distance) * grad;
    }
  }
}
```

### 2. 碰撞检测

```cpp
// 快速碰撞检测
double dist = context.esdf_map->getDistanceInterpolated(point);
if (dist < vehicle_radius) {
  // 碰撞！
}
```

### 3. 安全距离查询

```cpp
// 检查路径上的最小安全距离
double min_clearance = std::numeric_limits<double>::max();
for (const auto& point : path) {
  double dist = context.esdf_map->getDistanceInterpolated(point);
  min_clearance = std::min(min_clearance, dist);
}
```

---

## ✅ 总结

### 已完成

1. ✅ 定义 `ESDFMap` 数据结构
2. ✅ 实现 ESDF 工具函数（插值、梯度计算等）
3. ✅ 设计插件接口和核心算法

### 待实现

1. ⏳ 创建插件源文件
2. ⏳ 实现完整的 `buildOccupancyGrid()` 函数
3. ⏳ 添加 CMakeLists.txt
4. ⏳ 测试和调试

### 关键优势

- ✅ **简单高效**：直接从几何障碍物生成，无需点云
- ✅ **与现有系统一致**：遵循插件接口，易于集成
- ✅ **功能完整**：支持梯度计算、双线性插值
- ✅ **性能优秀**：使用高效的 Felzenszwalb 算法

---

**下一步**：是否需要我生成完整的插件源代码？

