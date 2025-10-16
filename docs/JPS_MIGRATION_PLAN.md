# JPS 规划器移植计划

## 🎯 总体目标

将现有的 JPS 规划器代码移植到 NavSim 插件系统，实现**算法与插件完全解耦**的架构。

---

## 📐 架构设计

### 目录结构

```
navsim-local/plugins/planning/jps_planner_plugin/
├── include/
│   ├── jps_planner_plugin.hpp        ← 插件接口层（NEW）
│   ├── jps_planner.hpp                ← 核心算法层（MIGRATED）
│   ├── graph_search.hpp               ← JPS 搜索算法（MIGRATED）
│   └── jps_data_structures.hpp        ← 数据结构定义（MIGRATED）
├── src/
│   ├── jps_planner_plugin.cpp         ← 插件实现（NEW）
│   ├── jps_planner.cpp                ← 核心算法实现（MIGRATED）
│   └── graph_search.cpp               ← JPS 搜索实现（MIGRATED）
├── CMakeLists.txt                     ← 构建配置（NEW）
└── config/
    └── jps_planner_config.json        ← 默认配置（NEW）
```

---

## 🏗️ 分层架构

### 第 1 层：插件接口层（NEW）

**文件**：`jps_planner_plugin.hpp/cpp`

**职责**：
- 继承 `PlanningPluginInterface`
- 处理插件生命周期
- 从 JSON 读取配置
- 从 `PlanningContext` 获取数据
- 调用核心算法层
- 将结果转换为 `PlanningContext` 格式

**不包含**：
- ❌ 任何算法逻辑
- ❌ 任何路径规划代码
- ❌ 任何轨迹生成代码

### 第 2 层：核心算法层（MIGRATED）

**文件**：`jps_planner.hpp/cpp`, `graph_search.hpp/cpp`

**职责**：
- 实现 JPS 搜索算法
- 实现路径优化
- 实现轨迹生成
- 实现时间规划

**不包含**：
- ❌ 任何插件系统依赖
- ❌ 任何 `PlanningContext` 依赖
- ❌ 任何 ROS 依赖

**接口**：
```cpp
class JPSPlanner {
public:
  JPSPlanner(std::shared_ptr<ESDFMap> map);
  
  // 纯算法接口
  bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
  
  // 获取结果
  const std::vector<Eigen::Vector2d>& getRawPath() const;
  const std::vector<Eigen::Vector2d>& getOptimizedPath() const;
  const FlatTrajData& getFlatTraj() const;
  
  // 配置
  void setConfig(const JPSConfig& config);
  
private:
  std::shared_ptr<ESDFMap> map_util_;
  std::shared_ptr<GraphSearch> graph_search_;
  // ... 其他成员
};
```

### 第 3 层：数据结构层（MIGRATED）

**文件**：`jps_data_structures.hpp`

**职责**：
- 定义所有数据结构
- 不依赖任何外部系统

**内容**：
- `State` - 搜索节点
- `JPS2DNeib` - 2D JPS 邻居
- `JPS3DNeib` - 3D JPS 邻居
- `FlatTrajData` - 平坦轨迹数据
- `PathNode` - 路径节点
- `JPSConfig` - JPS 配置（NEW）

---

## 📋 详细移植计划

### 步骤 1：创建数据结构文件

**文件**：`jps_data_structures.hpp`

**内容**：
1. 从 `graph_search.h` 复制：
   - `compare_state<T>`
   - `State`
   - `JPS2DNeib`
   - `JPS3DNeib`

2. 从 `traj_representation.h` 复制：
   - `PathNode`
   - `FlatTrajData`

3. 新增：
   ```cpp
   struct JPSConfig {
     // 安全距离
     double safe_dis = 0.3;
     double max_jps_dis = 10.0;
     
     // 权重
     double distance_weight = 1.0;
     double yaw_weight = 1.0;
     
     // 轨迹参数
     double traj_cut_length = 5.0;
     
     // 运动学约束
     double max_vel = 1.0;
     double max_acc = 1.0;
     double max_omega = 1.0;
     double max_domega = 1.0;
     
     // 采样参数
     double sample_time = 0.1;
     int min_traj_num = 10;
     
     // JPS 参数
     double jps_truncation_time = 5.0;
   };
   ```

**修改**：
- ❌ 移除所有 ROS 头文件
- ✅ 只保留 Eigen 和 STL 头文件

---

### 步骤 2：移植 GraphSearch 类

**源文件**：`jps_planner/include/graph_search.h` → `jps_planner_plugin/include/graph_search.hpp`
**源文件**：`jps_planner/src/graph_search.cpp` → `jps_planner_plugin/src/graph_search.cpp`

**修改清单**：

#### graph_search.hpp

1. **头文件替换**：
   ```cpp
   // 移除
   #include <plan_env/sdf_map.h>
   
   // 添加
   #include "esdf_map.hpp"
   #include "jps_data_structures.hpp"
   ```

2. **命名空间**：
   ```cpp
   // 保持
   namespace JPS {
     // ...
   }
   ```

3. **类型替换**：
   ```cpp
   // 替换
   std::shared_ptr<SDFmap> map_;
   // 为
   std::shared_ptr<navsim::perception::ESDFMap> map_;
   ```

4. **构造函数**：
   ```cpp
   // 替换
   GraphSearch(std::shared_ptr<SDFmap> Map, const double &safe_dis);
   // 为
   GraphSearch(std::shared_ptr<navsim::perception::ESDFMap> Map, const double &safe_dis);
   ```

5. **保持不变**：
   - 所有公有函数签名
   - 所有私有函数签名
   - 所有成员变量（除了 `map_`）
   - 所有算法逻辑

#### graph_search.cpp

1. **头文件替换**：
   ```cpp
   // 替换
   #include <front_end/jps_planner/graph_search.h>
   // 为
   #include "graph_search.hpp"
   ```

2. **SDFmap 函数调用**（保持不变，因为 ESDFMap 兼容）：
   - `map_->GLX_SIZE_` ✅
   - `map_->GLY_SIZE_` ✅
   - `map_->Index2Vectornum(x,y)` ✅
   - `map_->isOccWithSafeDis(x,y,safe_dis_)` ✅
   - `map_->isUnOccupied(x,y)` ✅
   - `map_->isOccupied(x,y)` ✅

3. **保持不变**：
   - 所有算法实现
   - 所有函数逻辑
   - 所有数据结构操作

---

### 步骤 3：移植 JPSPlanner 类

**源文件**：`jps_planner/include/jps_planner.h` → `jps_planner_plugin/include/jps_planner.hpp`
**源文件**：`jps_planner/src/jps_planner.cpp` → `jps_planner_plugin/src/jps_planner.cpp`

**修改清单**：

#### jps_planner.hpp

1. **头文件替换**：
   ```cpp
   // 移除
   #include <plan_env/sdf_map.h>
   #include <front_end/jps_planner/graph_search.h>
   #include <front_end/traj_representation.h>
   #include <ros/ros.h>
   #include <ros/package.h>
   #include <nav_msgs/Path.h>
   #include <visualization_msgs/Marker.h>
   #include <tf/tf.h>
   
   // 添加
   #include "esdf_map.hpp"
   #include "graph_search.hpp"
   #include "jps_data_structures.hpp"
   #include <Eigen/Eigen>
   #include <memory>
   #include <vector>
   ```

2. **类型替换**：
   ```cpp
   // 替换
   std::shared_ptr<SDFmap> map_util_;
   // 为
   std::shared_ptr<navsim::perception::ESDFMap> map_util_;
   ```

3. **移除 ROS 成员**：
   ```cpp
   // 移除
   ros::NodeHandle nh_;
   ros::Publisher path_pub_;
   ros::Publisher init_path_pub_;
   ros::Publisher normal_vector_pub_;
   ```

4. **构造函数修改**：
   ```cpp
   // 替换
   JPSPlanner(std::shared_ptr<SDFmap> map, const ros::NodeHandle &nh);
   // 为
   JPSPlanner(std::shared_ptr<navsim::perception::ESDFMap> map);
   ```

5. **添加配置接口**：
   ```cpp
   // 添加
   void setConfig(const JPSConfig& config);
   const JPSConfig& getConfig() const;
   ```

6. **移除 ROS 函数**：
   ```cpp
   // 移除
   void pubPath(const std::vector<Eigen::Vector2d> &path, const ros::Publisher &pub);
   ```

7. **添加结果获取接口**：
   ```cpp
   // 添加
   const std::vector<Eigen::Vector2d>& getRawPath() const { return raw_path_; }
   const std::vector<Eigen::Vector2d>& getOptimizedPath() const { return path_; }
   const FlatTrajData& getFlatTraj() const { return flat_traj_; }
   ```

8. **添加配置成员**：
   ```cpp
   // 添加
   JPSConfig config_;
   ```

#### jps_planner.cpp

1. **头文件替换**：
   ```cpp
   // 替换
   #include "front_end/jps_planner/jps_planner.h"
   // 为
   #include "jps_planner.hpp"
   ```

2. **构造函数重写**：
   ```cpp
   // 原始（line 5-29）
   JPSPlanner::JPSPlanner(std::shared_ptr<SDFmap> map, const ros::NodeHandle &nh)
     : map_util_(map), nh_(nh) {
     path_pub_ = ros::Publisher(nh_.advertise<nav_msgs::Path>("jps_path", 1));
     // ... ROS 参数读取 ...
     graph_search_ = std::make_shared<GraphSearch>(map_util_, safe_dis_);
   }
   
   // 新版
   JPSPlanner::JPSPlanner(std::shared_ptr<navsim::perception::ESDFMap> map)
     : map_util_(map) {
     // 配置通过 setConfig() 设置
   }
   ```

3. **添加配置函数**：
   ```cpp
   void JPSPlanner::setConfig(const JPSConfig& config) {
     config_ = config;
     
     // 创建 GraphSearch（如果还没有）
     if (!graph_search_) {
       graph_search_ = std::make_shared<GraphSearch>(map_util_, config_.safe_dis);
     } else {
       graph_search_->SetSafeDis(config_.safe_dis);
     }
   }
   ```

4. **移除 pubPath() 调用**：
   ```cpp
   // 移除所有
   pubPath(ps, init_path_pub_);
   pubPath(path_, path_pub_);
   ```

5. **移除 ROS 日志**：
   ```cpp
   // 替换
   ROS_INFO_STREAM("start_path_3d: " << pt.transpose());
   // 为
   // 移除或使用 NavSim 日志系统
   ```

6. **保持不变**：
   - 所有算法逻辑
   - 所有工具函数
   - 所有数据处理

---

### 步骤 4：创建插件接口层

**文件**：`jps_planner_plugin.hpp/cpp`（NEW）

#### jps_planner_plugin.hpp

```cpp
#ifndef JPS_PLANNER_PLUGIN_HPP
#define JPS_PLANNER_PLUGIN_HPP

#include "core/plugin_interface.hpp"
#include "jps_planner.hpp"
#include <memory>

namespace navsim {
namespace planning {

class JPSPlannerPlugin : public PlanningPluginInterface {
public:
  JPSPlannerPlugin() = default;
  ~JPSPlannerPlugin() override = default;

  // 插件生命周期
  bool initialize(const nlohmann::json& config) override;
  bool plan(const PlanningContext& context, PlanningResult& result) override;
  void reset() override;
  void shutdown() override;

  // 插件信息
  std::string name() const override { return "JPSPlanner"; }
  std::string version() const override { return "1.0.0"; }

private:
  // 核心算法对象
  std::unique_ptr<JPS::JPSPlanner> jps_planner_;
  
  // ESDFMap 指针
  std::shared_ptr<perception::ESDFMap> esdf_map_;
  
  // 配置
  JPS::JPSConfig config_;
  
  // 辅助函数
  bool loadConfig(const nlohmann::json& json_config);
  bool convertResultToPlanningContext(const JPS::FlatTrajData& flat_traj,
                                      PlanningResult& result);
};

} // namespace planning
} // namespace navsim

#endif // JPS_PLANNER_PLUGIN_HPP
```

#### jps_planner_plugin.cpp

```cpp
#include "jps_planner_plugin.hpp"
#include "core/logging.hpp"
#include <iostream>

namespace navsim {
namespace planning {

bool JPSPlannerPlugin::initialize(const nlohmann::json& config) {
  LOG_INFO("Initializing JPS Planner Plugin...");
  
  // 1. 加载配置
  if (!loadConfig(config)) {
    LOG_ERROR("Failed to load JPS planner configuration");
    return false;
  }
  
  // 2. 获取 ESDFBuilderPlugin
  auto esdf_builder = getPlugin<perception::ESDFBuilderPlugin>("ESDFBuilder");
  if (!esdf_builder) {
    LOG_ERROR("Failed to get ESDFBuilder plugin");
    return false;
  }
  
  // 3. 获取 ESDFMap
  esdf_map_ = esdf_builder->getESDFMap();
  if (!esdf_map_) {
    LOG_ERROR("Failed to get ESDF map from ESDFBuilder");
    return false;
  }
  
  // 4. 创建 JPSPlanner
  jps_planner_ = std::make_unique<JPS::JPSPlanner>(esdf_map_);
  jps_planner_->setConfig(config_);
  
  LOG_INFO("JPS Planner Plugin initialized successfully");
  return true;
}

bool JPSPlannerPlugin::plan(const PlanningContext& context, PlanningResult& result) {
  // 1. 检查输入
  if (!context.start_state || !context.goal_state) {
    LOG_ERROR("Start or goal state is null");
    return false;
  }
  
  // 2. 提取起点和终点
  Eigen::Vector3d start(context.start_state->x, context.start_state->y, context.start_state->yaw);
  Eigen::Vector3d goal(context.goal_state->x, context.goal_state->y, context.goal_state->yaw);
  
  // 3. 调用 JPS 规划
  if (!jps_planner_->plan(start, goal)) {
    LOG_WARN("JPS planning failed");
    return false;
  }
  
  // 4. 转换结果
  const auto& flat_traj = jps_planner_->getFlatTraj();
  if (!convertResultToPlanningContext(flat_traj, result)) {
    LOG_ERROR("Failed to convert JPS result to PlanningContext");
    return false;
  }
  
  LOG_INFO("JPS planning succeeded");
  return true;
}

void JPSPlannerPlugin::reset() {
  // 重置状态（如果需要）
}

void JPSPlannerPlugin::shutdown() {
  jps_planner_.reset();
  esdf_map_.reset();
  LOG_INFO("JPS Planner Plugin shutdown");
}

bool JPSPlannerPlugin::loadConfig(const nlohmann::json& json_config) {
  try {
    config_.safe_dis = json_config.value("safe_dis", 0.3);
    config_.max_jps_dis = json_config.value("max_jps_dis", 10.0);
    config_.distance_weight = json_config.value("distance_weight", 1.0);
    config_.yaw_weight = json_config.value("yaw_weight", 1.0);
    config_.traj_cut_length = json_config.value("traj_cut_length", 5.0);
    config_.max_vel = json_config.value("max_vel", 1.0);
    config_.max_acc = json_config.value("max_acc", 1.0);
    config_.max_omega = json_config.value("max_omega", 1.0);
    config_.max_domega = json_config.value("max_domega", 1.0);
    config_.sample_time = json_config.value("sample_time", 0.1);
    config_.min_traj_num = json_config.value("min_traj_num", 10);
    config_.jps_truncation_time = json_config.value("jps_truncation_time", 5.0);
    
    return true;
  } catch (const std::exception& e) {
    LOG_ERROR("Exception while loading config: {}", e.what());
    return false;
  }
}

bool JPSPlannerPlugin::convertResultToPlanningContext(
    const JPS::FlatTrajData& flat_traj, PlanningResult& result) {
  // TODO: 实现结果转换
  // 将 FlatTrajData 转换为 PlanningResult
  return true;
}

} // namespace planning
} // namespace navsim

// 注册插件
REGISTER_PLANNING_PLUGIN(navsim::planning::JPSPlannerPlugin)
```

---

## 📝 总结

### 移植工作量

| 任务 | 文件数 | 预计工作量 |
|------|--------|-----------|
| 创建数据结构文件 | 1 | 1 小时 |
| 移植 GraphSearch | 2 | 2 小时 |
| 移植 JPSPlanner | 2 | 3 小时 |
| 创建插件接口 | 2 | 2 小时 |
| 创建 CMakeLists.txt | 1 | 1 小时 |
| 测试和调试 | - | 3 小时 |
| **总计** | **8** | **12 小时** |

### 关键原则

1. ✅ **算法与插件完全解耦**
2. ✅ **保持算法逻辑不变**
3. ✅ **移除所有 ROS 依赖**
4. ✅ **使用 ESDFMap 替换 SDFmap**
5. ✅ **配置驱动**

### 下一步

1. 创建 `jps_data_structures.hpp`
2. 移植 `graph_search.hpp/cpp`
3. 移植 `jps_planner.hpp/cpp`
4. 创建 `jps_planner_plugin.hpp/cpp`
5. 创建 `CMakeLists.txt`
6. 测试和验证

