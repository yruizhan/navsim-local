# 栅格地图精度问题修复说明

## 🐛 问题描述

**症状**：
1. 静态障碍物在栅格地图中被表示为圆形
2. 栅格圆形的半径比障碍物本体大很多（不够精确）
3. 矩形障碍物显示为圆形（丢失了形状信息）
4. 多边形障碍物显示为包围圆（丢失了精确轮廓）

**影响**：
- 栅格地图中的占据区域远大于实际障碍物的真实轮廓
- 路径规划算法会认为障碍物比实际更大，导致过于保守的规划
- 可能导致无法通过实际可以通过的狭窄通道

---

## 🔍 根本原因分析

### 问题 1：圆形障碍物精度不足

**原始代码**（`grid_map_builder_plugin.cpp` 第 153-173 行）：
```cpp
void GridMapBuilderPlugin::addCircleObstacle(...) {
  int radius_cells = static_cast<int>(std::ceil(circle.radius / grid.config.resolution));
  
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      // ❌ 问题：在栅格空间判断，而不是在世界坐标判断
      if (dx * dx + dy * dy <= radius_cells * radius_cells) {
        setGridCell(center_x + dx, center_y + dy, config_.obstacle_cost, grid);
      }
    }
  }
}
```

**问题分析**：
- 使用 `dx * dx + dy * dy <= radius_cells * radius_cells` 判断
- 这是在**栅格空间**判断（栅格索引的距离），而不是在**世界坐标**判断
- 没有考虑栅格格子的中心点位置
- 导致圆形边界不精确，可能多标记或少标记格子

**示例**：
```
假设：circle.radius = 1.0m, resolution = 0.2m
radius_cells = ceil(1.0 / 0.2) = 5

栅格空间判断：dx^2 + dy^2 <= 25
实际半径：5 * 0.2 = 1.0m ✅

但是！栅格格子的中心点可能在 (center_x + 5 + 0.5) * 0.2 = 1.1m 处
这个格子的中心距离圆心 1.1m > 1.0m，不应该被标记为占据
```

---

### 问题 2：矩形障碍物简化为圆形

**原始代码**（`grid_map_builder_plugin.cpp` 第 175-188 行）：
```cpp
void GridMapBuilderPlugin::addRectangleObstacle(...) {
  // ❌ 简化实现：将矩形转换为圆形（使用对角线的一半作为半径）
  double radius = std::sqrt(rect.width * rect.width + rect.height * rect.height) / 2.0;
  
  planning::BEVObstacles::Circle circle;
  circle.center.x = rect.pose.x;
  circle.center.y = rect.pose.y;
  circle.radius = radius;
  
  addCircleObstacle(circle, grid);
}
```

**问题分析**：
- 完全忽略了矩形的形状和旋转角度
- 使用对角线的一半作为半径，导致圆形远大于矩形
- 例如：2m x 4m 的矩形，对角线 = 4.47m，半径 = 2.24m
- 圆形面积 = π × 2.24² ≈ 15.8 m²
- 矩形面积 = 2 × 4 = 8 m²
- **圆形面积是矩形的 2 倍！**

---

### 问题 3：多边形障碍物简化为包围圆

**原始代码**（`grid_map_builder_plugin.cpp` 第 190-220 行）：
```cpp
void GridMapBuilderPlugin::addPolygonObstacle(...) {
  // ❌ 简化实现：计算多边形的包围圆
  double center_x = 0.0, center_y = 0.0;
  for (const auto& vertex : polygon.vertices) {
    center_x += vertex.x;
    center_y += vertex.y;
  }
  center_x /= polygon.vertices.size();
  center_y /= polygon.vertices.size();
  
  // 计算最大半径
  double max_radius = 0.0;
  for (const auto& vertex : polygon.vertices) {
    double dist = std::sqrt(dx * dx + dy * dy);
    max_radius = std::max(max_radius, dist);
  }
  
  addCircleObstacle(circle, grid);
}
```

**问题分析**：
- 使用多边形顶点的平均值作为圆心（不是最优的包围圆）
- 使用最远顶点的距离作为半径
- 完全丢失了多边形的精确形状
- 对于狭长的多边形，包围圆会非常大

---

## ✅ 修复方案

### 修复 1：圆形障碍物 - 精确几何判断

**修复后的代码**：
```cpp
void GridMapBuilderPlugin::addCircleObstacle(...) {
  // 🔧 修复：精确计算哪些栅格格子的中心点在圆内
  
  // 计算圆形在栅格中的范围（边界框）
  int min_x, min_y, max_x, max_y;
  worldToGrid(circle.center.x - circle.radius, circle.center.y - circle.radius, grid, min_x, min_y);
  worldToGrid(circle.center.x + circle.radius, circle.center.y + circle.radius, grid, max_x, max_y);
  
  // 遍历边界框内的所有栅格
  for (int gy = min_y; gy <= max_y; ++gy) {
    for (int gx = min_x; gx <= max_x; ++gx) {
      // ✅ 计算栅格格子中心点的世界坐标
      double cell_center_x = grid.config.origin.x + (gx + 0.5) * grid.config.resolution;
      double cell_center_y = grid.config.origin.y + (gy + 0.5) * grid.config.resolution;
      
      // ✅ 计算格子中心到圆心的距离（在世界坐标系中）
      double dx = cell_center_x - circle.center.x;
      double dy = cell_center_y - circle.center.y;
      double dist_sq = dx * dx + dy * dy;
      
      // ✅ 如果格子中心在圆内，标记为占据
      if (dist_sq <= circle.radius * circle.radius) {
        setGridCell(gx, gy, config_.obstacle_cost, grid);
      }
    }
  }
}
```

**改进点**：
- ✅ 在**世界坐标系**中判断，而不是栅格空间
- ✅ 判断栅格格子的**中心点**是否在圆内
- ✅ 精确的几何计算，不会多标记或少标记格子

---

### 修复 2：矩形障碍物 - 考虑旋转的精确填充

**修复后的代码**：
```cpp
void GridMapBuilderPlugin::addRectangleObstacle(...) {
  // 🔧 修复：精确计算旋转矩形内部的栅格格子
  
  double cos_yaw = std::cos(rect.pose.yaw);
  double sin_yaw = std::sin(rect.pose.yaw);
  double half_width = rect.width / 2.0;
  double half_height = rect.height / 2.0;
  
  // 计算包围盒
  double max_extent = std::sqrt(half_width * half_width + half_height * half_height);
  worldToGrid(rect.pose.x - max_extent, rect.pose.y - max_extent, grid, min_x, min_y);
  worldToGrid(rect.pose.x + max_extent, rect.pose.y + max_extent, grid, max_x, max_y);
  
  // 遍历包围盒内的所有栅格
  for (int gy = min_y; gy <= max_y; ++gy) {
    for (int gx = min_x; gx <= max_x; ++gx) {
      // ✅ 计算栅格格子中心点的世界坐标
      double cell_center_x = grid.config.origin.x + (gx + 0.5) * grid.config.resolution;
      double cell_center_y = grid.config.origin.y + (gy + 0.5) * grid.config.resolution;
      
      // ✅ 将格子中心点转换到矩形的局部坐标系
      double dx = cell_center_x - rect.pose.x;
      double dy = cell_center_y - rect.pose.y;
      
      // ✅ 旋转到矩形的局部坐标系（逆旋转）
      double local_x = dx * cos_yaw + dy * sin_yaw;
      double local_y = -dx * sin_yaw + dy * cos_yaw;
      
      // ✅ 检查是否在矩形内部
      if (std::abs(local_x) <= half_width && std::abs(local_y) <= half_height) {
        setGridCell(gx, gy, config_.obstacle_cost, grid);
      }
    }
  }
}
```

**改进点**：
- ✅ 考虑矩形的旋转角度 `yaw`
- ✅ 使用坐标变换将格子中心点转换到矩形的局部坐标系
- ✅ 在局部坐标系中判断是否在矩形内部
- ✅ 精确填充矩形轮廓，不会简化为圆形

---

### 修复 3：多边形障碍物 - 射线法精确填充

**修复后的代码**：
```cpp
void GridMapBuilderPlugin::addPolygonObstacle(...) {
  // 🔧 修复：使用射线法精确判断点是否在多边形内部
  
  // 计算多边形的包围盒
  double min_x = polygon.vertices[0].x;
  double min_y = polygon.vertices[0].y;
  double max_x = polygon.vertices[0].x;
  double max_y = polygon.vertices[0].y;
  
  for (const auto& vertex : polygon.vertices) {
    min_x = std::min(min_x, vertex.x);
    min_y = std::min(min_y, vertex.y);
    max_x = std::max(max_x, vertex.x);
    max_y = std::max(max_y, vertex.y);
  }
  
  // 遍历包围盒内的所有栅格
  for (int gy = grid_min_y; gy <= grid_max_y; ++gy) {
    for (int gx = grid_min_x; gx <= grid_max_x; ++gx) {
      // ✅ 计算栅格格子中心点的世界坐标
      double cell_center_x = grid.config.origin.x + (gx + 0.5) * grid.config.resolution;
      double cell_center_y = grid.config.origin.y + (gy + 0.5) * grid.config.resolution;
      
      // ✅ 使用射线法判断点是否在多边形内部
      if (isPointInPolygon(cell_center_x, cell_center_y, polygon.vertices)) {
        setGridCell(gx, gy, config_.obstacle_cost, grid);
      }
    }
  }
}

// ✅ 射线法实现
bool GridMapBuilderPlugin::isPointInPolygon(double px, double py,
                                           const std::vector<planning::Point2d>& vertices) const {
  int crossings = 0;
  size_t n = vertices.size();
  
  for (size_t i = 0; i < n; ++i) {
    size_t j = (i + 1) % n;
    const auto& vi = vertices[i];
    const auto& vj = vertices[j];
    
    // 检查射线是否与边相交
    if (((vi.y > py) != (vj.y > py)) &&
        (px < (vj.x - vi.x) * (py - vi.y) / (vj.y - vi.y) + vi.x)) {
      crossings++;
    }
  }
  
  // 奇数次相交表示在多边形内部
  return (crossings % 2) == 1;
}
```

**改进点**：
- ✅ 使用射线法（Ray Casting Algorithm）精确判断点是否在多边形内部
- ✅ 只遍历多边形的包围盒，提高效率
- ✅ 精确填充多边形轮廓，不会简化为圆形

---

## 📊 修复前后对比

| 障碍物类型 | 修复前 | 修复后 |
|-----------|--------|--------|
| **圆形** | 栅格空间判断，精度不足 | 世界坐标判断，精确 |
| **矩形** | 简化为圆形（对角线/2） | 考虑旋转的精确矩形 |
| **多边形** | 简化为包围圆 | 射线法精确填充 |
| **精度** | 占据区域过大 | 与真实轮廓一致 |
| **形状保留** | 全部变成圆形 | 保留原始形状 |

---

## 🧪 测试步骤

1. **重启 navsim-local**
2. **在 Web 界面放置不同类型的障碍物**：
   - 圆形障碍物（半径 1.0m）
   - 矩形障碍物（2m x 4m，旋转 45°）
   - 多边形障碍物（三角形或五边形）
3. **在可视化窗口中勾选 "Show Occupancy Grid"**
4. **观察栅格地图**：
   - 圆形障碍物应该显示为圆形（不是更大的圆）
   - 矩形障碍物应该显示为旋转后的矩形（不是圆形）
   - 多边形障碍物应该显示为多边形（不是圆形）
5. **对比障碍物轮廓和栅格地图**：
   - 栅格地图的占据区域应该与障碍物轮廓精确匹配

---

## 🎯 预期效果

### 修复前

```
圆形障碍物 (r=1.0m) → 栅格地图中显示为 r≈1.1m 的圆形 ❌
矩形障碍物 (2m x 4m) → 栅格地图中显示为 r=2.24m 的圆形 ❌
多边形障碍物 → 栅格地图中显示为包围圆 ❌
```

### 修复后

```
圆形障碍物 (r=1.0m) → 栅格地图中显示为 r=1.0m 的圆形 ✅
矩形障碍物 (2m x 4m, yaw=45°) → 栅格地图中显示为旋转 45° 的矩形 ✅
多边形障碍物 → 栅格地图中显示为精确的多边形 ✅
```

---

## 📝 技术细节

### 射线法（Ray Casting Algorithm）

**原理**：
从点 P 向任意方向发射一条射线，统计射线与多边形边界的交点数量：
- 奇数次相交 → 点在多边形内部
- 偶数次相交 → 点在多边形外部

**实现**：
```cpp
// 检查射线是否与边 (vi, vj) 相交
if (((vi.y > py) != (vj.y > py)) &&  // 边跨越射线的 y 坐标
    (px < (vj.x - vi.x) * (py - vi.y) / (vj.y - vi.y) + vi.x)) {  // 交点在 P 的右侧
  crossings++;
}
```

### 坐标变换（矩形旋转）

**原理**：
将世界坐标系中的点转换到矩形的局部坐标系，然后判断是否在矩形内部。

**公式**：
```
local_x = (world_x - rect.x) * cos(yaw) + (world_y - rect.y) * sin(yaw)
local_y = -(world_x - rect.x) * sin(yaw) + (world_y - rect.y) * cos(yaw)

在矩形内部的条件：
|local_x| <= width/2 && |local_y| <= height/2
```

---

---

## 🔧 补充修复：添加动态障碍物支持

### 问题描述

原始代码只将 BEV 静态障碍物添加到栅格地图，**完全忽略了动态障碍物的本体**。

这导致：
- 路径规划算法无法避开动态障碍物
- 可能规划出与动态障碍物碰撞的路径
- 动态障碍物在栅格地图中"不可见"

### 修复方案

在 `process()` 方法中添加动态障碍物处理：

```cpp
// 添加 BEV 静态障碍物
addBEVObstacles(input.bev_obstacles, *grid);

// 🔧 添加动态障碍物
addDynamicObstacles(input.dynamic_obstacles, *grid);

// 膨胀处理
inflateObstacles(*grid);
```

### 实现细节

```cpp
void GridMapBuilderPlugin::addDynamicObstacles(
    const std::vector<planning::DynamicObstacle>& dynamic_obstacles,
    planning::OccupancyGrid& grid) {
  for (const auto& dyn_obs : dynamic_obstacles) {
    if (dyn_obs.shape_type == "circle") {
      // 圆形动态障碍物
      planning::BEVObstacles::Circle circle;
      circle.center.x = dyn_obs.current_pose.x;
      circle.center.y = dyn_obs.current_pose.y;
      circle.radius = (dyn_obs.length + dyn_obs.width) / 4.0;
      addCircleObstacle(circle, grid);
    } else if (dyn_obs.shape_type == "rectangle") {
      // 矩形动态障碍物
      planning::BEVObstacles::Rectangle rect;
      rect.pose.x = dyn_obs.current_pose.x;
      rect.pose.y = dyn_obs.current_pose.y;
      rect.pose.yaw = dyn_obs.current_pose.yaw;
      rect.width = dyn_obs.width;
      rect.height = dyn_obs.length;
      addRectangleObstacle(rect, grid);
    }
  }
}
```

### 关键点

1. **使用当前位置**：`dyn_obs.current_pose`（不是预测轨迹）
2. **支持圆形和矩形**：根据 `shape_type` 选择处理方式
3. **考虑旋转**：矩形动态障碍物的 `yaw` 角度
4. **复用现有方法**：调用 `addCircleObstacle()` 和 `addRectangleObstacle()`

### 修改的文件

1. **`grid_map_builder_plugin.cpp`**
   - 第 92-99 行：在 `process()` 中添加动态障碍物处理
   - 第 155-197 行：实现 `addDynamicObstacles()` 方法

2. **`grid_map_builder_plugin.hpp`**
   - 第 107-110 行：添加 `addDynamicObstacles()` 方法声明

---

**修复完成时间**：2025-10-14
**编译状态**：✅ 成功
**测试状态**：⏳ 待用户验证

**修复内容总结**：
1. ✅ 圆形障碍物精度修复（世界坐标判断）
2. ✅ 矩形障碍物精确填充（考虑旋转）
3. ✅ 多边形障碍物精确填充（射线法）
4. ✅ 动态障碍物支持（圆形和矩形）

