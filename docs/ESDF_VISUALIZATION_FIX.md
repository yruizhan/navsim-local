# ESDF 可视化问题修复

## 🐛 问题描述

用户报告：**ESDF 地图显示全是红色**

可能的原因：
1. ESDF 计算错误
2. 可视化代码错误
3. 数据格式不匹配

---

## 🔍 问题分析

### 1. 可视化代码期望

查看 `src/viz/imgui_visualizer.cpp` 第 674-690 行：

```cpp
// 颜色编码：蓝色（远离障碍物）-> 绿色 -> 黄色 -> 红色（接近障碍物）
double normalized_dist = std::clamp(distance / cfg.max_distance, 0.0, 1.0);

if (normalized_dist > 0.5) {
  // 蓝色 -> 绿色
  r = 0;
  g = static_cast<uint8_t>(255 * (1.0 - t));
  b = static_cast<uint8_t>(255 * t);
} else {
  // 红色 -> 黄色 -> 绿色
  r = static_cast<uint8_t>(255 * (1.0 - t));
  g = static_cast<uint8_t>(255 * t);
  b = 0;
}
```

**期望**：`distance` 是**正值**（0 到 max_distance）

### 2. ESDF 计算输出

查看 `plugins/perception/esdf_builder/src/esdf_map.cpp` 第 112-117 行：

```cpp
// 转换为有符号距离场（负值表示障碍物内部）
for (int i = 0; i < GLXY_SIZE_; ++i) {
  if (gridmap_[i] == Occupied) {
    distance_buffer_all_[i] = -distance_buffer_all_[i];  // ❌ 负值！
  }
}
```

**实际**：障碍物内部的距离是**负值**

### 3. 数据复制代码

查看 `plugins/perception/esdf_builder/src/esdf_builder_plugin.cpp` 第 103-106 行（修复前）：

```cpp
// 复制距离场数据
for (int i = 0; i < grid_width_ * grid_height_; ++i) {
  esdf_map_navsim->data[i] = esdf_map_->getDistance(...) * resolution_;
  // ❌ 没有取绝对值，负值被直接复制
}
```

### 4. 可视化过滤条件

查看 `src/viz/imgui_visualizer.cpp` 第 664 行：

```cpp
if (distance >= cfg.max_distance * 0.9) continue;  // 跳过距离太大的格子
```

**问题**：
- 如果所有格子的距离都是 `max_distance`（初始值），它们都会被跳过
- 如果距离是负值，`normalized_dist` 会被 clamp 到 0.0，导致**纯红色**

---

## ✅ 修复方案

### 修复代码

在 `plugins/perception/esdf_builder/src/esdf_builder_plugin.cpp` 中：

```cpp
// 复制距离场数据（取绝对值，因为可视化需要正值）
int occupied_count = 0;
double min_dist = std::numeric_limits<double>::max();
double max_dist = std::numeric_limits<double>::lowest();

for (int i = 0; i < grid_width_ * grid_height_; ++i) {
  double dist_grid = esdf_map_->getDistance(esdf_map_->vectornum2gridIndex(i));
  double dist_meter = std::abs(dist_grid) * resolution_;  // ✅ 取绝对值
  esdf_map_navsim->data[i] = dist_meter;
  
  if (dist_grid < 0.01) {  // 障碍物
    occupied_count++;
  }
  min_dist = std::min(min_dist, dist_meter);
  max_dist = std::max(max_dist, dist_meter);
}

// 每 60 帧打印一次 ESDF 统计信息
static int esdf_frame_count = 0;
if (++esdf_frame_count % 60 == 0) {
  std::cout << "[ESDFBuilder] ESDF stats: occupied=" << occupied_count 
            << ", min_dist=" << min_dist << "m, max_dist=" << max_dist << "m" << std::endl;
}
```

### 修复要点

1. **取绝对值**：`std::abs(dist_grid)` 确保距离为正值
2. **添加调试信息**：打印 ESDF 统计信息，帮助诊断问题
3. **保持 JPS 兼容性**：`esdf_map_` 内部仍然使用有符号距离场（负值表示障碍物内部）

---

## 🎨 预期可视化效果

修复后，ESDF 地图应该显示：

```
🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵  ← 蓝色：远离障碍物（安全）
🔵🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🔵  ← 绿色：中等距离
🔵🟢🟡🟡🟡🟡🟡🟡🟡🟡🟢🔵  ← 黄色：接近障碍物
🔵🟢🟡🔴🔴🔴🔴🔴🔴🟡🟢🔵  ← 红色：非常接近障碍物
🔵🟢🟡🔴⬛⬛⬛⬛🔴🟡🟢🔵  ← 黑色：障碍物（距离=0）
🔵🟢🟡🔴🔴🔴🔴🔴🔴🟡🟢🔵
🔵🟢🟡🟡🟡🟡🟡🟡🟡🟡🟢🔵
🔵🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🔵
🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵
```

### 颜色映射

| 距离 (m) | 归一化距离 | 颜色 | RGB |
|---------|-----------|------|-----|
| 5.0 | 1.0 | 纯蓝色 | (0, 0, 255) |
| 3.75 | 0.75 | 蓝绿色 | (0, 128, 255) |
| 2.5 | 0.5 | 纯绿色 | (0, 255, 0) |
| 1.25 | 0.25 | 黄绿色 | (128, 255, 0) |
| 0.5 | 0.1 | 橙色 | (230, 51, 0) |
| 0.0 | 0.0 | 纯红色 | (255, 0, 0) |

---

## 🧪 测试方法

### 1. 编译

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local/build
make esdf_builder_plugin -j8
```

### 2. 运行

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
./build_with_visualization.sh
```

### 3. 检查可视化

1. 在 Legend 面板中勾选 "Show ESDF Map"
2. 观察 ESDF 地图颜色：
   - ✅ 应该看到蓝色、绿色、黄色、红色的渐变
   - ❌ 不应该全是红色

### 4. 检查调试输出

每 60 帧应该看到类似输出：

```
[ESDFBuilder] ESDF stats: occupied=1234, min_dist=0.0m, max_dist=5.0m
```

**正常情况**：
- `occupied` > 0：有障碍物
- `min_dist` ≈ 0.0：障碍物边界
- `max_dist` ≈ `max_distance`：远离障碍物的区域

**异常情况**：
- `occupied` = 0：没有障碍物（检查 BEV 障碍物输入）
- `min_dist` = `max_dist` = 5.0：所有格子距离相同（ESDF 计算失败）

---

## 🔧 其他可能的问题

### 问题 1：没有障碍物

**症状**：ESDF 地图全是蓝色或不显示

**原因**：`occupancy_grid_` 全是 0（没有障碍物）

**检查**：
```cpp
// 在 buildOccupancyGrid() 后添加
int occ_count = std::count_if(occupancy_grid_.begin(), occupancy_grid_.end(),
                               [](uint8_t v) { return v > 50; });
std::cout << "[ESDFBuilder] Occupied cells: " << occ_count << std::endl;
```

### 问题 2：ESDF 计算失败

**症状**：所有距离都是 `max_distance`

**原因**：`fillESDF()` 算法有问题

**检查**：
- 查看 `fillESDF()` 是否正确实现
- 检查是否有数组越界

### 问题 3：坐标转换错误

**症状**：ESDF 地图位置不对

**原因**：`origin` 计算错误或坐标转换错误

**检查**：
```cpp
std::cout << "[ESDFBuilder] Origin: (" << origin.x << ", " << origin.y << ")" << std::endl;
std::cout << "[ESDFBuilder] Ego: (" << input.ego.pose.x << ", " << input.ego.pose.y << ")" << std::endl;
```

---

## 📊 调试信息说明

### ESDF 统计信息

```
[ESDFBuilder] ESDF stats: occupied=1234, min_dist=0.0m, max_dist=5.0m
```

- **occupied**：障碍物格子数量
  - 正常：> 0
  - 异常：= 0（没有障碍物）

- **min_dist**：最小距离
  - 正常：≈ 0.0（障碍物边界）
  - 异常：> 0.5（没有障碍物或 ESDF 计算错误）

- **max_dist**：最大距离
  - 正常：≈ `max_distance`（配置值，默认 5.0m）
  - 异常：< `max_distance` * 0.5（地图太小或障碍物太多）

### 处理时间

```
[ESDFBuilder] Processing time: 2.5 ms
```

- **正常**：< 10 ms
- **慢**：> 20 ms（可能需要优化）

---

## ✅ 修复总结

### 根本原因

**数据格式不匹配**：
- ESDF 计算输出：有符号距离场（负值表示障碍物内部）
- 可视化期望：无符号距离场（正值）

### 修复方法

在数据复制时**取绝对值**：

```cpp
double dist_meter = std::abs(dist_grid) * resolution_;
```

### 影响范围

- ✅ **可视化**：修复后正常显示
- ✅ **JPS 规划器**：不受影响（使用 `esdf_map_` 内部数据，仍然是有符号距离场）

### 后续工作

1. **测试可视化**：确认 ESDF 地图显示正常
2. **测试 JPS 规划器**：确认 JPS 可以正确使用 `esdf_map_`
3. **性能优化**：如果 ESDF 计算太慢，考虑优化

---

## 📚 相关文档

- `docs/ESDF_BUILDER_REFACTOR_SUMMARY.md` - ESDF Builder 重构总结
- `docs/ESDF_BUILDER_REFACTOR_PLAN.md` - ESDF Builder 重构计划
- `ESDF_VISUALIZATION.md` - ESDF 可视化说明

---

**现在请重新编译并运行，检查 ESDF 地图是否正常显示！** 🚀

