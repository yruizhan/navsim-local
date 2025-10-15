# ESDF 地图可视化说明

## ✅ 已添加 ESDF 可视化功能

### 🎨 可视化效果

ESDF 地图使用**颜色编码**显示距离场：

```
蓝色 (Blue)   ← 远离障碍物（安全区域）
    ↓
绿色 (Green)  ← 中等距离
    ↓
黄色 (Yellow) ← 接近障碍物
    ↓
红色 (Red)    ← 非常接近障碍物（危险区域）
```

### 📊 可视化元素

1. **ESDF 距离场**：
   - 颜色编码的栅格
   - 半透明显示（不遮挡其他元素）
   - 根据缩放级别自动调整采样率（优化性能）

2. **ESDF 边界框**：
   - 青色虚线边框
   - 3 像素粗细
   - 标识 ESDF 地图的范围

---

## 🎛️ 如何启用/禁用 ESDF 可视化

### 在 Legend 面板中

运行 navsim-local 后，在右侧的 **Legend & Visualization Options** 面板中：

```
Legend & Visualization Options
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Visualization Elements:
☑ Show Ego Vehicle [Green]
☑ Show Goal Point [Red]
☑ Show Trajectory [Blue]
☑ Show BEV Obstacles [Red]
☑ Show Dynamic Obstacles [Purple]
☑ Show Occupancy Grid [Gray]
☑ Show ESDF Map [Cyan Border]        ← 🔑 勾选/取消勾选
  • Color: Blue (far) -> Green -> Yellow -> Red (near)
```

### 快捷操作

- **勾选** "Show ESDF Map" → 显示 ESDF 地图
- **取消勾选** "Show ESDF Map" → 隐藏 ESDF 地图

---

## 🎨 颜色编码详解

### 距离到颜色的映射

```cpp
normalized_distance = distance / max_distance  // 归一化到 [0, 1]

if (normalized_distance > 0.5) {
  // 远离障碍物：蓝色 -> 绿色
  蓝色分量 = 255 * (normalized_distance - 0.5) * 2
  绿色分量 = 255 * (1 - (normalized_distance - 0.5) * 2)
  红色分量 = 0
} else {
  // 接近障碍物：红色 -> 黄色 -> 绿色
  红色分量 = 255 * (1 - normalized_distance * 2)
  绿色分量 = 255 * normalized_distance * 2
  蓝色分量 = 0
}
```

### 示例

假设 `max_distance = 5.0m`：

| 距离 (m) | 归一化距离 | 颜色 | 说明 |
|---------|-----------|------|------|
| 5.0 | 1.0 | 纯蓝色 (0, 0, 255) | 最远（最安全） |
| 3.75 | 0.75 | 蓝绿色 (0, 128, 255) | 较远 |
| 2.5 | 0.5 | 纯绿色 (0, 255, 0) | 中等距离 |
| 1.25 | 0.25 | 黄绿色 (128, 255, 0) | 较近 |
| 0.5 | 0.1 | 橙色 (230, 51, 0) | 很近 |
| 0.0 | 0.0 | 纯红色 (255, 0, 0) | 障碍物边界 |

---

## ⚡ 性能优化

### 自适应采样

ESDF 可视化使用**自适应采样**来优化性能：

```cpp
int sample_step = max(1, static_cast<int>(2.0 / view_state_.zoom));
```

- **放大视图** (zoom > 1.0)：`sample_step = 1`，绘制所有格子（高精度）
- **缩小视图** (zoom < 1.0)：`sample_step > 1`，跳过部分格子（优化性能）

### 距离过滤

跳过距离太大的格子（接近 `max_distance`）：

```cpp
if (distance >= max_distance * 0.9) continue;
```

这样可以：
- 减少绘制的格子数量
- 提高渲染性能
- 突出显示障碍物附近的区域

---

## 🔍 调试信息

### 在 Debug Info 面板中

运行时，右侧的 **Debug Info** 面板会显示 ESDF 信息：

```
Debug Info
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ESDF Map: 300x300 @0.10m (max=5.0m)
ESDF Size: 300x300
ESDF Resolution: 0.1m
ESDF Max Distance: 5.0m
```

---

## 🎯 使用场景

### 场景 1：查看障碍物周围的安全距离

1. 启用 ESDF 可视化
2. 观察障碍物周围的颜色变化
3. 红色区域 = 危险，蓝色区域 = 安全

### 场景 2：调试梯度优化规划器

1. 同时启用 ESDF 和轨迹显示
2. 观察轨迹是否沿着梯度方向远离障碍物
3. 检查轨迹是否在安全区域（绿色/蓝色）

### 场景 3：对比 Occupancy Grid 和 ESDF

1. 同时启用 Occupancy Grid 和 ESDF
2. Occupancy Grid 显示离散的占据/自由状态
3. ESDF 显示连续的距离场

---

## 🎨 可视化层级

从下到上的绘制顺序：

```
1. 坐标轴和网格线（最底层）
2. Occupancy Grid（灰色栅格）
3. ESDF Map（彩色距离场）← 半透明，不遮挡其他元素
4. BEV 障碍物（红色/绿色/黄色）
5. 动态障碍物（紫色）
6. 轨迹（蓝色）
7. 自车（绿色）
8. 目标点（红色）
```

---

## 📝 配置示例

### 高精度 ESDF（小范围）

```json
{
  "name": "ESDFBuilder",
  "enabled": true,
  "priority": 90,
  "params": {
    "resolution": 0.05,      // 更精细
    "map_width": 20.0,       // 更小范围
    "map_height": 20.0,
    "max_distance": 3.0,     // 更小的最大距离
    "include_dynamic": true
  }
}
```

**效果**：
- 更精细的距离场
- 更快的渲染速度（格子少）
- 适合局部路径规划

### 低精度 ESDF（大范围）

```json
{
  "name": "ESDFBuilder",
  "enabled": true,
  "priority": 90,
  "params": {
    "resolution": 0.2,       // 更粗糙
    "map_width": 50.0,       // 更大范围
    "map_height": 50.0,
    "max_distance": 10.0,    // 更大的最大距离
    "include_dynamic": false // 不包含动态障碍物
  }
}
```

**效果**：
- 更粗糙的距离场
- 更大的覆盖范围
- 适合全局路径规划

---

## 🚀 运行和测试

### 1. 编译

```bash
cd navsim-local
./build_with_visualization.sh
```

### 2. 运行

脚本会自动编译并运行 navsim_algo。

### 3. 查看 ESDF

1. 在 Legend 面板中勾选 "Show ESDF Map"
2. 观察场景中的彩色距离场
3. 青色虚线边框标识 ESDF 范围

### 4. 调整视图

- 使用 `+/-` 键缩放
- 使用鼠标拖拽平移
- 点击 "Fit Occupancy Grid" 按钮自动适应

---

## 🎯 预期效果

### 静态场景

```
┌─────────────────────────────────┐
│                                 │
│    🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵    │  蓝色 = 远离障碍物
│    🔵🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🔵    │  绿色 = 中等距离
│    🔵🟢🟡🟡🟡🟡🟡🟡🟡🟡🟢🔵    │  黄色 = 接近障碍物
│    🔵🟢🟡🔴🔴🔴🔴🔴🔴🟡🟢🔵    │  红色 = 障碍物边界
│    🔵🟢🟡🔴⬛⬛⬛⬛🔴🟡🟢🔵    │  黑色 = 障碍物
│    🔵🟢🟡🔴🔴🔴🔴🔴🔴🟡🟢🔵    │
│    🔵🟢🟡🟡🟡🟡🟡🟡🟡🟡🟢🔵    │
│    🔵🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🔵    │
│    🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵    │
│                                 │
└─────────────────────────────────┘
```

### 动态场景

- 动态障碍物周围也会有距离场
- 随着障碍物移动，距离场实时更新
- 颜色变化反映安全距离的变化

---

## ✅ 总结

### 已实现的功能

1. ✅ ESDF 距离场可视化（颜色编码）
2. ✅ ESDF 边界框显示（青色虚线）
3. ✅ Legend 面板控制（启用/禁用）
4. ✅ 自适应采样（性能优化）
5. ✅ 调试信息显示
6. ✅ 半透明渲染（不遮挡其他元素）

### 使用方法

1. 运行 navsim-local
2. 在 Legend 面板中勾选 "Show ESDF Map"
3. 观察彩色距离场
4. 蓝色 = 安全，红色 = 危险

---

**现在您可以在可视化界面中看到 ESDF 地图了！** 🎉

