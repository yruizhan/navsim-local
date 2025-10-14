# 图例面板功能说明

## 🎨 新增功能

为可视化界面添加了**图例面板（Legend Panel）**，允许用户通过勾选框控制各个可视化元素的显示/隐藏。

---

## 📋 功能列表

### 可控制的可视化元素

| 元素 | 颜色 | 默认状态 | 说明 |
|------|------|----------|------|
| **Ego Vehicle** | 🟢 绿色 | ✅ 显示 | 自车（圆形 + 朝向箭头） |
| **Goal Point** | 🔴 红色 | ✅ 显示 | 目标点 |
| **Trajectory** | 🔵 青色 | ✅ 显示 | 规划轨迹 |
| **BEV Obstacles** | 多种颜色 | ✅ 显示 | BEV 静态障碍物 |
| - Circles | 🔴 红色 | - | 圆形障碍物 |
| - Rectangles | 🟢 绿色 | - | 矩形障碍物 |
| - Polygons | 🟡 黄色 | - | 多边形障碍物 |
| **Dynamic Obstacles** | 🟣 紫色 | ✅ 显示 | 动态障碍物 |
| **Occupancy Grid** | 灰色 | ✅ 显示 | 栅格地图 |
| **Coordinate Axes** | 红/绿 | ✅ 显示 | 坐标轴（X 轴红色，Y 轴绿色） |
| **Grid Lines** | 深灰 | ✅ 显示 | 网格线 |

---

## 🖼️ 界面布局

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│                     Main Scene Window                           │
│                     (1000 x 900)                                │
│                                                                 │
│                                                                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                                                    ┌──────────────┐
                                                    │  Debug Info  │
                                                    │  (390 x 450) │
                                                    │              │
                                                    └──────────────┘
                                                    ┌──────────────┐
                                                    │   Legend &   │
                                                    │Visualization │
                                                    │   Options    │
                                                    │  (390 x 450) │
                                                    │              │
                                                    └──────────────┘
```

---

## 🎮 使用方法

### 1. 启动可视化

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json
```

### 2. 打开图例面板

可视化窗口右侧会自动显示两个面板：
- **Debug Info**（上方）：显示调试信息
- **Legend & Visualization Options**（下方）：图例和可视化选项

### 3. 控制可视化元素

在 **Legend & Visualization Options** 面板中：

#### 勾选/取消勾选元素
- ✅ **勾选**：显示该元素
- ⬜ **取消勾选**：隐藏该元素

#### 快捷按钮
- **Show All**：显示所有元素
- **Hide All**：隐藏所有元素

#### 统计信息
面板底部显示当前场景中各类元素的数量：
- BEV Circles: X
- BEV Rectangles: X
- BEV Polygons: X
- Dynamic Obstacles: X
- Trajectory Points: X

---

## 🔧 实现细节

### 修改的文件

1. **`navsim-local/include/viz/imgui_visualizer.hpp`**
   - 添加 `VisualizationOptions` 结构体
   - 添加 `renderLegendPanel()` 方法声明

2. **`navsim-local/src/viz/imgui_visualizer.cpp`**
   - 实现 `renderLegendPanel()` 方法
   - 在 `renderScene()` 中为各个绘制部分添加条件判断
   - 在 `endFrame()` 中调用 `renderLegendPanel()`

### 核心数据结构

```cpp
struct VisualizationOptions {
  bool show_ego = true;              // 显示自车
  bool show_goal = true;             // 显示目标点
  bool show_trajectory = true;       // 显示规划轨迹
  bool show_bev_obstacles = true;    // 显示 BEV 静态障碍物
  bool show_dynamic_obstacles = true;// 显示动态障碍物
  bool show_occupancy_grid = true;   // 显示栅格地图
  bool show_coordinate_axes = true;  // 显示坐标轴
  bool show_grid_lines = true;       // 显示网格线
} viz_options_;
```

### 条件渲染逻辑

```cpp
// 示例：只有在 show_ego 为 true 时才绘制自车
if (viz_options_.show_ego) {
  // 绘制自车的代码
  auto ego_pos = worldToScreen(ego_.pose.x, ego_.pose.y);
  draw_list->AddCircleFilled(...);
  // ...
}
```

---

## 📊 使用场景

### 场景 1：只查看障碍物
1. 点击 **Hide All** 隐藏所有元素
2. 勾选 **Show BEV Obstacles**
3. 勾选 **Show Dynamic Obstacles**
4. 勾选 **Show Coordinate Axes**（可选，用于参考）

### 场景 2：只查看规划轨迹
1. 点击 **Hide All**
2. 勾选 **Show Ego Vehicle**
3. 勾选 **Show Goal Point**
4. 勾选 **Show Trajectory**

### 场景 3：调试栅格地图
1. 点击 **Hide All**
2. 勾选 **Show Occupancy Grid**
3. 勾选 **Show Ego Vehicle**（可选）
4. 勾选 **Show Grid Lines**（可选）

### 场景 4：性能优化
如果场景中元素过多导致帧率下降，可以：
1. 取消勾选 **Show Grid Lines**（网格线绘制开销较大）
2. 取消勾选不需要的障碍物类型

---

## 🎯 优势

1. **灵活性**：用户可以根据需要自由组合显示的元素
2. **清晰度**：隐藏不需要的元素，减少视觉干扰
3. **调试效率**：快速定位问题（例如只显示障碍物来检查碰撞检测）
4. **性能优化**：减少不必要的绘制，提高帧率
5. **用户友好**：直观的勾选框界面，无需修改代码

---

## 🚀 未来扩展

可以考虑添加以下功能：

1. **颜色自定义**：允许用户自定义各元素的颜色
2. **透明度调节**：添加滑块控制元素的透明度
3. **图层顺序**：允许用户调整绘制顺序
4. **预设方案**：保存/加载常用的可视化配置
5. **快捷键**：为常用元素添加快捷键切换（例如 `E` 切换自车显示）
6. **分组折叠**：将相关元素分组，支持折叠/展开

---

## 📝 注意事项

1. **默认状态**：所有元素默认都是显示的
2. **实时更新**：勾选/取消勾选后立即生效，无需重启
3. **状态保持**：在同一次运行中，勾选状态会保持（但重启后会恢复默认）
4. **性能影响**：隐藏元素可以减少绘制开销，提高帧率

---

**功能完成时间**：2025-10-14  
**编译状态**：✅ 成功  
**测试状态**：⏳ 待用户验证

