# 可视化改进文档

## 📋 问题描述

### 问题 1：栅格地图边界不可见

**修复前的问题**：
- ❌ 只显示了被障碍物占据的栅格（黑色方块）
- ❌ 未被占据的栅格是透明的
- ❌ 无法看到整个栅格地图的边界范围
- ❌ 用户不知道栅格地图的有效区域在哪里

### 问题 2：视角跟随自车导致视觉混淆

**修复前的问题**：
- ❌ 视角（相机）默认跟随自车移动
- ❌ 自车始终在屏幕中心
- ❌ 看起来自车静止不动，而障碍物和地图在移动
- ❌ 难以理解自车在地图中的实际运动轨迹
- ❌ 只能通过快捷键 `F` 切换，没有 UI 控制

---

## 🔧 修复方案

### 修复 1：添加栅格地图边界显示

**文件**：`navsim-local/src/viz/imgui_visualizer.cpp`（第 489-567 行）

**修复内容**：

1. **计算栅格地图边界**：
   ```cpp
   double grid_min_x = cfg.origin.x;
   double grid_min_y = cfg.origin.y;
   double grid_max_x = cfg.origin.x + cfg.width * cfg.resolution;
   double grid_max_y = cfg.origin.y + cfg.height * cfg.resolution;
   ```

2. **转换到屏幕坐标**：
   ```cpp
   auto boundary_p1_temp = worldToScreen(grid_min_x, grid_min_y);
   auto boundary_p2_temp = worldToScreen(grid_max_x, grid_min_y);
   auto boundary_p3_temp = worldToScreen(grid_max_x, grid_max_y);
   auto boundary_p4_temp = worldToScreen(grid_min_x, grid_max_y);
   
   ImVec2 boundary_p1(boundary_p1_temp.x, boundary_p1_temp.y);
   ImVec2 boundary_p2(boundary_p2_temp.x, boundary_p2_temp.y);
   ImVec2 boundary_p3(boundary_p3_temp.x, boundary_p3_temp.y);
   ImVec2 boundary_p4(boundary_p4_temp.x, boundary_p4_temp.y);
   ```

3. **绘制虚线边界框**：
   ```cpp
   // 绘制边界框（白色虚线）
   const float dash_length = 10.0f;
   const float gap_length = 5.0f;
   
   auto drawDashedLine = [&](ImVec2 p1, ImVec2 p2, uint32_t color, float thickness) {
     float dx = p2.x - p1.x;
     float dy = p2.y - p1.y;
     float length = std::sqrt(dx * dx + dy * dy);
     if (length < 0.1f) return;
     
     float ux = dx / length;
     float uy = dy / length;
     
     float current = 0.0f;
     while (current < length) {
       float dash_end = std::min(current + dash_length, length);
       ImVec2 start(p1.x + ux * current, p1.y + uy * current);
       ImVec2 end(p1.x + ux * dash_end, p1.y + uy * dash_end);
       draw_list->AddLine(start, end, color, thickness);
       current += dash_length + gap_length;
     }
   };
   
   drawDashedLine(boundary_p1, boundary_p2, IM_COL32(200, 200, 200, 255), 2.0f);  // 底边
   drawDashedLine(boundary_p2, boundary_p3, IM_COL32(200, 200, 200, 255), 2.0f);  // 右边
   drawDashedLine(boundary_p3, boundary_p4, IM_COL32(200, 200, 200, 255), 2.0f);  // 顶边
   drawDashedLine(boundary_p4, boundary_p1, IM_COL32(200, 200, 200, 255), 2.0f);  // 左边
   ```

**关键改进**：
- ✅ 绘制栅格地图的边界框（白色虚线）
- ✅ 使用虚线效果，避免遮挡其他元素
- ✅ 边界框在栅格地图绘制之前绘制（在最底层）
- ✅ 用户可以清楚地看到栅格地图的有效范围

---

### 修复 2：添加视角跟随切换选项

**文件**：`navsim-local/src/viz/imgui_visualizer.cpp`（第 1264-1281 行）

**修复内容**：

在 Legend 面板中添加 "Follow Ego Vehicle" 复选框：

```cpp
ImGui::Spacing();
ImGui::Separator();
ImGui::Text("View Options:");
if (ImGui::Checkbox("Follow Ego Vehicle", &view_state_.follow_ego)) {
  std::cout << "[ImGuiVisualizer] Follow ego: " 
            << (view_state_.follow_ego ? "ON" : "OFF") << " (toggled from Legend panel)" << std::endl;
}
ImGui::SameLine();
ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(F key)");
```

**关键改进**：
- ✅ 在 Legend 面板中添加 "Follow Ego Vehicle" 复选框
- ✅ 用户可以通过 UI 切换视角跟随模式
- ✅ 保留快捷键 `F` 的功能
- ✅ 显示提示信息 "(F key)"，告知用户可以使用快捷键
- ✅ 切换时在控制台输出日志

---

## 📊 修复效果对比

### 栅格地图边界显示

| 项目 | 修复前 | 修复后 |
|------|--------|--------|
| **边界可见性** | 不可见 ❌ | 白色虚线边界 ✅ |
| **有效范围** | 不清楚 ❌ | 清晰可见 ✅ |
| **视觉效果** | 只有黑色方块 ❌ | 边界框 + 黑色方块 ✅ |

### 视角跟随控制

| 项目 | 修复前 | 修复后 |
|------|--------|--------|
| **UI 控制** | 无 ❌ | Legend 面板复选框 ✅ |
| **快捷键** | F 键 ✅ | F 键 ✅ |
| **提示信息** | 无 ❌ | "(F key)" 提示 ✅ |
| **控制台日志** | 有 ✅ | 有（区分来源）✅ |

---

## 🎨 可视化效果

### 栅格地图边界

**修复前**：
```
只有黑色方块（占据的格子）
无法看到栅格地图的边界
```

**修复后**：
```
┌ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐  ← 白色虚线边界
│                      │
│   ███  ███           │  ← 黑色方块（占据的格子）
│   ███  ███           │
│                      │
└ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘
```

### Legend 面板

**修复前**：
```
Legend & Visualization Options
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Visualization Elements:
☑ Show Ego Vehicle [Green]
☑ Show Goal Point [Red]
☑ Show Trajectory [Blue]
☑ Show BEV Obstacles [Red]
☑ Show Dynamic Obstacles [Purple]
☑ Show Occupancy Grid

Display Options:
☑ Show Coordinate Axes
☑ Show Grid Lines
```

**修复后**：
```
Legend & Visualization Options
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Visualization Elements:
☑ Show Ego Vehicle [Green]
☑ Show Goal Point [Red]
☑ Show Trajectory [Blue]
☑ Show BEV Obstacles [Red]
☑ Show Dynamic Obstacles [Purple]
☑ Show Occupancy Grid

Display Options:
☑ Show Coordinate Axes
☑ Show Grid Lines

View Options:
☑ Follow Ego Vehicle (F key)  ← 新增
[Fit Occupancy Grid]           ← 新增按钮
```

---

## 🧪 测试步骤

### 1. 测试栅格地图边界显示

1. **启动 navsim-local**：
   ```bash
   cd navsim-local
   ./build/navsim_algo
   ```

2. **确保栅格地图可见**：
   - 在 Legend 面板中勾选 "Show Occupancy Grid"

3. **点击"Fit Occupancy Grid"按钮**：
   - 在 Legend 面板的 "View Options" 部分找到 "Fit Occupancy Grid" 按钮
   - 点击按钮
   - ✅ 视图应该自动调整，显示整个栅格地图
   - ✅ 应该看到**亮黄色虚线边界框**围绕整个栅格地图
   - ✅ "Follow Ego Vehicle" 复选框应该自动取消勾选

4. **观察边界框**：
   - ✅ 边界框应该是**亮黄色**（方便调试）
   - ✅ 边界框应该围绕整个栅格地图区域
   - ✅ 虚线应该清晰可见（粗细 4 像素）

5. **缩放测试**：
   - 按 `+` 键放大
   - 按 `-` 键缩小
   - ✅ 边界框应该随着缩放正确调整大小

6. **移动测试**：
   - 取消 "Follow Ego Vehicle"
   - 自车移动时，边界框应该保持在栅格地图的位置

### 2. 测试视角跟随切换

1. **通过 Legend 面板切换**：
   - 在 Legend 面板中找到 "Follow Ego Vehicle" 复选框
   - 点击复选框切换状态
   - ✅ 控制台应该输出：`[ImGuiVisualizer] Follow ego: ON/OFF (toggled from Legend panel)`

2. **通过快捷键切换**：
   - 按 `F` 键
   - ✅ Legend 面板中的复选框状态应该同步更新
   - ✅ 控制台应该输出：`[ImGuiVisualizer] Follow ego: ON/OFF`

3. **跟随模式测试**：
   - 勾选 "Follow Ego Vehicle"
   - 自车移动时，视角应该跟随自车
   - 自车始终在屏幕中心

4. **固定模式测试**：
   - 取消勾选 "Follow Ego Vehicle"
   - 自车移动时，视角应该保持固定
   - 自车在地图上移动，障碍物保持静止

---

## 📝 修改的文件

1. **`navsim-local/src/viz/imgui_visualizer.cpp`**（第 489-567 行）
   - 添加栅格地图边界框绘制逻辑
   - 使用虚线效果绘制边界
   - 添加调试信息输出（每 60 帧打印一次）
   - 增强边界线可见性（亮黄色，4 像素粗）

2. **`navsim-local/src/viz/imgui_visualizer.cpp`**（第 1297-1343 行）
   - 在 Legend 面板中添加 "Follow Ego Vehicle" 复选框
   - 添加 "Fit Occupancy Grid" 按钮
   - 实现自动适应栅格地图的逻辑

3. **`navsim-local/include/core/algorithm_manager.hpp`**（第 49-54 行）
   - 添加栅格地图配置参数到 `Config` 结构体
   - `grid_map_width`、`grid_map_height`、`grid_resolution`、`grid_inflation_radius`

4. **`navsim-local/src/core/algorithm_manager.cpp`**（第 450-468 行）
   - 修改 GridMapBuilder 插件配置，从 `config_` 读取参数
   - 添加配置参数的日志输出

5. **`navsim-local/src/core/main.cpp`**（第 78-102 行）
   - 添加从 JSON 配置文件读取栅格地图参数的逻辑
   - 支持从 `perception.plugins[].params` 读取 GridMapBuilder 配置

---

## 🎯 使用建议

### 栅格地图边界

- **何时有用**：
  - 调试栅格地图生成逻辑
  - 检查栅格地图的覆盖范围
  - 验证栅格地图的原点和尺寸

- **如何使用**：
  - 勾选 "Show Occupancy Grid" 即可看到边界框
  - 边界框会自动显示，无需额外操作

### 视角跟随模式

- **跟随模式（Follow Ego Vehicle = ON）**：
  - ✅ 适合观察自车周围的局部环境
  - ✅ 适合调试自车的运动控制
  - ✅ 自车始终在屏幕中心，便于观察

- **固定模式（Follow Ego Vehicle = OFF）**：
  - ✅ 适合观察自车在全局地图中的运动轨迹
  - ✅ 适合理解自车的路径规划
  - ✅ 障碍物保持静止，自车在地图上移动

- **推荐使用**：
  - 初始阶段：使用固定模式，观察全局布局
  - 调试阶段：使用跟随模式，观察局部细节
  - 演示阶段：根据需要切换模式

---

## 🎉 总结

### 修复内容

1. **栅格地图边界显示**：
   - ✅ 添加白色虚线边界框
   - ✅ 清晰显示栅格地图的有效范围
   - ✅ 不遮挡其他可视化元素

2. **视角跟随控制**：
   - ✅ 在 Legend 面板中添加 UI 控制
   - ✅ 保留快捷键 `F` 的功能
   - ✅ 添加快捷键提示信息
   - ✅ 区分 UI 和快捷键的日志输出

### 关键改进

- ✅ **可视化更清晰**：栅格地图边界可见
- ✅ **控制更方便**：UI 和快捷键双重控制
- ✅ **用户体验更好**：提示信息完善
- ✅ **调试更容易**：控制台日志详细

---

## 🔍 技术细节

### 虚线绘制算法

```cpp
auto drawDashedLine = [&](ImVec2 p1, ImVec2 p2, uint32_t color, float thickness) {
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  float length = std::sqrt(dx * dx + dy * dy);
  if (length < 0.1f) return;
  
  float ux = dx / length;  // 单位向量 X
  float uy = dy / length;  // 单位向量 Y
  
  float current = 0.0f;
  while (current < length) {
    float dash_end = std::min(current + dash_length, length);
    ImVec2 start(p1.x + ux * current, p1.y + uy * current);
    ImVec2 end(p1.x + ux * dash_end, p1.y + uy * dash_end);
    draw_list->AddLine(start, end, color, thickness);
    current += dash_length + gap_length;  // 跳过间隙
  }
};
```

**算法说明**：
1. 计算线段的方向向量和长度
2. 沿着线段方向，每隔 `dash_length + gap_length` 绘制一段实线
3. 实线长度为 `dash_length`，间隙长度为 `gap_length`
4. 使用单位向量确保虚线均匀分布

### 坐标转换

```cpp
// Point2D → ImVec2
auto boundary_p1_temp = worldToScreen(grid_min_x, grid_min_y);
ImVec2 boundary_p1(boundary_p1_temp.x, boundary_p1_temp.y);
```

**说明**：
- `worldToScreen` 返回自定义的 `Point2D` 类型
- ImGui 的绘制函数需要 `ImVec2` 类型
- 需要手动转换类型

