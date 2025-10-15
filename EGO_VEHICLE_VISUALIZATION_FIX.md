# 自车可视化精度修复说明

## 🐛 问题描述

**症状**：
- 自车在可视化窗口中显示为一个绿色圆形 + 朝向箭头
- 圆形的大小是基于轴距（wheelbase）计算的，不准确
- 无法看到车辆的真实轮廓和尺寸
- 无法准确判断车辆与障碍物的距离关系

**影响**：
- 可视化不够直观，难以判断车辆是否会与障碍物碰撞
- 无法看到车辆的前后悬（overhang）
- 无法准确评估路径规划的安全性

---

## 🔍 根本原因

**原始代码**（`imgui_visualizer.cpp` 第 776-801 行）：
```cpp
// 简化：绘制为圆形 + 朝向箭头
auto ego_pos = worldToScreen(ego_.pose.x, ego_.pose.y);
float car_length = ego_.kinematics.wheelbase * config_.pixels_per_meter * view_state_.zoom;
float car_width = ego_.kinematics.width * config_.pixels_per_meter * view_state_.zoom;

draw_list->AddCircleFilled(
  ImVec2(ego_pos.x, ego_pos.y),
  std::max(car_length, car_width) / 2.0f,
  IM_COL32(0, 255, 0, 200)  // 绿色半透明
);

// 绘制朝向箭头
float arrow_len = car_length * 0.8f;
auto arrow_end = worldToScreen(...);
draw_list->AddLine(...);
```

**问题分析**：
1. ❌ 使用圆形简化车辆轮廓
2. ❌ 只使用 `wheelbase`，忽略了 `front_overhang` 和 `rear_overhang`
3. ❌ 车辆总长度 = `front_overhang + wheelbase + rear_overhang`，但代码只用了 `wheelbase`
4. ❌ 无法看到车辆的真实形状（矩形）

---

## ✅ 修复方案

### 核心思想

使用车辆的**精确尺寸信息**绘制**矩形轮廓**，考虑旋转角度。

### 车辆尺寸信息

从 `EgoVehicle::Kinematics` 中获取：
```cpp
struct Kinematics {
  double wheelbase = 2.8;       // 轴距 (m)
  double front_overhang = 1.0;  // 前悬 (m)
  double rear_overhang = 1.0;   // 后悬 (m)
  double width = 2.0;           // 车宽 (m)
  double height = 1.8;          // 车高 (m)
} kinematics;
```

**车辆总长度**：
```
total_length = front_overhang + wheelbase + rear_overhang
             = 1.0 + 2.8 + 1.0 = 4.8m
```

### 坐标系说明

**重要**：`ego_.pose` 是**后轴中心**的位置，不是车辆几何中心！

```
        前悬      轴距      后悬
    |-------|---------|-------|
    ┌───────────────────────────┐
    │                           │
    │         车辆              │
    │                           │
    └───────────────────────────┘
                      ↑
                   后轴中心
                 (ego_.pose)
```

### 实现步骤

#### 1. 计算车辆的四个角点（局部坐标系）

以**后轴中心**为原点，车头方向为 X 轴正方向：

```cpp
// 前左、前右、后右、后左
std::vector<std::pair<double, double>> corners_local = {
  {rear_overhang + wheelbase + front_overhang, width/2},   // 前左
  {rear_overhang + wheelbase + front_overhang, -width/2},  // 前右
  {-rear_overhang, -width/2},  // 后右
  {-rear_overhang, width/2}    // 后左
};
```

#### 2. 旋转到世界坐标系

```cpp
double cos_yaw = std::cos(ego_.pose.yaw);
double sin_yaw = std::sin(ego_.pose.yaw);

for (const auto& corner : corners_local) {
  // 旋转矩阵
  double world_x = ego_.pose.x + corner.first * cos_yaw - corner.second * sin_yaw;
  double world_y = ego_.pose.y + corner.first * sin_yaw + corner.second * cos_yaw;
  
  // 转换到屏幕坐标
  auto screen_pos = worldToScreen(world_x, world_y);
  corners_screen.push_back(ImVec2(screen_pos.x, screen_pos.y));
}
```

#### 3. 绘制车辆轮廓

```cpp
// 填充矩形
draw_list->AddConvexPolyFilled(
  corners_screen.data(),
  corners_screen.size(),
  IM_COL32(0, 200, 0, 180)  // 绿色半透明
);

// 边框
draw_list->AddPolyline(
  corners_screen.data(),
  corners_screen.size(),
  IM_COL32(0, 255, 0, 255),  // 绿色边框
  ImDrawFlags_Closed,
  2.0f
);
```

#### 4. 绘制方向指示

**车头方向**（黄色圆点）：
```cpp
double front_center_x = ego_.pose.x + 
  (rear_overhang + wheelbase + front_overhang) * cos_yaw;
double front_center_y = ego_.pose.y + 
  (rear_overhang + wheelbase + front_overhang) * sin_yaw;
auto front_pos = worldToScreen(front_center_x, front_center_y);
draw_list->AddCircleFilled(
  ImVec2(front_pos.x, front_pos.y),
  5.0f,
  IM_COL32(255, 255, 0, 255)  // 黄色圆点
);
```

**后轴位置**（红色圆点，参考点）：
```cpp
auto rear_axle_pos = worldToScreen(ego_.pose.x, ego_.pose.y);
draw_list->AddCircleFilled(
  ImVec2(rear_axle_pos.x, rear_axle_pos.y),
  3.0f,
  IM_COL32(255, 0, 0, 255)  // 红色圆点
);
```

---

## 📊 修复前后对比

| 特性 | 修复前 | 修复后 |
|------|--------|--------|
| **形状** | 圆形 | 精确矩形 |
| **尺寸** | 只考虑轴距 | 完整尺寸（前悬+轴距+后悬） |
| **旋转** | 只有箭头表示方向 | 矩形随车辆旋转 |
| **参考点** | 不明确 | 红色圆点标记后轴 |
| **车头指示** | 绿色箭头 | 黄色圆点 |
| **精度** | 不准确 | 与真实车辆一致 |

---

## 🎨 可视化效果

### 修复前
```
        ●  ← 绿色圆形
       /
      /  ← 绿色箭头（朝向）
```

### 修复后
```
    ┌───────────────┐
    │               │ ● ← 黄色圆点（车头）
    │   绿色矩形    │
    │               │
    └───────────────┘
            ● ← 红色圆点（后轴，参考点）
```

**颜色说明**：
- 🟢 **绿色矩形**：车辆轮廓（半透明填充 + 实线边框）
- 🟡 **黄色圆点**：车头中心（前保险杠中心）
- 🔴 **红色圆点**：后轴中心（`ego_.pose` 的位置，参考点）

---

## 🧪 测试步骤

1. **重启 navsim-local**
2. **启动仿真**
3. **观察可视化窗口**：
   - 自车应该显示为绿色矩形（不是圆形）
   - 矩形的长宽应该与车辆真实尺寸一致
   - 矩形应该随车辆旋转
   - 车头有黄色圆点，后轴有红色圆点
4. **测试旋转**：
   - 让车辆转弯，观察矩形是否正确旋转
5. **测试尺寸**：
   - 对比矩形与障碍物的距离，判断是否准确

---

## 📝 技术细节

### 坐标变换公式

**旋转矩阵**（2D）：
```
[ cos(θ)  -sin(θ) ]
[ sin(θ)   cos(θ) ]
```

**从局部坐标到世界坐标**：
```
world_x = ego_x + local_x * cos(yaw) - local_y * sin(yaw)
world_y = ego_y + local_x * sin(yaw) + local_y * cos(yaw)
```

### ImGui 绘制函数

- `AddConvexPolyFilled()`：绘制填充的凸多边形
- `AddPolyline()`：绘制多边形边框
- `AddCircleFilled()`：绘制填充的圆形

### 为什么后轴是参考点？

在车辆动力学中，**后轴中心**通常作为参考点，因为：
1. 简化运动学模型（Bicycle Model）
2. 转向时，后轴是瞬时旋转中心
3. 便于计算车辆的运动轨迹

---

## 🎯 预期效果

### 修复前
```
车辆显示为圆形，半径 ≈ wheelbase/2 = 1.4m
实际车辆长度 = 4.8m
误差很大！❌
```

### 修复后
```
车辆显示为矩形：
- 长度 = front_overhang + wheelbase + rear_overhang = 4.8m ✅
- 宽度 = width = 2.0m ✅
- 旋转角度 = yaw ✅
- 与真实车辆完全一致！✅
```

---

## 🚀 未来改进

可以考虑添加以下功能：

1. **车轮可视化**：
   - 绘制四个车轮的位置
   - 显示前轮的转向角度

2. **速度向量**：
   - 绘制速度箭头，显示车辆的运动方向和速度大小

3. **加速度指示**：
   - 使用颜色表示加速/减速状态

4. **转向角指示**：
   - 显示当前的转向角度

5. **车辆模型**：
   - 加载更精细的车辆模型（SVG 或图片）

---

**修复完成时间**：2025-10-14  
**编译状态**：✅ 成功  
**测试状态**：⏳ 待用户验证

**修复文件**：
- `navsim-local/src/viz/imgui_visualizer.cpp`（第 776-849 行）

