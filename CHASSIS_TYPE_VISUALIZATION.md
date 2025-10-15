# 底盘类型可视化支持说明

## 🎯 问题描述

**原始问题**：
- 自车可视化使用固定的尺寸参数（`wheelbase=2.8`, `front_overhang=1.0`, `rear_overhang=1.0`, `width=2.0`）
- 没有根据 WorldTick 中实际下发的底盘类型和尺寸进行可视化
- 所有底盘类型都显示为相同的矩形，无法区分

**影响**：
- 差速底盘（小型机器人）显示为大型汽车 ❌
- 履带底盘显示为普通汽车 ❌
- 无法直观看出底盘类型
- 尺寸不准确，影响碰撞判断

---

## 🔧 修复方案

### 核心思想

1. **从 WorldTick 中解析底盘信息**：
   - 底盘类型（`chassis.model`）
   - 底盘几何参数（`chassis.geometry`）
   - 运动限制（`chassis.limits`）

2. **根据底盘类型选择不同的可视化方式**：
   - **差速底盘**（differential）：圆形机器人 + 方向指示线
   - **阿克曼底盘**（ackermann）：矩形车辆轮廓 + 车头/后轴标记
   - **四轮底盘**（four_wheel）：矩形车辆轮廓（同阿克曼）
   - **履带底盘**（tracked）：矩形车体 + 左右履带

3. **使用实际尺寸参数**：
   - 从 `chassis.geometry` 中获取 `body_length`, `body_width`, `front_overhang`, `rear_overhang` 等
   - 如果没有 `geometry`，根据底盘类型使用合理的默认值

---

## 📝 修改的文件

### 1. `include/core/planning_context.hpp`

**添加底盘类型字段**：
```cpp
struct EgoVehicle {
  // 🔧 底盘类型
  std::string chassis_model = "differential";  // "differential", "ackermann", "tracked", "four_wheel"

  // 车辆参数
  struct Kinematics {
    double wheelbase = 2.8;       // 轴距 (m)
    double track_width = 2.0;     // 轮距 (m)
    double front_overhang = 1.0;  // 前悬 (m)
    double rear_overhang = 1.0;   // 后悬 (m)
    double width = 2.0;           // 车宽 (m)
    double height = 1.8;          // 车高 (m)
    double body_length = 4.8;     // 车体长度 (m)
    double body_width = 2.0;      // 车体宽度 (m)
    double wheel_radius = 0.3;    // 轮半径 (m)
  } kinematics;
  // ...
};
```

### 2. `src/plugin/preprocessing/basic_converter.cpp`

**解析完整的底盘信息**：
```cpp
planning::EgoVehicle BasicDataConverter::convertEgo(
    const proto::WorldTick& world_tick) {
  planning::EgoVehicle ego;
  
  // ... 位姿、速度、时间戳 ...
  
  // 🔧 车辆参数（从 world_tick 中获取底盘配置）
  if (world_tick.has_chassis()) {
    const auto& chassis = world_tick.chassis();
    
    // 底盘类型
    ego.chassis_model = chassis.model();
    
    // 基础参数
    ego.kinematics.wheelbase = chassis.wheelbase();
    ego.kinematics.track_width = chassis.track_width();
    
    // 🔧 几何参数（从 ChassisGeometry 中获取）
    if (chassis.has_geometry()) {
      const auto& geom = chassis.geometry();
      ego.kinematics.body_length = geom.body_length();
      ego.kinematics.body_width = geom.body_width();
      ego.kinematics.width = geom.body_width();
      ego.kinematics.height = geom.body_height();
      ego.kinematics.front_overhang = geom.front_overhang();
      ego.kinematics.rear_overhang = geom.rear_overhang();
      ego.kinematics.wheel_radius = geom.wheel_radius();
    } else {
      // 🔧 如果没有 geometry，根据底盘类型使用默认值
      if (ego.chassis_model == "differential") {
        // 差速底盘：小型机器人
        ego.kinematics.body_length = ego.kinematics.wheelbase * 1.5;
        ego.kinematics.body_width = ego.kinematics.track_width * 1.2;
        // ...
      } else if (ego.chassis_model == "ackermann" || ego.chassis_model == "four_wheel") {
        // 阿克曼/四轮底盘：标准汽车
        ego.kinematics.body_length = ego.kinematics.wheelbase * 1.7;
        ego.kinematics.body_width = ego.kinematics.track_width * 1.1;
        // ...
      } else if (ego.chassis_model == "tracked") {
        // 履带底盘
        ego.kinematics.body_length = ego.kinematics.wheelbase * 1.4;
        ego.kinematics.body_width = ego.kinematics.track_width;
        // ...
      }
    }
    
    // 运动限制
    if (chassis.has_limits()) {
      const auto& limits = chassis.limits();
      ego.limits.max_velocity = limits.v_max();
      ego.limits.max_acceleration = limits.a_max();
      ego.limits.max_steer_angle = limits.steer_max();
    }
  }
  
  return ego;
}
```

### 3. `src/viz/imgui_visualizer.cpp`

**根据底盘类型选择可视化方式**：

#### 差速底盘（Differential Drive）
```cpp
if (ego_.chassis_model == "differential") {
  // 🤖 圆形机器人 + 方向指示线
  double radius = ego_.kinematics.body_width / 2.0;
  
  // 绘制圆形本体
  draw_list->AddCircleFilled(..., IM_COL32(0, 200, 0, 180));
  draw_list->AddCircle(..., IM_COL32(0, 255, 0, 255));
  
  // 绘制方向指示线（从中心到边缘）
  draw_list->AddLine(..., IM_COL32(255, 255, 0, 255));
}
```

**可视化效果**：
```
    ●  ← 绿色圆形（机器人本体）
   /
  /  ← 黄色方向线
```

#### 阿克曼/四轮底盘（Ackermann / Four-Wheel）
```cpp
else if (ego_.chassis_model == "ackermann" || ego_.chassis_model == "four_wheel") {
  // 🚗 矩形车辆轮廓
  double total_length = ego_.kinematics.front_overhang + 
                       ego_.kinematics.wheelbase + 
                       ego_.kinematics.rear_overhang;
  
  // 计算四个角点
  std::vector<std::pair<double, double>> corners_local = {
    {total_length, half_width},   // 前左
    {total_length, -half_width},  // 前右
    {-ego_.kinematics.rear_overhang, -half_width},  // 后右
    {-ego_.kinematics.rear_overhang, half_width}    // 后左
  };
  
  // 绘制矩形轮廓
  draw_list->AddConvexPolyFilled(..., IM_COL32(0, 200, 0, 180));
  draw_list->AddPolyline(..., IM_COL32(0, 255, 0, 255));
  
  // 绘制车头方向指示（黄色圆点）
  draw_list->AddCircleFilled(..., IM_COL32(255, 255, 0, 255));
  
  // 绘制后轴位置（红色圆点）
  draw_list->AddCircleFilled(..., IM_COL32(255, 0, 0, 255));
}
```

**可视化效果**：
```
┌───────────────┐
│               │ ● ← 黄色圆点（车头）
│   绿色矩形    │
│               │
└───────────────┘
        ● ← 红色圆点（后轴）
```

#### 履带底盘（Tracked）
```cpp
else if (ego_.chassis_model == "tracked") {
  // 🚜 矩形车体 + 左右履带
  
  // 绘制车体（稍窄）
  draw_list->AddConvexPolyFilled(..., IM_COL32(0, 200, 0, 180));
  
  // 绘制左右履带（深灰色）
  for (int side = -1; side <= 1; side += 2) {
    draw_list->AddConvexPolyFilled(..., IM_COL32(50, 50, 50, 200));
  }
  
  // 绘制车头方向指示
  draw_list->AddCircleFilled(..., IM_COL32(255, 255, 0, 255));
}
```

**可视化效果**：
```
█┌─────────┐█  ← 深灰色履带
█│ 绿色车体 │█
█└─────────┘█
      ● ← 黄色圆点（车头）
```

---

## 🎨 底盘类型对比

| 底盘类型 | 可视化形状 | 颜色 | 特征 |
|---------|-----------|------|------|
| **differential** | 圆形 | 绿色 | 方向指示线 |
| **ackermann** | 矩形 | 绿色 | 车头黄点 + 后轴红点 |
| **four_wheel** | 矩形 | 绿色 | 车头黄点 + 后轴红点 |
| **tracked** | 矩形 + 履带 | 绿色 + 深灰 | 左右履带 + 车头黄点 |

---

## 🔄 配置同步方案

### 方案选择：**方案 B（运行时内存同步）**

**理由**：
1. ✅ **单一数据源**：WorldTick 是唯一的真实数据源
2. ✅ **实时更新**：用户在 Web 界面修改底盘配置后，立即生效
3. ✅ **无文件冲突**：不修改本地 JSON 文件，避免版本冲突
4. ✅ **简单可靠**：不需要文件 I/O，减少错误

**实现**：
- navsim-online 通过 WorldTick 发送底盘配置
- navsim-local 解析 WorldTick 并更新 `ego.chassis_model` 和 `ego.kinematics`
- 可视化代码直接使用 `ego` 中的实时数据

**本地 JSON 配置文件的作用**：
- 仅用于**离线测试**或**默认配置**
- 不影响运行时的底盘配置
- 如果需要，可以手动编辑用于测试

---

## 🧪 测试步骤

### 1. 测试差速底盘
1. 在 Web 界面选择 **差速底盘**（Differential Drive）
2. 设置参数：
   - `wheelbase = 0.5m`
   - `track_width = 0.4m`
   - `body_width = 0.5m`
3. 启动仿真
4. **预期效果**：
   - 自车显示为绿色圆形（半径 ≈ 0.25m）
   - 有黄色方向指示线

### 2. 测试阿克曼底盘
1. 在 Web 界面选择 **阿克曼底盘**（Ackermann Steering）
2. 设置参数：
   - `wheelbase = 2.8m`
   - `front_overhang = 1.0m`
   - `rear_overhang = 1.0m`
   - `body_width = 2.0m`
3. 启动仿真
4. **预期效果**：
   - 自车显示为绿色矩形（4.8m x 2.0m）
   - 车头有黄色圆点，后轴有红色圆点

### 3. 测试履带底盘
1. 在 Web 界面选择 **履带底盘**（Tracked）
2. 设置参数：
   - `wheelbase = 1.5m`
   - `body_width = 1.2m`
3. 启动仿真
4. **预期效果**：
   - 自车显示为绿色矩形车体
   - 左右两侧有深灰色履带
   - 车头有黄色圆点

### 4. 测试动态切换
1. 启动仿真（差速底盘）
2. 在 Web 界面切换到阿克曼底盘
3. **预期效果**：
   - 自车可视化立即从圆形变为矩形
   - 尺寸参数立即更新

---

## 📊 默认尺寸参数

### 差速底盘（Differential Drive）
```
wheelbase: 0.5m
track_width: 0.4m
body_length: 0.75m (wheelbase * 1.5)
body_width: 0.48m (track_width * 1.2)
front_overhang: 0.125m (wheelbase * 0.25)
rear_overhang: 0.125m (wheelbase * 0.25)
wheel_radius: 0.1m
```

### 阿克曼底盘（Ackermann Steering）
```
wheelbase: 2.8m
track_width: 1.8m
body_length: 4.76m (wheelbase * 1.7)
body_width: 1.98m (track_width * 1.1)
front_overhang: 0.98m (wheelbase * 0.35)
rear_overhang: 0.98m (wheelbase * 0.35)
wheel_radius: 0.3m
```

### 履带底盘（Tracked）
```
wheelbase: 1.5m
track_width: 1.2m
body_length: 2.1m (wheelbase * 1.4)
body_width: 1.2m (track_width * 1.0)
front_overhang: 0.3m (wheelbase * 0.2)
rear_overhang: 0.3m (wheelbase * 0.2)
wheel_radius: 0.15m
```

---

## 🎯 预期效果

### 修复前
```
所有底盘类型都显示为相同的矩形 ❌
尺寸固定为 4.8m x 2.0m ❌
无法区分底盘类型 ❌
```

### 修复后
```
差速底盘 → 圆形机器人（0.5m 直径）✅
阿克曼底盘 → 矩形汽车（4.8m x 2.0m）✅
履带底盘 → 矩形 + 履带（2.1m x 1.2m）✅
尺寸根据 WorldTick 实时更新 ✅
```

---

**修复完成时间**：2025-10-14  
**编译状态**：✅ 成功  
**测试状态**：⏳ 待用户验证

**修复内容总结**：
1. ✅ 添加底盘类型字段（`chassis_model`）
2. ✅ 解析完整的底盘几何参数（`ChassisGeometry`）
3. ✅ 根据底盘类型选择不同的可视化方式
4. ✅ 使用 WorldTick 中的实时尺寸参数
5. ✅ 支持四种底盘类型（differential, ackermann, tracked, four_wheel）

