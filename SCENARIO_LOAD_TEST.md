# 场景加载功能测试指南

## 🎯 最新更新（已修复路径问题）

**✅ 修复内容：**
1. **所有场景加载日志都会显示在 UI 的按钮日志区域**
2. **修复了场景文件路径问题**：现在使用 `scenarios/` 而不是 `../scenarios/`

**现在可以直接在 UI 中输入 `map2.json`，程序会自动在 `scenarios/` 目录下查找！**

## 问题描述

用户报告：在 navsim-local 项目中，使用 UI 加载新场景后，**无论是否点击开始，地图都没有切换**。

## 已添加的调试日志

为了诊断问题，我在以下位置添加了详细的调试日志：

### 1. `LocalSimulator::load_scenario()` (local_simulator.cpp)

加载场景时会输出：
```
[LocalSimulator] ========================================
[LocalSimulator] Loading scenario: scenarios/map2.json
[LocalSimulator] Scenario loaded into context
[LocalSimulator] Context has bev_obstacles: YES
[LocalSimulator] BEV circles: 1
[LocalSimulator] BEV polygons: 2
[LocalSimulator] Converting BEV obstacles to static obstacles...
[LocalSimulator] Converted static obstacles: 3
[LocalSimulator] Map version updated to: 3
[LocalSimulator] ========================================
[LocalSimulator] Loaded scenario: scenarios/map2.json
  Ego: (-1.33622, 2.78482, 0.241869)
  Goal: (6.25961, 7.87578, 0)
  Dynamic obstacles: 3
  Static obstacles: 3
    [0] Circle at (2.36804, 9.31695), r=0.3
    [1] Polygon with 4 vertices
    [2] Polygon with 4 vertices
[LocalSimulator] ========================================
```

**关键信息**：
- `BEV circles: 1` - 应该显示新场景的圆形障碍物数量
- `BEV polygons: 2` - 应该显示新场景的多边形障碍物数量
- `Static obstacles: 3` - 应该显示转换后的静态障碍物总数

### 2. `LocalSimulator::to_world_tick()` (local_simulator.cpp)

每 30 帧（约 1 秒）输出一次：
```
[LocalSimulator::to_world_tick] tick_id=30, map_version=3, circles=1, polygons=2
```

**关键信息**：
- `map_version` - 地图版本号，每次加载新场景时递增
- `circles` 和 `polygons` - 应该显示当前场景的障碍物数量

### 3. `BEVExtractor::extract()` (bev_extractor.cpp)

每次提取时输出：
```
[BEVExtractor] ========== Extract called ==========
[BEVExtractor] WorldTick tick_id: 30
[BEVExtractor] Has static_map: 1
[BEVExtractor] Dynamic obstacles count: 3
[BEVExtractor] StaticMap circles: 1
[BEVExtractor] StaticMap polygons: 2
[BEVExtractor] Updated static map cache
[BEVExtractor] Processing static obstacles...
[BEVExtractor]   Ego position: (-1.33622, 2.78482)
[BEVExtractor]   Detection range: 50 m
[BEVExtractor]   Cached circles: 1
[BEVExtractor]   Cached polygons: 2
[BEVExtractor]   Static circles in range: 1
[BEVExtractor]   Static polygons in range: 2
[BEVExtractor] ========== Extract result ==========
[BEVExtractor] Extracted circles: 1
[BEVExtractor] Extracted rectangles: 0
[BEVExtractor] Extracted polygons: 2
[BEVExtractor] ======================================
```

**关键信息**：
- `StaticMap circles` 和 `StaticMap polygons` - 从 WorldTick 接收到的静态地图数据
- `Extracted circles` 和 `Extracted polygons` - 实际提取的障碍物数量（在检测范围内）

## 测试步骤

### 1. 启动程序

```bash
cd navsim-local
./build/navsim_algo --local-sim --scenario=scenarios/map1.json --visualize
```

### 2. 在 UI 中加载新场景

1. 在 UI 右侧的 "Load Scenario" 输入框中输入：`map2.json`
2. 点击 "Load" 按钮
3. **观察 "Button Logs" 区域**（在 Start/Pause/Reset 按钮下方）

### 3. 观察 UI 日志显示

在 "Button Logs" 区域，您应该看到类似以下的日志：

```
HH:MM:SS.mmm - 🔄 Loading scenario: scenarios/map2.json
HH:MM:SS.mmm - ✅ File found, loading...
HH:MM:SS.mmm - 🔄 Resetting system...
HH:MM:SS.mmm - 🔄 Loading into simulator...
HH:MM:SS.mmm - 📊 Obstacles: 1 circles, 2 polygons
HH:MM:SS.mmm - 📍 Dynamic obstacles: 3
HH:MM:SS.mmm - ✅ Converted 3 static obstacles
HH:MM:SS.mmm - 🗺️  Map version: 3
HH:MM:SS.mmm - ✅ Scenario loaded successfully!
HH:MM:SS.mmm - ℹ️  Click START to begin simulation
```

### 4. 观察可视化更新

**期望看到的日志**：

```
[AlgorithmManager] ========================================
[AlgorithmManager] loadScenario() called!
[AlgorithmManager] Loading scenario: scenarios/map2.json
[AlgorithmManager] Checking if file exists...
[AlgorithmManager] File exists, proceeding...
[AlgorithmManager] Performing full system reset...
[AlgorithmManager] Resetting all plugins...
[AlgorithmManager] All plugins reset successfully
[AlgorithmManager] Resetting LocalSimulator...
[LocalSimulator] Reset to initial state
[AlgorithmManager] Clearing visualizer cache...
[AlgorithmManager] Full system reset complete
[AlgorithmManager] Loading scenario into simulator...
[LocalSimulator] ========================================
[LocalSimulator] Loading scenario: scenarios/map2.json
[ScenarioLoader] Detected online format, converting...
[ScenarioLoader] Successfully loaded scenario
[LocalSimulator] Scenario loaded into context
[LocalSimulator] Context has bev_obstacles: YES
[LocalSimulator] BEV circles: 1          ← 应该是 1（map2 只有 1 个圆形）
[LocalSimulator] BEV polygons: 2         ← 应该是 2（map2 有 2 个多边形）
[LocalSimulator] Converting BEV obstacles to static obstacles...
[LocalSimulator] Converted static obstacles: 3
[LocalSimulator] Map version updated to: 3
[LocalSimulator] ========================================
[LocalSimulator] Loaded scenario: scenarios/map2.json
  Ego: (-1.33622, 2.78482, 0.241869)
  Goal: (6.25961, 7.87578, 0)
  Dynamic obstacles: 3
  Static obstacles: 3
    [0] Circle at (2.36804, 9.31695), r=0.3
    [1] Polygon with 4 vertices
    [2] Polygon with 4 vertices
[LocalSimulator] ========================================
[AlgorithmManager] Scenario loaded successfully: scenarios/map2.json
[AlgorithmManager] Simulation paused, click START to begin
```

### 5. 观察可视化更新

**期望行为**：
- 加载完成后，**即使在暂停状态**，可视化器应该立即显示新场景的地图
- 应该看到 1 个圆形障碍物和 2 个多边形障碍物
- Ego 位置应该移动到 map2 的起始位置：(-1.34, 2.78)
- Goal 位置应该移动到 map2 的目标位置：(6.26, 7.88)

### 6. 观察 BEVExtractor 日志

加载新场景后，下一次 BEVExtractor 调用应该显示：
```
[BEVExtractor] StaticMap circles: 1     ← 应该是 1（不是 6）
[BEVExtractor] StaticMap polygons: 2    ← 应该是 2（不是 4）
[BEVExtractor] Extracted circles: 1
[BEVExtractor] Extracted polygons: 2
```

### 7. 观察 to_world_tick 日志

每秒应该看到：
```
[LocalSimulator::to_world_tick] tick_id=30, map_version=3, circles=1, polygons=2
```

## 场景数据对比

### map1.json
- 起始位置：(0, 0, 0)
- 目标位置：(6, 6, 0)
- 静态障碍物：**6 个圆形 + 4 个多边形 = 10 个**
- 动态障碍物：12 个

### map2.json
- 起始位置：(-1.34, 2.78, 0.24)
- 目标位置：(6.26, 7.88, 0)
- 静态障碍物：**1 个圆形 + 2 个多边形 = 3 个**
- 动态障碍物：3 个

## 诊断问题

如果地图没有切换，请检查以下日志：

### 问题 1：`LocalSimulator::load_scenario()` 没有被调用

**症状**：点击 Load 按钮后，没有看到 `[LocalSimulator] Loading scenario:` 日志

**可能原因**：
- UI 的 Load 按钮回调没有正确设置
- `AlgorithmManager::loadScenario()` 没有被调用

### 问题 2：场景加载了，但 BEV 障碍物数量不对

**症状**：看到 `[LocalSimulator] BEV circles: 6` 而不是 `1`

**可能原因**：
- `ScenarioLoader::loadFromFile()` 没有正确解析 JSON 文件
- JSON 格式转换有问题

### 问题 3：BEVExtractor 仍然显示旧的障碍物数量

**症状**：看到 `[BEVExtractor] StaticMap circles: 6` 而不是 `1`

**可能原因**：
- `LocalSimulator::to_world_tick()` 没有返回新的静态地图数据
- `world_state_.static_obstacles` 没有被正确更新

### 问题 4：可视化器没有更新

**症状**：日志显示正确的障碍物数量，但可视化器仍然显示旧地图

**可能原因**：
- 可视化器的缓存没有被清除
- `drawBEVObstacles()` 没有被调用
- 暂停状态下的可视化更新逻辑有问题

## 预期结果

如果修复成功，应该看到：

1. ✅ 点击 Load 按钮后，立即看到场景加载日志
2. ✅ `LocalSimulator` 日志显示正确的障碍物数量（1 个圆形 + 2 个多边形）
3. ✅ `BEVExtractor` 日志显示正确的障碍物数量
4. ✅ 可视化器立即显示新场景的地图（即使在暂停状态）
5. ✅ 点击 Start 按钮后，仿真在新场景中正常运行

## 下一步

请运行程序并按照上述步骤测试，然后将完整的日志输出发送给我，特别是：
1. 点击 Load 按钮后的所有日志
2. BEVExtractor 的日志
3. to_world_tick 的日志
4. 可视化器是否显示了新地图

这样我就能准确诊断问题所在。

