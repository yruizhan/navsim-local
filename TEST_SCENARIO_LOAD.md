# 场景加载功能测试指南

## 问题描述

**原始问题**：加载新场景后，无论是否点击开始，地图都没有切换。

## 测试步骤

### 1. 启动程序

```bash
cd navsim-local
./build/navsim_algo --local-sim --scenario=scenarios/map1.json --visualize
```

程序启动后会显示 map1 的场景：
- 起点：(0, 0)
- 终点：(6, 6)
- 静态障碍物：6 个圆形 + 4 个多边形
- 动态障碍物：12 个

### 2. 加载新场景

在 UI 界面中：
1. 找到 "Load Scenario" 输入框
2. 输入：`scenarios/map2.json`
3. 点击 "Load" 按钮（或按 Enter 键）

**预期结果**：
- ✅ 控制台输出：`[AlgorithmManager] Scenario loaded successfully: scenarios/map2.json`
- ✅ 地图**立即**切换到 map2（即使在暂停状态）
- ✅ 显示 map2 的场景：
  - 起点：(-1.34, 2.78)
  - 终点：(6.26, 7.88)
  - 静态障碍物：1 个圆形 + 2 个多边形
  - 动态障碍物：3 个
- ✅ 状态显示为 "PAUSED"

### 3. 点击 Start 按钮

在 UI 界面中：
1. 点击 "Start" 按钮

**预期结果**：
- ✅ 地图保持为 map2（不会切换回 map1）
- ✅ 仿真开始运行
- ✅ 自车开始移动
- ✅ 动态障碍物开始移动

### 4. 再次加载场景

在仿真运行过程中：
1. 在 "Load Scenario" 输入框中输入：`scenarios/map1.json`
2. 点击 "Load" 按钮

**预期结果**：
- ✅ 地图切换回 map1
- ✅ 仿真自动暂停
- ✅ 自车位置重置到 map1 的起点 (0, 0)
- ✅ 显示 map1 的障碍物

### 5. 测试相对路径

尝试使用相对路径加载场景：
1. 输入：`map2.json`（不带 `scenarios/` 前缀）
2. 点击 "Load" 按钮

**预期结果**：
- ❌ 控制台输出错误：`[AlgorithmManager] ERROR: Scenario file not found: map2.json`
- ℹ️ 需要使用完整路径：`scenarios/map2.json`

## 验证要点

### ✅ 修复前的问题

- ❌ 加载场景后，地图不切换
- ❌ 仿真循环停止，界面冻结
- ❌ 需要重启程序才能看到新场景

### ✅ 修复后的行为

- ✅ 加载场景后，地图**立即**切换（即使在暂停状态）
- ✅ 仿真循环继续运行，界面正常更新
- ✅ 点击 Start 后，仿真正常运行
- ✅ 可以多次加载不同场景，无需重启程序

## 控制台输出示例

### 成功加载场景

```
[AlgorithmManager] ==========================================
[AlgorithmManager] loadScenario() called!
[AlgorithmManager] Loading scenario: scenarios/map2.json
[AlgorithmManager] Checking if file exists...
[AlgorithmManager] File exists, proceeding...
[AlgorithmManager] Loading scenario into simulator...
[LocalSimulator] Loaded scenario: scenarios/map2.json
  Ego: (-1.33622, 2.78482, 0.241869)
  Goal: (6.25961, 7.87578, 0)
  Dynamic obstacles: 3
  Static obstacles: 3
[AlgorithmManager] Scenario loaded successfully: scenarios/map2.json
[AlgorithmManager] Simulation paused, click START to begin
```

### 文件不存在

```
[AlgorithmManager] ==========================================
[AlgorithmManager] loadScenario() called!
[AlgorithmManager] Loading scenario: map2.json
[AlgorithmManager] Checking if file exists...
[AlgorithmManager] ERROR: Scenario file not found: map2.json
[AlgorithmManager] Please check the file path and try again.
```

## 已知限制

1. **路径必须相对于可执行文件**：
   - 正确：`scenarios/map2.json`
   - 错误：`map2.json`

2. **场景文件必须存在**：
   - 如果文件不存在，会显示错误信息
   - 不会影响当前场景

3. **加载场景会重置仿真状态**：
   - 自车位置重置到新场景的起点
   - 仿真时间重置为 0
   - 所有插件状态重置

## 相关文件

- `navsim-local/platform/src/core/algorithm_manager.cpp`
  - `loadScenario()` 函数（第 456-506 行）
  - `run_simulation_loop()` 函数（第 860-909 行）
- `navsim-local/scenarios/map1.json`：测试场景 1
- `navsim-local/scenarios/map2.json`：测试场景 2
- `navsim-local/scenarios/map3.json`：测试场景 3

## 故障排除

### 问题：加载场景后地图没有切换

**可能原因**：
1. 文件路径错误（检查控制台错误信息）
2. 场景文件格式错误（检查 JSON 格式）
3. 可视化窗口没有刷新（尝试移动窗口或调整大小）

**解决方法**：
1. 检查控制台输出，确认场景加载成功
2. 确保使用正确的路径：`scenarios/map2.json`
3. 检查场景文件是否存在且格式正确

### 问题：点击 Start 后仿真没有运行

**可能原因**：
1. 场景加载失败
2. 插件初始化失败

**解决方法**：
1. 检查控制台输出，查看错误信息
2. 尝试重新加载场景
3. 重启程序

## 测试清单

- [ ] 启动程序，显示 map1
- [ ] 加载 map2，地图立即切换
- [ ] 点击 Start，仿真正常运行
- [ ] 在运行中加载 map1，地图切换并暂停
- [ ] 再次点击 Start，仿真继续运行
- [ ] 尝试加载不存在的文件，显示错误信息
- [ ] 尝试加载 map3，地图正常切换
- [ ] 多次切换场景，程序稳定运行

## 版本信息

- 修复日期：2025-10-27
- 测试版本：navsim-local v1.0
- 相关文档：`docs/SCENARIO_LOAD_FIX.md`

