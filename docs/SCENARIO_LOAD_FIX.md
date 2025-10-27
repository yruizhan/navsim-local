# 场景加载功能修复

## 问题描述

在 navsim-local 项目中，`load scenario` 功能存在问题：
- 用户在 UI 中加载新场景（例如从 map1.json 切换到 map2.json）
- 加载成功后，点击 Start 按钮
- **问题**：地图没有切换，仍然显示旧场景的障碍物

## 根本原因

问题有两个部分：

### 问题 1：`loadScenario()` 停止了仿真循环

原始的 `loadScenario()` 代码：

```cpp
// 2. 停止当前仿真循环（如果正在运行）
bool was_running = !simulation_should_stop_.load();
if (was_running) {
  std::cout << "[AlgorithmManager] Stopping current simulation..." << std::endl;
  stop_simulation_loop();  // 设置 simulation_should_stop_ = true
  // 等待仿真循环停止
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// ... 加载场景 ...

// 6. 重新开始仿真（如果之前在运行）
if (was_running) {
  std::cout << "[AlgorithmManager] Restarting simulation with new scenario..." << std::endl;
  simulation_should_stop_.store(false);  // 只有在 was_running 为 true 时才重置
  // 注意：这里不调用 run_simulation_loop()，因为它会阻塞
  // 仿真循环会在下一次迭代时自动继续
}
```

**问题**：
- `stop_simulation_loop()` 设置 `simulation_should_stop_ = true`，导致仿真循环退出
- 只有在 `was_running` 为 true 时才重置 `simulation_should_stop_ = false`
- 但是仿真循环已经退出了，不会再继续运行
- 结果：加载场景后，仿真循环停止，地图不再更新

### 问题 2：暂停状态下不更新可视化数据

原始的暂停状态处理代码：

```cpp
// 🎮 检查仿真是否暂停
if (simulation_paused_.load()) {
  // 暂停时仍然渲染可视化界面，但不执行仿真步进
  if (visualizer_) {
    visualizer_->beginFrame();
    visualizer_->showDebugInfo("Simulation Status", "PAUSED");
    visualizer_->endFrame();
  }

  // 短暂休眠避免CPU占用过高
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  continue;  // 跳过本次循环，不执行仿真步进
}
```

**问题**：
- 在暂停状态下，仿真循环只渲染简单的 "PAUSED" 信息
- **不会更新可视化器的世界数据**（ego、goal、obstacles 等）
- 即使加载了新场景，用户在暂停状态下也看不到新地图

## 解决方案

### 修复 1：`loadScenario()` 不再停止仿真循环

修改后的代码：

```cpp
bool AlgorithmManager::loadScenario(const std::string& scenario_file) {
  // ... 检查文件是否存在 ...

  // 2. 🔧 不要停止仿真循环，只是暂停仿真
  // 保存当前状态
  bool was_paused = simulation_paused_.load();

  // 暂停仿真（但不停止循环）
  simulation_paused_.store(true);

  // 等待当前帧完成
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 3. 执行完整的系统重置
  performFullReset();

  // 4. 重新加载场景到仿真器
  if (local_simulator_) {
    std::cout << "[AlgorithmManager] Loading scenario into simulator..." << std::endl;
    if (!local_simulator_->load_scenario(scenario_file)) {
      std::cerr << "[AlgorithmManager] Failed to load scenario into simulator" << std::endl;
      return false;
    }
  } else {
    std::cerr << "[AlgorithmManager] No local simulator available" << std::endl;
    return false;
  }

  // 5. 保存当前场景文件路径
  current_scenario_file_ = scenario_file;

  // 6. 加载新场景后默认暂停，等待用户点击 Start
  simulation_paused_.store(true);

  std::cout << "[AlgorithmManager] Scenario loaded successfully: " << scenario_file << std::endl;
  std::cout << "[AlgorithmManager] Simulation paused, click START to begin" << std::endl;
  return true;
}
```

**关键改进**：
- 不再调用 `stop_simulation_loop()`，避免停止仿真循环
- 只是设置 `simulation_paused_ = true`，让仿真循环继续运行但处于暂停状态
- 仿真循环会继续更新可视化数据（见修复 2）

### 修复 2：暂停状态下也更新可视化数据

修改后的代码：

```cpp
// 🎮 检查仿真是否暂停
if (simulation_paused_.load()) {
  // 暂停时仍然渲染可视化界面，并显示当前世界状态
  if (visualizer_) {
    visualizer_->beginFrame();
    
    // 🔧 即使暂停，也要显示当前世界状态（特别是加载新场景后）
    const auto& world_state = local_simulator_->get_world_state();
    
    // 更新仿真时间
    double sim_time = local_simulator_->get_simulation_time();
    std::ostringstream time_stream;
    time_stream << std::fixed << std::setprecision(3) << sim_time << "s";
    visualizer_->showDebugInfo("Simulation Time", time_stream.str());
    visualizer_->showDebugInfo("Frame ID", std::to_string(local_simulator_->get_frame_id()));
    
    // 转换为protobuf格式并更新可视化
    auto world_tick = local_simulator_->to_world_tick();
    
    // 创建前置处理管线并处理
    perception::PreprocessingPipeline preprocessing_pipeline;
    plugin::PerceptionInput perception_input = preprocessing_pipeline.process(world_tick);
    
    // 更新可视化器的世界数据
    visualizer_->drawEgo(perception_input.ego);
    visualizer_->drawGoal(perception_input.task.goal_pose);
    visualizer_->drawBEVObstacles(perception_input.bev_obstacles);
    visualizer_->drawDynamicObstacles(perception_input.dynamic_obstacles);
    
    // 如果有感知插件，也处理一下以获取栅格地图
    planning::PlanningContext context;
    context.ego = perception_input.ego;
    context.task = perception_input.task;
    context.dynamic_obstacles = perception_input.dynamic_obstacles;
    
    if (perception_plugin_manager_->process(perception_input, context)) {
      visualizer_->updatePlanningContext(context);
      if (context.occupancy_grid) {
        visualizer_->drawOccupancyGrid(*context.occupancy_grid);
      }
    }
    
    visualizer_->showDebugInfo("Simulation Status", "PAUSED");
    visualizer_->endFrame();
  }

  // 短暂休眠避免CPU占用过高
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  continue;  // 跳过本次循环，不执行仿真步进
}
```

## 修复效果

修复后的行为：

1. 用户在 UI 中输入新场景文件名（例如 `map2.json`）并点击 Load 按钮
2. `AlgorithmManager::loadScenario()` 被调用：
   - 停止仿真
   - 执行完整的系统重置
   - 加载新场景到 `LocalSimulator`
   - 设置 `simulation_paused_ = true`
3. **关键改进**：即使在暂停状态，仿真循环也会：
   - 从 `LocalSimulator` 获取最新的世界状态
   - 更新可视化器的所有数据（ego、goal、obstacles、grid map 等）
   - 用户**立即**看到新场景的地图
4. 用户点击 Start 按钮：
   - `startSimulation()` 被调用，设置 `simulation_paused_ = false`
   - 仿真开始运行，继续显示新场景的地图

## 测试方法

运行测试脚本：

```bash
cd navsim-local
./test_scenario_load.sh
```

测试步骤：

1. 程序启动后会加载 `map1.json`
2. 在 UI 中输入 `map2.json` 并点击 Load 按钮
3. **验证**：地图应该立即切换到 map2（即使在暂停状态）
4. 点击 Start 按钮
5. **验证**：地图应该保持为 map2，仿真开始运行

预期结果：
- 加载 map2 后，即使在暂停状态也应该显示 map2 的障碍物
- 点击 Start 后，应该继续显示 map2 的障碍物，并开始仿真

## 相关文件

- `navsim-local/platform/src/core/algorithm_manager.cpp`：主要修复文件
  - `run_simulation_loop()` 函数（第 860-909 行）
  - `loadScenario()` 函数（第 456-512 行）
  - `startSimulation()` 函数（第 514-530 行）

## 注意事项

1. 这个修复只影响本地仿真模式（`--local-sim`），不影响 WebSocket 在线模式
2. 暂停状态下仍然会执行感知插件处理（用于生成栅格地图），但不会执行规划器和仿真步进
3. 暂停状态下的渲染频率较低（每 10ms 一次），以避免 CPU 占用过高

## 版本信息

- 修复日期：2025-10-27
- 修复版本：navsim-local v1.0
- 相关 Issue：场景加载后地图不切换

