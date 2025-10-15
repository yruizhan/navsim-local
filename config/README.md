# NavSim-Local 配置文件

本目录包含 navsim-local 的配置文件。

## 📁 目录结构

```
config/
├── README.md       # 本文档
└── default.json    # 默认配置（带可视化）
```

## 🚀 快速开始

### 1. 使用默认配置

默认配置已启用可视化和 A* 规划器：

```bash
./build_with_visualization.sh
```

脚本会自动编译并运行，无需手动指定配置文件。

### 2. 修改配置

直接编辑 `config/default.json`：

```json
{
  "algorithm": {
    "primary_planner": "AStarPlanner",
    "enable_visualization": true
  },
  "perception": {
    "plugins": [
      {
        "name": "GridMapBuilder",
        "params": {
          "map_width": 30.0,
          "map_height": 30.0,
          "resolution": 0.1
        }
      }
    ]
  }
}
```

### 3. 手动运行

如果需要手动运行：

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/default.json
```

## 📋 配置文件结构

### 算法配置

```json
{
  "algorithm": {
    "primary_planner": "AStarPlanner",        // 主规划器
    "fallback_planner": "StraightLinePlanner", // 降级规划器
    "enable_planner_fallback": true,          // 启用降级
    "max_computation_time_ms": 25.0,          // 最大计算时间
    "verbose_logging": true,                  // 详细日志
    "enable_visualization": true              // 启用可视化
  }
}
```

### 感知配置

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {
        "enabled": true,
        "static_obstacle_inflation": 0.2,
        "dynamic_obstacle_inflation": 0.3
      },
      "dynamic_prediction": {
        "enabled": true,
        "prediction_horizon": 5.0,
        "time_step": 0.1
      }
    },
    "plugins": [
      {
        "name": "GridMapBuilder",
        "enabled": true,
        "priority": 100,
        "params": {
          "resolution": 0.1,
          "map_width": 30.0,
          "map_height": 30.0,
          "obstacle_cost": 100,
          "inflation_radius": 0.0
        }
      }
    ]
  }
}
```

### 规划器配置

```json
{
  "planning": {
    "StraightLinePlanner": {
      "default_velocity": 1.5,
      "time_step": 0.1,
      "planning_horizon": 5.0,
      "use_trapezoidal_profile": true,
      "max_acceleration": 1.0
    },
    "AStarPlanner": {
      "time_step": 0.1,
      "heuristic_weight": 1.2,
      "step_size": 0.5,
      "max_iterations": 10000,
      "goal_tolerance": 0.5,
      "default_velocity": 1.5
    }
  }
}
```

## ⚙️ 关键参数说明

### 栅格地图参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `resolution` | double | 0.1 | 栅格分辨率 (m/cell) |
| `map_width` | double | 30.0 | 地图宽度 (m) |
| `map_height` | double | 30.0 | 地图高度 (m) |
| `inflation_radius` | double | 0.0 | 膨胀半径 (m) |
| `obstacle_cost` | int | 100 | 障碍物代价值 |

### A* 规划器参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `time_step` | double | 0.1 | 时间步长 (s) |
| `heuristic_weight` | double | 1.2 | 启发式权重 |
| `step_size` | double | 0.5 | 搜索步长 (m) |
| `max_iterations` | int | 10000 | 最大迭代次数 |
| `goal_tolerance` | double | 0.5 | 目标容差 (m) |
| `default_velocity` | double | 1.5 | 默认速度 (m/s) |

## ⌨️ 可视化控制

| 按键 | 功能 |
|------|------|
| `F` | 切换跟随自车模式 |
| `+` | 放大视图 |
| `-` | 缩小视图 |
| `ESC` | 关闭窗口 |

**Legend 面板**：
- 勾选/取消勾选可视化元素
- 点击 "Fit Occupancy Grid" 自动适应栅格地图

## 💡 提示

1. **快速开始**: 直接运行 `./build_with_visualization.sh`
2. **调整地图大小**: 修改 `map_width` 和 `map_height`（建议 10-50m）
3. **启用日志**: `verbose_logging: true` 查看详细信息
4. **切换规划器**: 修改 `primary_planner` 为 `StraightLinePlanner` 或 `AStarPlanner`

---

**最后更新**: 2025-10-15
