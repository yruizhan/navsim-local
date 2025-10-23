# NavSim Local - 快速开始指南

本指南帮助您快速开始使用 NavSim 本地仿真模式。

---

## 📋 前置要求

### 系统要求
- Ubuntu 20.04+ 或其他 Linux 发行版
- CMake 3.15+
- GCC 9+ 或 Clang 10+
- Python 3.8+ (仅用于 WebSocket 可视化)

### 依赖库
- Protobuf 3.0+
- SDL2 (用于本地可视化)
- GoogleTest (用于单元测试)
- ixwebsocket (已包含在 third_party)

---

## 🚀 快速开始

**重要说明**：
- ✅ **仿真都在本地运行**（navsim_algo 进程内的 LocalSimulator）
- ✅ **算法都在本地运行**（navsim_algo 进程内的 AlgorithmManager）
- ✅ **区别仅在于场景来源**：本地 JSON 文件 vs 前端网页构建

### 1. 编译项目

```bash
cd navsim-local
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 2. 运行本地仿真模式

```bash
cd navsim-local
./build/navsim_algo --local-sim \
  --scenario=scenarios/map1.json \
  --config=config/default.json
```

**特点**：
- ✅ 场景来源：本地 JSON 文件
- ✅ 仿真位置：navsim_algo 进程内
- ✅ 零网络延迟
- ✅ 30Hz 稳定运行
- ✅ 无外部依赖
- ✅ 适合批量测试

### 3. 运行 WebSocket 在线模式（未实现）

#### 步骤 1：安装 Python 依赖

```bash
cd navsim-online/server
pip3 install -r requirements.txt
```

#### 步骤 2：启动 navsim-online 服务器（仅转发消息）

```bash
cd navsim-online/server
python3 main.py
```

服务器将在 `http://localhost:8080` 启动。

#### 步骤 3：运行 navsim_algo（WebSocket 模式）

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

#### 步骤 4：打开浏览器构建场景

在浏览器中访问：`http://localhost:8080`

在网页上：
1. 绘制障碍物
2. 设置起点和终点
3. 点击"开始"按钮

**特点**：
- ✅ 场景来源：前端网页构建
- ✅ 仿真位置：navsim_algo 进程内
- ✅ 前端可视化
- ✅ 便于在线演示和远程调试
- ✅ 用户可以交互式构建场景

---

## 📝 命令行参数

### 基本参数

| 参数 | 说明 | 示例 |
|------|------|------|
| `--local-sim` | 启用本地仿真模式 | `--local-sim` |
| `--scenario=<path>` | 指定场景文件路径 | `--scenario=scenarios/map1.json` |
| `--config=<path>` | 指定配置文件路径 | `--config=config/default.json` |

### WebSocket 可视化参数（可选）

| 参数 | 说明 | 示例 |
|------|------|------|
| `--ws-url=<url>` | WebSocket 服务器地址 | `--ws-url=ws://127.0.0.1:8080/ws` |
| `--room-id=<id>` | 房间ID | `--room-id=demo` |

---

## 📁 场景文件

### 场景文件格式

场景文件使用 JSON 格式，支持两种格式：

1. **Online 格式**（推荐）：与 navsim-online 兼容
2. **Local 格式**：简化的本地格式

系统会自动检测并转换格式。

### 示例场景

```json
{
  "ego": {
    "pose": {"x": 0, "y": 0, "yaw": 0},
    "twist": {"vx": 0, "vy": 0, "omega": 0}
  },
  "goal": {
    "x": 6,
    "y": 6,
    "yaw": 0
  },
  "static_obstacles": [
    {
      "type": "circle",
      "center": {"x": 3, "y": 3},
      "radius": 0.5
    }
  ],
  "dynamic_obstacles": [
    {
      "id": 1,
      "pose": {"x": 5, "y": 5, "yaw": 0},
      "twist": {"vx": 0.5, "vy": 0, "omega": 0},
      "shape": "circle",
      "radius": 0.3
    }
  ]
}
```

### 可用场景

- `scenarios/map1.json`：包含 6 个静态障碍物和 12 个动态障碍物的测试场景

---

## ⚙️ 配置文件

### 配置文件格式

配置文件使用 JSON 格式，包含感知和规划模块的配置。

### 示例配置

```json
{
  "perception": {
    "plugins": [
      {
        "name": "GridMapBuilder",
        "enabled": true,
        "priority": 100,
        "config": {
          "resolution": 0.1,
          "map_size": 30,
          "inflation_radius": 0.0
        }
      },
      {
        "name": "EsdfBuilder",
        "enabled": true,
        "priority": 90,
        "config": {
          "resolution": 0.1,
          "map_width": 19,
          "map_height": 19,
          "max_distance": 5.0,
          "include_dynamic": true
        }
      }
    ]
  },
  "planning": {
    "primary_planner": "JpsPlanner",
    "fallback_planner": "StraightLinePlanner",
    "planners": [
      {
        "name": "JpsPlanner",
        "config": {
          "safe_distance": 0.3,
          "max_velocity": 1.5,
          "max_acceleration": 1.0,
          "max_omega": 1.0
        }
      }
    ]
  }
}
```

### 可用配置

- `config/default.json`：默认配置文件

---

## 🔧 故障排除

### 问题 1：编译失败

**症状**：CMake 或 make 报错

**解决方案**：
1. 检查依赖库是否安装：`sudo apt install libprotobuf-dev libsdl2-dev`
2. 检查 CMake 版本：`cmake --version`（需要 3.15+）
3. 清理 build 目录：`rm -rf build && mkdir build`

### 问题 2：规划失败

**症状**：日志显示 "Cannot find a path"

**解决方案**：
1. 检查目标点是否在 ESDF 地图范围内
2. 调整 `config/default.json` 中的 `map_width` 和 `map_height`
3. 检查场景文件中的障碍物是否阻挡了路径

### 问题 3：车辆不移动

**症状**：规划成功但车辆位置不变

**解决方案**：
1. 检查轨迹跟踪控制器是否正常工作
2. 查看日志中的 "Ego pose" 是否变化
3. 确认规划器生成的轨迹点数 > 0

### 问题 4：WebSocket 连接失败

**症状**：日志显示 "Failed to connect to WebSocket server"

**解决方案**：
1. 检查 navsim-online 服务器是否启动：`curl http://localhost:8080`
2. 检查 WebSocket URL 是否正确：`--ws-url=ws://127.0.0.1:8080/ws`
3. 检查防火墙设置

**注意**：WebSocket 连接失败时，本地仿真会继续运行（优雅降级）。

---

## 📊 性能优化建议

### 1. 调整仿真频率

修改 `navsim_algo.cpp` 中的 `dt` 参数：

```cpp
const double dt = 0.0333;  // 30Hz (默认)
// const double dt = 0.01;  // 100Hz (高精度)
// const double dt = 0.1;   // 10Hz (低性能设备)
```

### 2. 调整 ESDF 地图尺寸

修改 `config/default.json` 中的 ESDF 配置：

```json
{
  "name": "EsdfBuilder",
  "config": {
    "resolution": 0.1,      // 分辨率（米/格）
    "map_width": 19,        // 地图宽度（米）
    "map_height": 19,       // 地图高度（米）
    "max_distance": 5.0     // 最大距离（米）
  }
}
```

**建议**：
- 小场景：19x19m
- 中场景：50x50m
- 大场景：100x100m

### 3. 调整规划器参数

修改 `config/default.json` 中的规划器配置：

```json
{
  "name": "JpsPlanner",
  "config": {
    "safe_distance": 0.3,      // 安全距离（米）
    "max_velocity": 1.5,       // 最大速度（米/秒）
    "max_acceleration": 1.0,   // 最大加速度（米/秒²）
    "max_omega": 1.0           // 最大角速度（弧度/秒）
  }
}
```

---

## 📚 更多文档

- **架构文档**：`docs/architecture-refactor.md`
- **测试报告**：`docs/phase5-testing-report.md`
- **最终总结**：`docs/FINAL_SUMMARY.md`

---

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

## 📄 许可证

本项目遵循原 NavSim 项目的许可证。

---

**祝您使用愉快！** 🎉

