# NavSim-Local 构建和运行指南

## 🚀 快速开始

### 一键构建并运行

```bash
./build_with_visualization.sh
```

这个脚本会：
1. ✅ 检查并下载 ImGui（如果需要）
2. ✅ 检查 SDL2 依赖
3. ✅ 清理旧的构建
4. ✅ 配置 CMake（启用可视化）
5. ✅ 编译项目
6. ✅ **自动运行** navsim_algo

---

## 📋 前置条件

### 系统依赖

**Ubuntu/Debian**:
```bash
sudo apt-get install libsdl2-dev
```

**macOS**:
```bash
brew install sdl2
```

### 其他依赖

- CMake 3.16+
- C++17 编译器
- Protobuf
- ixwebsocket

---

## ⚙️ 配置文件

### 默认配置

配置文件位于 `config/default.json`，包含：

- ✅ **可视化**: 启用 ImGui 可视化
- ✅ **规划器**: A* 规划器（主）+ 直线规划器（降级）
- ✅ **栅格地图**: 30m x 30m，分辨率 0.1m
- ✅ **详细日志**: 启用

### 修改配置

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
          "map_width": 30.0,      // 修改地图宽度
          "map_height": 30.0,     // 修改地图高度
          "resolution": 0.1       // 修改分辨率
        }
      }
    ]
  }
}
```

---

## 🎮 可视化控制

### 键盘快捷键

| 按键 | 功能 |
|------|------|
| `F` | 切换跟随自车模式 |
| `+` | 放大视图 |
| `-` | 缩小视图 |
| `ESC` | 关闭窗口 |

### Legend 面板

**Visualization Elements**（可视化元素）:
- ☑ Show Ego Vehicle（显示自车）
- ☑ Show Goal Point（显示目标点）
- ☑ Show Trajectory（显示轨迹）
- ☑ Show BEV Obstacles（显示 BEV 障碍物）
- ☑ Show Dynamic Obstacles（显示动态障碍物）
- ☑ Show Occupancy Grid（显示栅格地图）

**Display Options**（显示选项）:
- ☑ Show Coordinate Axes（显示坐标轴）
- ☑ Show Grid Lines（显示网格线）

**View Options**（视图选项）:
- ☑ Follow Ego Vehicle（跟随自车）
- [Fit Occupancy Grid]（适应栅格地图）

---

## 🔧 手动构建和运行

### 1. 仅构建（不运行）

如果您想手动控制运行，可以修改 `build_with_visualization.sh`，注释掉最后的自动运行部分：

```bash
# 自动运行
# ./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/default.json
```

然后手动运行：

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/default.json
```

### 2. 使用不同的 WebSocket URL 或 Room ID

```bash
./build/navsim_algo ws://YOUR_SERVER:PORT/ws YOUR_ROOM_ID --config=config/default.json
```

### 3. 不使用配置文件

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

这将使用代码中的默认配置（不推荐）。

---

## 📊 调试信息

### 启动时的日志

```
[AlgorithmManager] GridMapBuilder config:
  - map_width: 30.0 m
  - map_height: 30.0 m
  - resolution: 0.1 m/cell
  - inflation_radius: 0.0 m
```

### 运行时的日志

```
[Viz] Occupancy Grid Boundary:
  - Grid size: 300x300
  - Resolution: 0.1 m
  - World size: 30.0 x 30.0 m
  - View center: (0.6, 0.2)
  - View zoom: 1.0
```

---

## 🐛 常见问题

### 1. SDL2 未找到

**错误**:
```
❌ SDL2 not found!
```

**解决**:
```bash
# Ubuntu/Debian
sudo apt-get install libsdl2-dev

# macOS
brew install sdl2
```

### 2. ImGui 未找到

**错误**:
```
❌ ImGui not found!
```

**解决**:
脚本会自动下载 ImGui。如果失败，手动下载：
```bash
cd third_party
git clone https://github.com/ocornut/imgui.git --depth 1
```

### 3. 栅格地图边界看不到

**原因**: 栅格地图太大，边界在视野外

**解决**:
1. 点击 Legend 面板中的 "Fit Occupancy Grid" 按钮
2. 或者修改 `config/default.json` 中的 `map_width` 和 `map_height`（建议 10-50m）

### 4. 配置文件未生效

**原因**: 配置文件路径错误或 JSON 格式错误

**解决**:
1. 检查配置文件路径是否正确
2. 使用 JSON 验证器检查格式
3. 查看启动日志中的 "Loaded config from" 信息

---

## 📝 文件结构

```
navsim-local/
├── build_with_visualization.sh   # 构建并运行脚本
├── config/
│   ├── default.json              # 默认配置
│   └── README.md                 # 配置文档
├── build/
│   └── navsim_algo               # 编译后的可执行文件
└── ...
```

---

## 💡 最佳实践

1. **快速开始**: 直接运行 `./build_with_visualization.sh`
2. **调整地图大小**: 根据场景大小修改 `map_width` 和 `map_height`
3. **启用详细日志**: `verbose_logging: true` 方便调试
4. **使用 Fit 按钮**: 点击 "Fit Occupancy Grid" 查看完整地图
5. **切换规划器**: 修改 `primary_planner` 测试不同规划器

---

## 🎯 下一步

- 查看 `config/README.md` 了解详细配置说明
- 查看 `VISUALIZATION_IMPROVEMENTS.md` 了解可视化功能
- 修改 `config/default.json` 自定义配置

---

**最后更新**: 2025-10-15

