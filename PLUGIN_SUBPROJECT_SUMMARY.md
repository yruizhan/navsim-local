# 插件子工程架构 - 总结文档

**日期**: 2025-10-13  
**状态**: 设计完成，待实施  
**预计收益**: 大幅提升扩展性和维护性

---

## 🎯 核心改进

### 改进前 vs 改进后

| 方面 | 改进前 | 改进后 |
|------|--------|--------|
| **插件位置** | `src/plugin/plugins/` 混在一起 | `plugins/` 独立子工程 |
| **编译单元** | 所有插件在一个库中 | 每个插件独立编译 |
| **CMake 结构** | 单一 CMakeLists.txt | 分层 CMakeLists.txt |
| **用户扩展** | 需要修改核心 CMake | 独立 CMake 项目 |
| **编译时间** | 修改任何插件全量编译 | 只编译修改的插件 |
| **插件分发** | 无法独立分发 | 可独立打包分发 |
| **版本管理** | 统一版本 | 插件独立版本 |

---

## 📁 新的目录结构

```
navsim-local/
├── CMakeLists.txt                    # 主 CMake（简化）
├── cmake/                            # CMake 配置 ⭐ 新增
│   ├── NavSimPluginConfig.cmake.in   # 插件开发配置
│   └── NavSimPluginHelpers.cmake     # 辅助函数
├── include/
│   ├── core/                         # 核心模块
│   └── plugin/
│       ├── framework/                # 插件框架（SDK）
│       ├── data/                     # 数据结构（SDK）
│       └── preprocessing/            # 前置处理（SDK）
├── src/
│   ├── core/                         # 核心实现
│   └── plugin/
│       ├── framework/                # 插件框架实现
│       └── preprocessing/            # 前置处理实现
├── plugins/                          # 插件子工程 ⭐ 新增
│   ├── CMakeLists.txt
│   ├── README.md
│   ├── perception/
│   │   ├── CMakeLists.txt
│   │   └── grid_map_builder/
│   │       ├── CMakeLists.txt
│   │       ├── include/
│   │       └── src/
│   └── planning/
│       ├── CMakeLists.txt
│       ├── straight_line/
│       ├── astar/
│       └── rrt_star/                 # 示例：新插件
└── external_plugins/                 # 外部插件 ⭐ 新增
    └── README.md
```

---

## 🚀 快速开始

### 方式 1: 自动迁移（推荐）

```bash
cd navsim-local

# 运行迁移脚本
chmod +x scripts/migrate_to_plugin_subprojects.sh
./scripts/migrate_to_plugin_subprojects.sh

# 脚本会自动：
# 1. 创建备份
# 2. 创建目录结构
# 3. 移动插件代码
# 4. 更新包含路径
# 5. 验证迁移
# 6. 测试编译
```

### 方式 2: 手动迁移

参考 `PLUGIN_SUBPROJECT_MIGRATION.md` 文档。

---

## 📦 已创建的文件

### CMake 配置文件
- ✅ `cmake/NavSimPluginConfig.cmake.in` - 插件开发配置模板
- ✅ `cmake/NavSimPluginHelpers.cmake` - CMake 辅助函数

### 插件 CMakeLists.txt
- ✅ `plugins/CMakeLists.txt` - 插件总入口
- ✅ `plugins/perception/CMakeLists.txt` - 感知插件包
- ✅ `plugins/perception/grid_map_builder/CMakeLists.txt` - GridMapBuilder 插件
- ✅ `plugins/planning/CMakeLists.txt` - 规划器插件包
- ✅ `plugins/planning/straight_line/CMakeLists.txt` - StraightLine 插件
- ✅ `plugins/planning/astar/CMakeLists.txt` - A* 插件

### 文档
- ✅ `PLUGIN_SUBPROJECT_DESIGN.md` - 架构设计文档
- ✅ `PLUGIN_SUBPROJECT_MIGRATION.md` - 迁移实施计划
- ✅ `plugins/README.md` - 内置插件开发指南
- ✅ `external_plugins/README.md` - 外部插件开发指南

### 脚本
- ✅ `scripts/migrate_to_plugin_subprojects.sh` - 自动迁移脚本

---

## 🔧 使用示例

### 编译选项

```bash
# 默认：编译所有插件
cmake -B build
cmake --build build

# 禁用所有插件
cmake -B build -DBUILD_PLUGINS=OFF

# 只编译感知插件
cmake -B build -DBUILD_PLANNING_PLUGINS=OFF

# 禁用特定插件
cmake -B build \
  -DBUILD_GRID_MAP_BUILDER_PLUGIN=ON \
  -DBUILD_STRAIGHT_LINE_PLANNER_PLUGIN=ON \
  -DBUILD_ASTAR_PLANNER_PLUGIN=OFF
```

### 增量编译

```bash
# 修改 GridMapBuilder 插件后
cmake --build build --target grid_map_builder_plugin

# 只需要 ~5 秒，而不是 ~30 秒
```

### 添加新插件

```bash
# 1. 创建插件目录
mkdir -p plugins/planning/rrt_star/{include,src}

# 2. 创建 CMakeLists.txt
cat > plugins/planning/rrt_star/CMakeLists.txt << 'EOF'
add_library(rrt_star_planner_plugin STATIC
    src/rrt_star_planner_plugin.cpp)

target_include_directories(rrt_star_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(rrt_star_planner_plugin
    PUBLIC navsim_plugin_framework)

target_compile_features(rrt_star_planner_plugin PUBLIC cxx_std_17)
EOF

# 3. 在 plugins/planning/CMakeLists.txt 中添加
# option(BUILD_RRT_STAR_PLANNER_PLUGIN "Build RRT* planner" ON)
# if(BUILD_RRT_STAR_PLANNER_PLUGIN)
#     add_subdirectory(rrt_star)
#     list(APPEND PLANNING_PLUGIN_LIBS rrt_star_planner_plugin)
# endif()

# 4. 实现插件代码
# ...

# 5. 编译
cmake -B build -DBUILD_RRT_STAR_PLANNER_PLUGIN=ON
cmake --build build
```

---

## 🌐 外部插件开发

### 安装 NavSim SDK

```bash
cd navsim-local
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build
sudo cmake --install build
```

### 创建外部插件

```bash
# 1. 创建项目
mkdir my_custom_planner
cd my_custom_planner
mkdir -p include src

# 2. 创建 CMakeLists.txt
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)
project(my_custom_planner VERSION 1.0.0 LANGUAGES CXX)

find_package(NavSim REQUIRED)

add_library(my_custom_planner_plugin SHARED
    src/my_custom_planner_plugin.cpp)

target_include_directories(my_custom_planner_plugin
    PUBLIC include)

target_link_libraries(my_custom_planner_plugin
    PUBLIC NavSim::navsim_plugin_framework)

target_compile_features(my_custom_planner_plugin PUBLIC cxx_std_17)
EOF

# 3. 实现插件
# ...

# 4. 编译
cmake -B build -DCMAKE_PREFIX_PATH=/usr/local
cmake --build build
```

---

## ✅ 验收标准

迁移完成后，应满足以下标准：

- [ ] 所有内置插件成功编译
- [ ] 插件可以独立编译（修改单个插件只重新编译该插件）
- [ ] 可以通过 CMake 选项禁用特定插件
- [ ] `test_plugin_system` 测试通过
- [ ] `navsim_algo` 主程序正常运行
- [ ] 外部插件可以成功编译和链接
- [ ] 文档完整且准确

---

## 📊 预期收益

### 编译时间

| 场景 | 改进前 | 改进后 | 提升 |
|------|--------|--------|------|
| 全量编译 | ~60s | ~60s | - |
| 修改单个插件 | ~30s | ~5s | **6x** |
| 添加新插件 | 需要修改核心 | 独立项目 | **∞** |

### 开发体验

| 方面 | 改进前 | 改进后 |
|------|--------|--------|
| 添加新插件 | 需要修改 3-4 个文件 | 创建独立目录 |
| 插件测试 | 需要编译整个项目 | 只编译插件 |
| 插件分发 | 无法独立分发 | 可独立打包 |
| 版本管理 | 统一版本 | 独立版本 |

---

## 🔄 迁移步骤概览

1. **准备工作** (10 分钟)
   - 创建备份
   - 创建目录结构

2. **移动代码** (20 分钟)
   - 移动插件源文件
   - 更新包含路径

3. **更新 CMake** (30 分钟)
   - 修改主 CMakeLists.txt
   - 创建插件 CMakeLists.txt

4. **测试验证** (30 分钟)
   - 编译测试
   - 运行测试
   - 验证功能

5. **文档更新** (20 分钟)
   - 更新 README
   - 添加示例

**总计**: 约 2 小时

---

## 📚 相关文档

### 设计文档
- [插件子工程架构设计](PLUGIN_SUBPROJECT_DESIGN.md) - 详细设计方案
- [迁移实施计划](PLUGIN_SUBPROJECT_MIGRATION.md) - 分步实施指南

### 开发指南
- [内置插件开发](plugins/README.md) - 如何添加内置插件
- [外部插件开发](external_plugins/README.md) - 如何开发外部插件

### 原有文档
- [插件架构设计](docs/PLUGIN_ARCHITECTURE_DESIGN.md) - 原始架构设计
- [插件快速参考](docs/PLUGIN_QUICK_REFERENCE.md) - 插件开发速查

---

## 🤝 贡献指南

### 添加新的内置插件

1. 在 `plugins/perception/` 或 `plugins/planning/` 下创建插件目录
2. 实现插件接口
3. 创建 CMakeLists.txt
4. 在父 CMakeLists.txt 中添加选项
5. 更新文档

### 分享外部插件

1. 开发外部插件
2. 创建 GitHub 仓库
3. 提供安装说明
4. 提交 PR 到插件列表

---

## ❓ 常见问题

### Q1: 迁移后原有代码还能用吗？

A: 是的，迁移是向后兼容的。只是改变了目录结构和编译方式。

### Q2: 如何回滚？

A: 使用备份分支：`git checkout backup/before-plugin-subproject-*`

### Q3: 外部插件如何注册？

A: 目前需要在 `plugin_init.cpp` 中手动注册。未来版本将支持动态加载。

### Q4: 可以禁用所有插件吗？

A: 可以，使用 `-DBUILD_PLUGINS=OFF`。但主程序需要至少一个规划器。

---

## 📞 获取帮助

- 查看文档: `docs/` 目录
- 运行示例: `plugins/` 目录
- 提交 Issue: GitHub Issues
- 联系团队: NavSim Team

---

**祝您使用愉快！** 🚀

