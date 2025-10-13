# 插件子工程迁移检查清单

**使用说明**: 按照此清单逐步完成迁移，确保不遗漏任何步骤。

---

## 📋 迁移前准备

### 环境检查
- [ ] 确认在 `navsim-local` 根目录
- [ ] 确认 Git 仓库状态干净（或已提交更改）
- [ ] 确认有足够的磁盘空间（至少 500MB）
- [ ] 确认已安装必要的依赖（CMake, Protobuf, Eigen3）

### 备份
- [ ] 创建 Git 备份分支
- [ ] 或创建文件系统备份
- [ ] 记录备份位置: `_________________`

---

## 🏗️ Phase 1: 创建目录结构

### 主目录
- [ ] 创建 `plugins/` 目录
- [ ] 创建 `cmake/` 目录
- [ ] 创建 `external_plugins/` 目录
- [ ] 创建 `scripts/` 目录（如果不存在）

### 感知插件目录
- [ ] 创建 `plugins/perception/`
- [ ] 创建 `plugins/perception/grid_map_builder/`
- [ ] 创建 `plugins/perception/grid_map_builder/include/`
- [ ] 创建 `plugins/perception/grid_map_builder/src/`

### 规划器插件目录
- [ ] 创建 `plugins/planning/`
- [ ] 创建 `plugins/planning/straight_line/`
- [ ] 创建 `plugins/planning/straight_line/include/`
- [ ] 创建 `plugins/planning/straight_line/src/`
- [ ] 创建 `plugins/planning/astar/`
- [ ] 创建 `plugins/planning/astar/include/`
- [ ] 创建 `plugins/planning/astar/src/`

---

## 📦 Phase 2: 复制/创建配置文件

### CMake 配置
- [ ] 复制 `cmake/NavSimPluginConfig.cmake.in`
- [ ] 复制 `cmake/NavSimPluginHelpers.cmake`

### 插件 CMakeLists.txt
- [ ] 复制 `plugins/CMakeLists.txt`
- [ ] 复制 `plugins/perception/CMakeLists.txt`
- [ ] 复制 `plugins/perception/grid_map_builder/CMakeLists.txt`
- [ ] 复制 `plugins/planning/CMakeLists.txt`
- [ ] 复制 `plugins/planning/straight_line/CMakeLists.txt`
- [ ] 复制 `plugins/planning/astar/CMakeLists.txt`

### 文档
- [ ] 复制 `plugins/README.md`
- [ ] 复制 `external_plugins/README.md`
- [ ] 复制 `PLUGIN_SUBPROJECT_DESIGN.md`
- [ ] 复制 `PLUGIN_SUBPROJECT_MIGRATION.md`
- [ ] 复制 `PLUGIN_SUBPROJECT_SUMMARY.md`

### 脚本
- [ ] 复制 `scripts/migrate_to_plugin_subprojects.sh`
- [ ] 添加执行权限: `chmod +x scripts/migrate_to_plugin_subprojects.sh`

---

## 🔄 Phase 3: 移动插件代码

### GridMapBuilder 插件
- [ ] 移动 `include/plugin/plugins/perception/grid_map_builder_plugin.hpp`
  - 目标: `plugins/perception/grid_map_builder/include/`
- [ ] 移动 `src/plugin/plugins/perception/grid_map_builder_plugin.cpp`
  - 目标: `plugins/perception/grid_map_builder/src/`

### StraightLine 规划器
- [ ] 移动 `include/plugin/plugins/planning/straight_line_planner_plugin.hpp`
  - 目标: `plugins/planning/straight_line/include/`
- [ ] 移动 `src/plugin/plugins/planning/straight_line_planner_plugin.cpp`
  - 目标: `plugins/planning/straight_line/src/`

### A* 规划器
- [ ] 移动 `include/plugin/plugins/planning/astar_planner_plugin.hpp`
  - 目标: `plugins/planning/astar/include/`
- [ ] 移动 `src/plugin/plugins/planning/astar_planner_plugin.cpp`
  - 目标: `plugins/planning/astar/src/`

### 清理
- [ ] 删除 `include/plugin/plugins/` 目录
- [ ] 删除 `src/plugin/plugins/` 目录

---

## ✏️ Phase 4: 更新主 CMakeLists.txt

### 修改库定义
- [ ] 将 `navsim_plugin_system` 重命名为 `navsim_plugin_framework`
- [ ] 从 `navsim_plugin_framework` 中移除具体插件源文件
  - [ ] 移除 `src/plugin/plugins/perception/grid_map_builder_plugin.cpp`
  - [ ] 移除 `src/plugin/plugins/planning/straight_line_planner_plugin.cpp`
  - [ ] 移除 `src/plugin/plugins/planning/astar_planner_plugin.cpp`

### 添加插件子工程
- [ ] 添加选项: `option(BUILD_PLUGINS "Build built-in plugins" ON)`
- [ ] 添加子目录: `add_subdirectory(plugins)`

### 更新链接
- [ ] 更新 `navsim_core` 链接到 `navsim_plugin_framework`
- [ ] 更新 `navsim_algo` 链接到 `navsim_builtin_plugins`
- [ ] 更新 `test_plugin_system` 链接（如果需要）

### 添加安装配置
- [ ] 添加 `install(TARGETS ...)` 配置
- [ ] 添加 `install(EXPORT ...)` 配置
- [ ] 添加 `configure_file(...)` 配置

---

## 🔧 Phase 5: 更新插件注册

### 方案 A: 保持现有注册方式
- [ ] 在各插件 CMakeLists.txt 中暴露头文件
- [ ] 更新 `plugin_init.cpp` 中的包含路径

### 方案 B: 插件自注册（推荐）
- [ ] 在每个插件中创建 `register.cpp`
- [ ] 实现自动注册逻辑
- [ ] 更新 `plugin_init.cpp` 为空实现

---

## 🧪 Phase 6: 测试和验证

### 编译测试
- [ ] 清理构建目录: `rm -rf build`
- [ ] 运行 CMake 配置: `cmake -B build`
  - [ ] 检查输出中是否有插件配置信息
  - [ ] 确认没有错误
- [ ] 编译项目: `cmake --build build`
  - [ ] 确认所有插件成功编译
  - [ ] 确认没有链接错误

### 功能测试
- [ ] 运行插件系统测试: `./build/test_plugin_system`
  - [ ] 确认所有插件注册成功
  - [ ] 确认所有插件初始化成功
  - [ ] 确认测试通过
- [ ] 运行主程序: `./build/navsim_algo ws://127.0.0.1:8080/ws demo`
  - [ ] 确认程序正常启动
  - [ ] 确认插件正常工作

### 选项测试
- [ ] 测试禁用所有插件: `cmake -B build-no-plugins -DBUILD_PLUGINS=OFF`
- [ ] 测试禁用感知插件: `cmake -B build-no-perception -DBUILD_PERCEPTION_PLUGINS=OFF`
- [ ] 测试禁用规划器插件: `cmake -B build-no-planning -DBUILD_PLANNING_PLUGINS=OFF`
- [ ] 测试禁用特定插件: `cmake -B build-custom -DBUILD_ASTAR_PLANNER_PLUGIN=OFF`

### 增量编译测试
- [ ] 修改 GridMapBuilder 插件源文件
- [ ] 运行增量编译: `cmake --build build --target grid_map_builder_plugin`
- [ ] 确认只重新编译了该插件（耗时 < 10 秒）

---

## 📝 Phase 7: 文档更新

### 主 README.md
- [ ] 添加插件子工程说明
- [ ] 更新编译选项说明
- [ ] 添加外部插件开发链接

### 插件文档
- [ ] 确认 `plugins/README.md` 完整
- [ ] 确认 `external_plugins/README.md` 完整
- [ ] 确认所有示例代码可运行

### 设计文档
- [ ] 确认 `PLUGIN_SUBPROJECT_DESIGN.md` 准确
- [ ] 确认 `PLUGIN_SUBPROJECT_MIGRATION.md` 准确
- [ ] 确认 `PLUGIN_SUBPROJECT_SUMMARY.md` 准确

---

## 🚀 Phase 8: 外部插件测试（可选）

### 安装 NavSim SDK
- [ ] 运行: `cmake -B build -DCMAKE_INSTALL_PREFIX=/tmp/navsim_install`
- [ ] 运行: `cmake --build build`
- [ ] 运行: `cmake --install build`

### 创建测试插件
- [ ] 创建外部插件项目
- [ ] 编写简单的测试插件
- [ ] 编译外部插件
- [ ] 验证可以链接到 NavSim 框架

---

## ✅ 最终验收

### 功能验收
- [ ] 所有内置插件成功编译
- [ ] 所有测试通过
- [ ] 主程序正常运行
- [ ] 可以通过选项禁用插件
- [ ] 增量编译正常工作

### 性能验收
- [ ] 全量编译时间 ≤ 原来的 1.2 倍
- [ ] 增量编译时间 ≤ 10 秒
- [ ] 运行时性能无下降

### 文档验收
- [ ] 所有文档完整
- [ ] 所有示例可运行
- [ ] 迁移指南清晰

---

## 🎉 完成

### 提交更改
- [ ] 检查所有更改: `git status`
- [ ] 添加更改: `git add -A`
- [ ] 提交: `git commit -m "Refactor: Migrate plugins to independent sub-projects"`
- [ ] 推送（可选）: `git push origin feature/plugin-subproject`

### 清理
- [ ] 删除备份（如果不需要）
- [ ] 删除临时构建目录
- [ ] 更新 `.gitignore`（如果需要）

### 庆祝 🎊
- [ ] 迁移成功完成！
- [ ] 记录迁移时间: `_________________`
- [ ] 记录遇到的问题: `_________________`

---

## 📊 迁移统计

| 指标 | 数值 |
|------|------|
| 迁移开始时间 | _____________ |
| 迁移结束时间 | _____________ |
| 总耗时 | _____________ |
| 移动的文件数 | _____________ |
| 创建的文件数 | _____________ |
| 修改的文件数 | _____________ |
| 遇到的问题数 | _____________ |

---

## 🐛 问题记录

### 问题 1
- **描述**: _________________
- **解决方案**: _________________

### 问题 2
- **描述**: _________________
- **解决方案**: _________________

### 问题 3
- **描述**: _________________
- **解决方案**: _________________

---

## 💡 改进建议

### 对迁移过程的建议
1. _________________
2. _________________
3. _________________

### 对新架构的建议
1. _________________
2. _________________
3. _________________

---

**完成日期**: _________________  
**完成人**: _________________  
**审核人**: _________________

