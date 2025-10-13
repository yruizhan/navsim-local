#!/bin/bash
# 
# NavSim 插件子工程迁移脚本
# 
# 用途: 自动将现有插件代码迁移到独立的子工程结构
# 作者: NavSim Team
# 日期: 2025-10-13

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否在正确的目录
check_directory() {
    if [ ! -f "CMakeLists.txt" ] || [ ! -d "include" ] || [ ! -d "src" ]; then
        log_error "请在 navsim-local 根目录下运行此脚本"
        exit 1
    fi
    log_success "目录检查通过"
}

# 创建备份
create_backup() {
    log_info "创建备份..."
    
    BACKUP_BRANCH="backup/before-plugin-subproject-$(date +%Y%m%d-%H%M%S)"
    
    if git rev-parse --git-dir > /dev/null 2>&1; then
        git checkout -b "$BACKUP_BRANCH"
        git add -A
        git commit -m "Backup before plugin subproject migration" || true
        git checkout -
        log_success "已创建备份分支: $BACKUP_BRANCH"
    else
        log_warning "不是 Git 仓库，跳过 Git 备份"
        
        # 创建文件备份
        BACKUP_DIR="backup_$(date +%Y%m%d-%H%M%S)"
        mkdir -p "$BACKUP_DIR"
        cp -r include src CMakeLists.txt "$BACKUP_DIR/"
        log_success "已创建文件备份: $BACKUP_DIR"
    fi
}

# 创建目录结构
create_directory_structure() {
    log_info "创建插件子工程目录结构..."
    
    # 创建主目录
    mkdir -p plugins/{perception,planning}
    mkdir -p cmake
    mkdir -p external_plugins
    
    # 创建感知插件目录
    mkdir -p plugins/perception/grid_map_builder/{include,src}
    
    # 创建规划器插件目录
    mkdir -p plugins/planning/straight_line/{include,src}
    mkdir -p plugins/planning/astar/{include,src}
    
    log_success "目录结构创建完成"
}

# 移动插件代码
move_plugin_code() {
    log_info "移动插件代码..."
    
    # 检查源文件是否存在
    if [ ! -d "include/plugin/plugins" ]; then
        log_warning "未找到 include/plugin/plugins 目录，可能已经迁移过"
        return
    fi
    
    # 移动感知插件
    if [ -f "include/plugin/plugins/perception/grid_map_builder_plugin.hpp" ]; then
        mv include/plugin/plugins/perception/grid_map_builder_plugin.hpp \
           plugins/perception/grid_map_builder/include/
        log_success "已移动 grid_map_builder_plugin.hpp"
    fi
    
    if [ -f "src/plugin/plugins/perception/grid_map_builder_plugin.cpp" ]; then
        mv src/plugin/plugins/perception/grid_map_builder_plugin.cpp \
           plugins/perception/grid_map_builder/src/
        log_success "已移动 grid_map_builder_plugin.cpp"
    fi
    
    # 移动规划器插件
    if [ -f "include/plugin/plugins/planning/straight_line_planner_plugin.hpp" ]; then
        mv include/plugin/plugins/planning/straight_line_planner_plugin.hpp \
           plugins/planning/straight_line/include/
        log_success "已移动 straight_line_planner_plugin.hpp"
    fi
    
    if [ -f "src/plugin/plugins/planning/straight_line_planner_plugin.cpp" ]; then
        mv src/plugin/plugins/planning/straight_line_planner_plugin.cpp \
           plugins/planning/straight_line/src/
        log_success "已移动 straight_line_planner_plugin.cpp"
    fi
    
    if [ -f "include/plugin/plugins/planning/astar_planner_plugin.hpp" ]; then
        mv include/plugin/plugins/planning/astar_planner_plugin.hpp \
           plugins/planning/astar/include/
        log_success "已移动 astar_planner_plugin.hpp"
    fi
    
    if [ -f "src/plugin/plugins/planning/astar_planner_plugin.cpp" ]; then
        mv src/plugin/plugins/planning/astar_planner_plugin.cpp \
           plugins/planning/astar/src/
        log_success "已移动 astar_planner_plugin.cpp"
    fi
    
    # 删除空目录
    if [ -d "include/plugin/plugins" ]; then
        rm -rf include/plugin/plugins
        log_success "已删除旧的 include/plugin/plugins 目录"
    fi
    
    if [ -d "src/plugin/plugins" ]; then
        rm -rf src/plugin/plugins
        log_success "已删除旧的 src/plugin/plugins 目录"
    fi
}

# 更新头文件包含路径
update_include_paths() {
    log_info "更新头文件包含路径..."
    
    # 在插件源文件中更新包含路径
    find plugins -name "*.cpp" -o -name "*.hpp" | while read file; do
        # 将 "plugin/plugins/xxx" 替换为相对路径
        sed -i 's|#include "plugin/plugins/perception/grid_map_builder_plugin.hpp"|#include "grid_map_builder_plugin.hpp"|g' "$file"
        sed -i 's|#include "plugin/plugins/planning/straight_line_planner_plugin.hpp"|#include "straight_line_planner_plugin.hpp"|g' "$file"
        sed -i 's|#include "plugin/plugins/planning/astar_planner_plugin.hpp"|#include "astar_planner_plugin.hpp"|g' "$file"
    done
    
    log_success "头文件包含路径更新完成"
}

# 验证迁移
verify_migration() {
    log_info "验证迁移结果..."
    
    # 检查必要的文件是否存在
    local all_ok=true
    
    # 检查 CMakeLists.txt 文件
    if [ ! -f "plugins/CMakeLists.txt" ]; then
        log_error "缺少 plugins/CMakeLists.txt"
        all_ok=false
    fi
    
    if [ ! -f "plugins/perception/CMakeLists.txt" ]; then
        log_error "缺少 plugins/perception/CMakeLists.txt"
        all_ok=false
    fi
    
    if [ ! -f "plugins/planning/CMakeLists.txt" ]; then
        log_error "缺少 plugins/planning/CMakeLists.txt"
        all_ok=false
    fi
    
    # 检查插件源文件
    if [ ! -f "plugins/perception/grid_map_builder/src/grid_map_builder_plugin.cpp" ]; then
        log_error "缺少 grid_map_builder_plugin.cpp"
        all_ok=false
    fi
    
    if [ ! -f "plugins/planning/straight_line/src/straight_line_planner_plugin.cpp" ]; then
        log_error "缺少 straight_line_planner_plugin.cpp"
        all_ok=false
    fi
    
    if [ ! -f "plugins/planning/astar/src/astar_planner_plugin.cpp" ]; then
        log_error "缺少 astar_planner_plugin.cpp"
        all_ok=false
    fi
    
    if [ "$all_ok" = true ]; then
        log_success "迁移验证通过"
        return 0
    else
        log_error "迁移验证失败"
        return 1
    fi
}

# 测试编译
test_build() {
    log_info "测试编译..."
    
    # 清理旧的构建
    if [ -d "build" ]; then
        log_info "清理旧的构建目录..."
        rm -rf build
    fi
    
    # 配置
    log_info "运行 CMake 配置..."
    if cmake -B build -S . > build_config.log 2>&1; then
        log_success "CMake 配置成功"
    else
        log_error "CMake 配置失败，查看 build_config.log 了解详情"
        return 1
    fi
    
    # 编译
    log_info "编译项目（这可能需要几分钟）..."
    if cmake --build build > build_compile.log 2>&1; then
        log_success "编译成功"
    else
        log_error "编译失败，查看 build_compile.log 了解详情"
        return 1
    fi
    
    return 0
}

# 主函数
main() {
    echo "========================================="
    echo "  NavSim 插件子工程迁移脚本"
    echo "========================================="
    echo ""
    
    # 步骤 1: 检查目录
    check_directory
    
    # 步骤 2: 创建备份
    read -p "是否创建备份？(y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        create_backup
    else
        log_warning "跳过备份"
    fi
    
    # 步骤 3: 创建目录结构
    create_directory_structure
    
    # 步骤 4: 移动插件代码
    move_plugin_code
    
    # 步骤 5: 更新包含路径
    update_include_paths
    
    # 步骤 6: 验证迁移
    if ! verify_migration; then
        log_error "迁移失败，请检查错误信息"
        exit 1
    fi
    
    # 步骤 7: 测试编译
    read -p "是否测试编译？(y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        if test_build; then
            log_success "测试编译成功"
        else
            log_error "测试编译失败"
            exit 1
        fi
    else
        log_warning "跳过测试编译"
    fi
    
    echo ""
    echo "========================================="
    log_success "迁移完成！"
    echo "========================================="
    echo ""
    echo "后续步骤:"
    echo "1. 查看 PLUGIN_SUBPROJECT_MIGRATION.md 了解详细信息"
    echo "2. 运行测试: ./build/test_plugin_system"
    echo "3. 查看插件文档: plugins/README.md"
    echo ""
}

# 运行主函数
main "$@"

