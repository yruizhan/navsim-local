#!/bin/bash

# NavSim Local - 本地调试模式编译运行脚本

set -e  # 遇到错误立即退出

echo "=========================================="
echo "NavSim Local - Build and Run Debug Mode"
echo "=========================================="
echo ""

# 默认参数
SCENARIO="scenarios/map1.json"
PLANNER="JpsPlanner"
PERCEPTION="EsdfBuilder"
CMAKE_BUILD_TYPE="Release"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --scenario)
            SCENARIO="$2"
            shift 2
            ;;
        --planner)
            PLANNER="$2"
            shift 2
            ;;
        --perception)
            PERCEPTION="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --scenario PATH     场景文件路径 (默认: scenarios/map1.json)"
            echo "  --planner NAME      规划器名称 (默认: JpsPlanner)"
            echo "  --perception NAME   感知模块名称 (默认: EsdfBuilder)"
            echo "  -h, --help         显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0"
            echo "  $0 --scenario scenarios/simple_corridor.json --planner AstarPlanner"
            echo ""
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 -h 或 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 检查场景文件是否存在
if [ ! -f "$SCENARIO" ]; then
    echo "❌ 场景文件不存在: $SCENARIO"
    echo ""
    echo "可用的场景文件:"
    find scenarios -name "*.json" 2>/dev/null | head -10 || echo "  (未找到场景文件)"
    echo ""
    exit 1
fi

# 转换为绝对路径（因为后面会cd到build目录）
SCENARIO_ABS=$(realpath "$SCENARIO")

echo "📋 配置信息:"
echo "  场景文件: $SCENARIO"
echo "  规划器:   $PLANNER"
echo "  感知模块: $PERCEPTION"
echo ""

# 检查 ImGui 是否存在
if [ ! -d "third_party/imgui" ]; then
    echo "❌ ImGui not found!"
    echo "📥 Downloading ImGui..."
    cd third_party
    git clone https://github.com/ocornut/imgui.git --depth 1
    cd ..
    echo "✅ ImGui downloaded successfully"
    echo ""
fi

# 检查 SDL2 是否安装
echo "🔍 Checking SDL2..."
if ! pkg-config --exists sdl2; then
    echo "❌ SDL2 not found!"
    echo ""
    echo "Please install SDL2:"
    echo "  Ubuntu/Debian: sudo apt-get install libsdl2-dev"
    echo "  macOS:         brew install sdl2"
    echo ""
    exit 1
fi
echo "✅ SDL2 found: $(pkg-config --modversion sdl2)"
echo ""

# 创建 build 目录
echo "📁 Preparing build directory..."
mkdir -p build
cd build

# 配置 CMake (可视化默认启用)
echo "⚙️  Configuring CMake..."
cmake .. \
    -DBUILD_PLUGINS=ON \
    -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"

echo ""

# 编译
echo "🔨 Building..."
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo ""
echo "=========================================="
echo "✅ Build completed successfully!"
echo "=========================================="
echo ""
echo "⌨️  可视化控制:"
echo "  鼠标拖拽  - 平移视图"
echo "  鼠标滚轮  - 缩放"
echo "  点击右侧  - 设置新目标点"
echo "  ESC       - 退出程序"
echo ""
echo "🚀 Starting navsim_local_debug..."
echo "   场景: $SCENARIO"
echo "   规划器: $PLANNER"
echo "   感知: $PERCEPTION"
echo ""
echo "=========================================="
echo ""

# 自动运行 navsim_local_debug (可视化默认启用)
./navsim_local_debug \
    --scenario "$SCENARIO_ABS" \
    --planner "$PLANNER" \
    --perception "$PERCEPTION"