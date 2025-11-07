# 快速上手指南

希望立刻体验 NavSim Local？照着下面几个步骤走就能跑起来：


## 1. 手动准备环境

- Linux / macOS（建议）
- C++17 编译器（GCC / Clang）
- CMake ≥ 3.16
- Protobuf（编译器及开发库）
- SDL2 开发包（Linux: `sudo apt-get install libsdl2-dev`，macOS: `brew install sdl2`）

首次运行脚本会自动拉取 ImGui / ImPlot；如果失败，可手动执行：

```bash
cd navsim-local
git submodule update --init --recursive   # 若启用了子模块
mkdir -p third_party && cd third_party
git clone https://github.com/ocornut/imgui.git --depth 1
git clone https://github.com/epezent/implot.git --depth 1
```


## 2. 构建 + 运行

提供三个运行脚本：

- `build_and_run_debug.sh`：单帧静态规划，默认使用esdf插件和jps插件(ddr-opt)， ./build_and_run_debug.sh
- `run_tmpc_test.sh`：单帧静态规划，默认使用拓扑MPC规划插件(t-mpc), ./run_tmpc_test.sh
- `build.sh`：动态仿真脚本，使用时执行 ./build.sh local 
