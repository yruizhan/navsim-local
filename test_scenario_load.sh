#!/bin/bash

# 测试场景加载功能
# 验证：加载新场景后，点击 Start 按钮，地图应该切换

echo "=========================================="
echo "测试场景加载功能"
echo "=========================================="
echo ""
echo "测试步骤："
echo "1. 程序启动后会加载 map1.json"
echo "2. 在 UI 中输入 'map2.json' 并点击 Load 按钮"
echo "3. 观察地图是否切换到 map2"
echo "4. 点击 Start 按钮"
echo "5. 观察地图是否保持为 map2（不应该切换回 map1）"
echo ""
echo "预期结果："
echo "- 加载 map2 后，即使在暂停状态也应该显示 map2 的障碍物"
echo "- 点击 Start 后，应该继续显示 map2 的障碍物"
echo ""
echo "按 Enter 键启动测试..."
read

cd "$(dirname "$0")"
./build/navsim_algo --local-sim --scenario=scenarios/map1.json --visualize

