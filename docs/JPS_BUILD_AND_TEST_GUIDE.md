# JPS Planner Plugin - Build and Test Guide

## 📋 Prerequisites

### Required Dependencies

1. **C++ Compiler**: GCC 7.0+ or Clang 6.0+
2. **CMake**: 3.10+
3. **Eigen3**: Linear algebra library
4. **Boost**: For heap data structures
5. **nlohmann/json**: JSON parsing library

### Check Dependencies

```bash
# Check GCC version
gcc --version

# Check CMake version
cmake --version

# Check if Eigen3 is installed
pkg-config --modversion eigen3

# Check if Boost is installed
dpkg -l | grep libboost
```

---

## 🔨 Build Instructions

### Step 1: Clean Previous Build (Optional)

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
rm -rf build
```

### Step 2: Configure CMake

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
mkdir -p build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_JPS_PLANNER_PLUGIN=ON \
  -DBUILD_STRAIGHT_LINE_PLANNER_PLUGIN=ON \
  -DBUILD_ASTAR_PLANNER_PLUGIN=ON
```

**CMake Options**:
- `-DCMAKE_BUILD_TYPE=Release`: Build in release mode (optimized)
- `-DCMAKE_BUILD_TYPE=Debug`: Build in debug mode (with debug symbols)
- `-DBUILD_JPS_PLANNER_PLUGIN=ON`: Enable JPS planner plugin
- `-DBUILD_JPS_PLANNER_PLUGIN=OFF`: Disable JPS planner plugin

### Step 3: Build the Plugin

```bash
# Build only JPS planner plugin
make jps_planner_plugin -j8

# Or build all plugins
make -j8
```

**Expected Output**:
```
[ 10%] Building CXX object plugins/planning/jps_planner_plugin/CMakeFiles/jps_planner_plugin.dir/src/graph_search.cpp.o
[ 20%] Building CXX object plugins/planning/jps_planner_plugin/CMakeFiles/jps_planner_plugin.dir/src/jps_planner.cpp.o
[ 30%] Building CXX object plugins/planning/jps_planner_plugin/CMakeFiles/jps_planner_plugin.dir/src/jps_planner_plugin.cpp.o
[ 40%] Linking CXX shared library libjps_planner_plugin.so
[100%] Built target jps_planner_plugin
```

### Step 4: Verify Build

```bash
# Check if the plugin library was created
ls -lh build/plugins/planning/jps_planner_plugin/libjps_planner_plugin.so

# Check symbols in the library
nm -D build/plugins/planning/jps_planner_plugin/libjps_planner_plugin.so | grep JPSPlanner
```

---

## 🧪 Testing

### Unit Tests (TODO)

Create unit tests for individual components:

```cpp
// test_graph_search.cpp
#include "graph_search.hpp"
#include "esdf_map.hpp"
#include <gtest/gtest.h>

TEST(GraphSearchTest, BasicPlanning) {
  // Create a simple ESDF map
  auto esdf_map = std::make_shared<navsim::perception::ESDFMap>(
      100, 100, 0.1, Eigen::Vector2d(0, 0));
  
  // Create GraphSearch
  JPS::GraphSearch graph_search(esdf_map, 0.3);
  
  // Plan from (0, 0) to (10, 10)
  bool success = graph_search.plan(0, 0, 10, 10, true);
  
  EXPECT_TRUE(success);
  EXPECT_GT(graph_search.getPath().size(), 0);
}
```

### Integration Tests

Create integration tests for the full plugin:

```cpp
// test_jps_planner_plugin.cpp
#include "jps_planner_plugin.hpp"
#include <gtest/gtest.h>

TEST(JPSPlannerPluginTest, InitializeAndPlan) {
  // Create plugin
  navsim::plugins::planning::JPSPlannerPlugin plugin;
  
  // Load configuration
  nlohmann::json config = {
    {"verbose", true},
    {"jps", {
      {"safe_dis", 0.3},
      {"max_vel", 1.0},
      {"max_acc", 1.0}
    }}
  };
  
  // Initialize
  ASSERT_TRUE(plugin.initialize(config));
  
  // Create planning context
  navsim::planning::PlanningContext context;
  context.start = {0.0, 0.0, 0.0};
  context.goal = {10.0, 10.0, 0.0};
  context.esdf_map = /* create ESDF map */;
  
  // Plan
  plugin::PlanningResult result;
  bool success = plugin.plan(context, std::chrono::milliseconds(100), result);
  
  EXPECT_TRUE(success);
  EXPECT_GT(result.path.size(), 0);
}
```

### Manual Testing

Create a simple test program:

```cpp
// test_jps_manual.cpp
#include "jps_planner_plugin.hpp"
#include "esdf_builder_plugin.hpp"
#include <iostream>

int main() {
  // 1. Create ESDF map
  navsim::plugins::perception::ESDFBuilderPlugin esdf_plugin;
  nlohmann::json esdf_config = {
    {"resolution", 0.1},
    {"map_width", 30.0},
    {"map_height", 30.0},
    {"max_distance", 5.0}
  };
  esdf_plugin.initialize(esdf_config);
  
  // 2. Create JPS planner
  navsim::plugins::planning::JPSPlannerPlugin jps_plugin;
  nlohmann::json jps_config = {
    {"verbose", true},
    {"jps", {
      {"safe_dis", 0.3},
      {"max_vel", 1.0},
      {"max_acc", 1.0},
      {"max_omega", 1.0}
    }}
  };
  jps_plugin.initialize(jps_config);
  
  // 3. Create planning context
  navsim::planning::PlanningContext context;
  context.start = {0.0, 0.0, 0.0};
  context.goal = {10.0, 10.0, 0.0};
  context.esdf_map = esdf_plugin.getESDFMap();
  
  // 4. Plan
  plugin::PlanningResult result;
  auto start_time = std::chrono::steady_clock::now();
  bool success = jps_plugin.plan(context, std::chrono::milliseconds(100), result);
  auto end_time = std::chrono::steady_clock::now();
  
  // 5. Print results
  if (success) {
    std::cout << "✅ Planning succeeded!" << std::endl;
    std::cout << "  - Path length: " << result.path.size() << " points" << std::endl;
    std::cout << "  - Planning time: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() 
              << " ms" << std::endl;
    
    // Print path
    std::cout << "  - Path:" << std::endl;
    for (size_t i = 0; i < std::min(result.path.size(), size_t(5)); ++i) {
      std::cout << "    [" << i << "] (" << result.path[i].x << ", " << result.path[i].y << ")" << std::endl;
    }
    if (result.path.size() > 5) {
      std::cout << "    ... (" << (result.path.size() - 5) << " more points)" << std::endl;
    }
  } else {
    std::cout << "❌ Planning failed!" << std::endl;
  }
  
  // 6. Print statistics
  auto stats = jps_plugin.getStatistics();
  std::cout << "\nStatistics:" << std::endl;
  std::cout << "  - Total plans: " << stats["total_plans"] << std::endl;
  std::cout << "  - Successful plans: " << stats["successful_plans"] << std::endl;
  std::cout << "  - Failed plans: " << stats["failed_plans"] << std::endl;
  std::cout << "  - Success rate: " << stats["success_rate"] << std::endl;
  std::cout << "  - Avg planning time: " << stats["avg_planning_time_ms"] << " ms" << std::endl;
  
  return success ? 0 : 1;
}
```

**Build and run**:
```bash
g++ -std=c++17 -o test_jps_manual test_jps_manual.cpp \
  -I../include \
  -I../plugins/planning/jps_planner_plugin/include \
  -I../plugins/perception/esdf_builder/include \
  -L./build/plugins/planning/jps_planner_plugin \
  -L./build/plugins/perception/esdf_builder \
  -ljps_planner_plugin \
  -lesdf_builder \
  -lEigen3 \
  -lboost_system

./test_jps_manual
```

---

## 🐛 Troubleshooting

### Build Errors

#### Error: "Eigen3 not found"

```bash
# Install Eigen3
sudo apt-get install libeigen3-dev

# Or specify Eigen3 path
cmake .. -DEigen3_DIR=/path/to/eigen3
```

#### Error: "Boost not found"

```bash
# Install Boost
sudo apt-get install libboost-all-dev

# Or specify Boost path
cmake .. -DBOOST_ROOT=/path/to/boost
```

#### Error: "esdf_map.hpp not found"

Make sure the ESDF builder plugin is built first:

```bash
make esdf_builder -j8
```

### Runtime Errors

#### Error: "ESDF map not available in context"

Make sure the ESDF builder plugin is initialized and has processed the perception data before calling the JPS planner.

#### Error: "Planning failed"

Check the following:
1. Start and goal are within the map bounds
2. Start and goal are not in collision
3. There is a collision-free path between start and goal
4. Safe distance is not too large

---

## 📊 Performance Benchmarks

### Expected Performance

| Scenario | Map Size | Path Length | Planning Time |
|----------|----------|-------------|---------------|
| Simple (no obstacles) | 100x100 | 10m | < 5ms |
| Medium (few obstacles) | 100x100 | 20m | < 20ms |
| Complex (many obstacles) | 100x100 | 30m | < 50ms |
| Very complex | 200x200 | 50m | < 100ms |

### Benchmark Test

```cpp
// benchmark_jps.cpp
#include "jps_planner_plugin.hpp"
#include <chrono>
#include <iostream>

void benchmark(int num_runs) {
  // Setup...
  
  double total_time_ms = 0.0;
  int successful_runs = 0;
  
  for (int i = 0; i < num_runs; ++i) {
    auto start = std::chrono::steady_clock::now();
    bool success = jps_plugin.plan(context, std::chrono::milliseconds(100), result);
    auto end = std::chrono::steady_clock::now();
    
    if (success) {
      successful_runs++;
      total_time_ms += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    }
  }
  
  std::cout << "Benchmark Results (" << num_runs << " runs):" << std::endl;
  std::cout << "  - Success rate: " << (100.0 * successful_runs / num_runs) << "%" << std::endl;
  std::cout << "  - Avg planning time: " << (total_time_ms / successful_runs) << " ms" << std::endl;
  std::cout << "  - Min planning time: " << /* track min */ << " ms" << std::endl;
  std::cout << "  - Max planning time: " << /* track max */ << " ms" << std::endl;
}
```

---

## ✅ Verification Checklist

- [ ] Plugin builds without errors
- [ ] Plugin library (libjps_planner_plugin.so) is created
- [ ] Plugin can be initialized with JSON config
- [ ] Plugin can plan a simple path
- [ ] Plugin can handle obstacles
- [ ] Plugin returns correct path format
- [ ] Plugin statistics are tracked correctly
- [ ] Planning time is reasonable (< 100ms for typical scenarios)
- [ ] Memory usage is reasonable (no leaks)
- [ ] Plugin can be reset and reused

---

## 📝 Next Steps

1. **Write Unit Tests**: Create comprehensive unit tests for all components
2. **Write Integration Tests**: Test the full plugin in realistic scenarios
3. **Performance Optimization**: Profile and optimize critical paths
4. **Documentation**: Add more detailed API documentation
5. **Examples**: Create more example programs
6. **Visualization**: Add visualization support for debugging

---

**Last Updated**: 2025-10-16

