# ESDF Builder 重构方案

## 📋 目标

重构 `esdf_builder` 感知插件，为 JPS 规划器移植做准备：

1. **分离插件接口和算法实现**
2. **完整移植 SDFmap 的所有公有函数**
3. **确保 JPS 规划器可以无缝使用**

---

## 1. 原始 SDFmap 公有函数清单

### 1.1 坐标转换函数（JPS 核心依赖）

| 函数名 | 签名 | 功能 | JPS 使用 |
|--------|------|------|---------|
| `gridIndex2coordd` | `Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index)` | 栅格坐标 → 世界坐标 | ✅ 高频 |
| `gridIndex2coordd` | `Eigen::Vector2d gridIndex2coordd(const int &x, const int &y)` | 栅格坐标 → 世界坐标（重载） | ✅ 高频 |
| `coord2gridIndex` | `Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt)` | 世界坐标 → 栅格坐标 | ✅ 高频 |
| `ESDFcoord2gridIndex` | `Eigen::Vector2i ESDFcoord2gridIndex(const Eigen::Vector2d &pt)` | ESDF 坐标转换（偏移 0.5） | ✅ 中频 |

### 1.2 索引转换函数

| 函数名 | 签名 | 功能 | JPS 使用 |
|--------|------|------|---------|
| `Index2Vectornum` | `int Index2Vectornum(const int &x, const int &y)` | 2D 索引 → 1D 索引 | ✅ 高频 |
| `Index2Vectornum` | `int Index2Vectornum(const Eigen::Vector2i &index)` | 2D 索引 → 1D 索引（重载） | ✅ 高频 |
| `vectornum2gridIndex` | `Eigen::Vector2i vectornum2gridIndex(const int &num)` | 1D 索引 → 2D 索引 | ✅ 中频 |

### 1.3 碰撞检测函数（JPS 核心依赖）

| 函数名 | 签名 | 功能 | JPS 使用 |
|--------|------|------|---------|
| `isOccupied` | `bool isOccupied(const Eigen::Vector2i &index)` | 是否占据 | ✅ 高频 |
| `isOccupied` | `bool isOccupied(const int &idx, const int &idy)` | 是否占据（重载） | ✅ 高频 |
| `isUnOccupied` | `bool isUnOccupied(const int &idx, const int &idy)` | 是否自由 | ✅ 中频 |
| `isUnOccupied` | `bool isUnOccupied(const Eigen::Vector2i &index)` | 是否自由（重载） | ✅ 中频 |
| `isUnknown` | `bool isUnknown(const Eigen::Vector2i &index)` | 是否未知 | ✅ 低频 |
| `isUnknown` | `bool isUnknown(const int &idx, const int &idy)` | 是否未知（重载） | ✅ 低频 |
| `isOccWithSafeDis` | `bool isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis)` | 带安全距离检测 | ✅ 高频 |
| `isOccWithSafeDis` | `bool isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis)` | 带安全距离检测（重载） | ✅ 高频 |
| `CheckCollisionBycoord` | `uint8_t CheckCollisionBycoord(const Eigen::Vector2d &pt)` | 按坐标检查碰撞 | ✅ 中频 |
| `CheckCollisionBycoord` | `uint8_t CheckCollisionBycoord(const double ptx, const double pty)` | 按坐标检查碰撞（重载） | ✅ 中频 |

### 1.4 距离场函数（JPS 核心依赖）

| 函数名 | 签名 | 功能 | JPS 使用 |
|--------|------|------|---------|
| `getDistanceReal` | `double getDistanceReal(const Eigen::Vector2d& pos)` | 获取距离场值 | ✅ 高频 |
| `getDistance` | `double getDistance(const Eigen::Vector2i& id)` | 获取距离（栅格索引） | ✅ 中频 |
| `getDistance` | `double getDistance(const int& idx, const int& idy)` | 获取距离（重载） | ✅ 中频 |
| `getDistWithGradBilinear` | `double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad)` | 双线性插值距离+梯度 | ✅ 中频 |
| `getDistWithGradBilinear` | `double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double &mindis)` | 双线性插值（带最小距离） | ✅ 低频 |
| `getDistWithGradBilinear` | `double getDistWithGradBilinear(const Eigen::Vector2d &pos)` | 双线性插值（仅距离） | ✅ 低频 |

### 1.5 地图边界函数

| 函数名 | 签名 | 功能 | JPS 使用 |
|--------|------|------|---------|
| `isInGloMap` | `bool isInGloMap(const Eigen::Vector2d &pt)` | 是否在地图内 | ✅ 中频 |
| `closetPointInMap` | `Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos)` | 最近的地图内点 | ✅ 低频 |

### 1.6 工具函数

| 函数名 | 签名 | 功能 | JPS 使用 |
|--------|------|------|---------|
| `getGridsBetweenPoints2D` | `std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end)` | Bresenham 直线算法 | ✅ 高频 |
| `normalize_angle` | `double normalize_angle(double angle)` | 角度归一化 | ✅ 低频 |
| `get_update_odom` | `Eigen::Vector2d get_update_odom()` | 获取更新位置 | ❌ 不需要 |

### 1.7 设置障碍物函数（不需要）

| 函数名 | 签名 | 功能 | JPS 使用 |
|--------|------|------|---------|
| `setObs` | `void setObs(const Eigen::Vector3d coord)` | 设置障碍物 | ❌ 不需要 |
| `setObs` | `void setObs(const Eigen::Vector2d coord)` | 设置障碍物（重载） | ❌ 不需要 |
| `grid_insertbox` | `void grid_insertbox(...)` | 插入盒子障碍物 | ❌ 不需要 |

### 1.8 公有成员变量

| 变量名 | 类型 | 说明 | JPS 使用 |
|--------|------|------|---------|
| `GLX_SIZE_` | `int` | 全局地图宽度（栅格数） | ✅ 高频 |
| `GLY_SIZE_` | `int` | 全局地图高度（栅格数） | ✅ 高频 |
| `GLXY_SIZE_` | `int` | 全局地图总栅格数 | ✅ 中频 |
| `grid_interval_` | `double` | 栅格分辨率 | ✅ 高频 |
| `inv_grid_interval_` | `double` | 栅格分辨率倒数 | ✅ 中频 |
| `global_x_lower_` | `double` | 地图 X 下界 | ✅ 高频 |
| `global_x_upper_` | `double` | 地图 X 上界 | ✅ 高频 |
| `global_y_lower_` | `double` | 地图 Y 下界 | ✅ 高频 |
| `global_y_upper_` | `double` | 地图 Y 上界 | ✅ 高频 |

---

## 2. 重构方案

### 2.1 新文件结构

```
plugins/perception/esdf_builder/
├── include/
│   ├── esdf_builder_plugin.hpp      # 插件接口（简化）
│   └── esdf_map.hpp                 # ESDFMap 类（新建）
└── src/
    ├── esdf_builder_plugin.cpp      # 插件实现（简化）
    ├── esdf_map.cpp                 # ESDFMap 实现（新建）
    └── register.cpp                 # 插件注册
```

### 2.2 类设计

#### 2.2.1 ESDFMap 类（新建）

**职责**：封装所有 ESDF 算法和 SDFmap 兼容接口

```cpp
class ESDFMap {
public:
  // ========== 构造/析构 ==========
  ESDFMap();
  ~ESDFMap();
  
  // ========== 配置 ==========
  struct Config {
    double resolution = 0.1;
    double map_width = 30.0;
    double map_height = 30.0;
    double max_distance = 5.0;
  };
  void initialize(const Config& config);
  
  // ========== 主要功能 ==========
  void buildFromOccupancyGrid(const std::vector<uint8_t>& occupancy_grid,
                              const Eigen::Vector2d& origin);
  void computeESDF();
  
  // ========== SDFmap 兼容接口（JPS 需要） ==========
  
  // 坐标转换
  Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index) const;
  Eigen::Vector2d gridIndex2coordd(const int &x, const int &y) const;
  Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt) const;
  Eigen::Vector2i ESDFcoord2gridIndex(const Eigen::Vector2d &pt) const;
  
  // 索引转换
  int Index2Vectornum(const int &x, const int &y) const;
  int Index2Vectornum(const Eigen::Vector2i &index) const;
  Eigen::Vector2i vectornum2gridIndex(const int &num) const;
  
  // 碰撞检测
  bool isOccupied(const Eigen::Vector2i &index) const;
  bool isOccupied(const int &idx, const int &idy) const;
  bool isUnOccupied(const int &idx, const int &idy) const;
  bool isUnOccupied(const Eigen::Vector2i &index) const;
  bool isUnknown(const Eigen::Vector2i &index) const;
  bool isUnknown(const int &idx, const int &idy) const;
  bool isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis) const;
  bool isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis) const;
  uint8_t CheckCollisionBycoord(const Eigen::Vector2d &pt) const;
  uint8_t CheckCollisionBycoord(const double ptx, const double pty) const;
  
  // 距离场
  double getDistanceReal(const Eigen::Vector2d& pos) const;
  double getDistance(const Eigen::Vector2i& id) const;
  double getDistance(const int& idx, const int& idy) const;
  double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad) const;
  double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double &mindis) const;
  double getDistWithGradBilinear(const Eigen::Vector2d &pos) const;
  
  // 地图边界
  bool isInGloMap(const Eigen::Vector2d &pt) const;
  Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos) const;
  
  // 工具函数
  std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end) const;
  double normalize_angle(double angle) const;
  
  // ========== 公有成员变量（SDFmap 兼容） ==========
  int GLX_SIZE_ = 0;
  int GLY_SIZE_ = 0;
  int GLXY_SIZE_ = 0;
  double grid_interval_ = 0.1;
  double inv_grid_interval_ = 10.0;
  double global_x_lower_ = 0.0;
  double global_x_upper_ = 0.0;
  double global_y_lower_ = 0.0;
  double global_y_upper_ = 0.0;
  
private:
  // 内部数据
  std::vector<uint8_t> gridmap_;
  std::vector<double> distance_buffer_all_;
  Eigen::Vector2d origin_;
  
  // ESDF 算法
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size);
  
  // 枚举
  enum { Unknown = 0, Unoccupied = 1, Occupied = 2 };
};
```

#### 2.2.2 ESDFBuilderPlugin 类（简化）

**职责**：仅负责插件接口，委托给 ESDFMap

```cpp
class ESDFBuilderPlugin : public plugin::PerceptionPluginInterface {
public:
  plugin::PerceptionPluginMetadata getMetadata() const override;
  bool initialize(const nlohmann::json& config) override;
  bool process(const plugin::PerceptionInput& input, planning::PlanningContext& context) override;

private:
  std::unique_ptr<ESDFMap> esdf_map_;
  
  // 配置参数
  double resolution_ = 0.1;
  double map_width_ = 30.0;
  double map_height_ = 30.0;
  double max_distance_ = 5.0;
  bool include_dynamic_ = true;
  
  // 辅助函数
  void buildOccupancyGrid(const plugin::PerceptionInput& input, 
                         const planning::Point2d& origin,
                         std::vector<uint8_t>& occupancy_grid);
};
```

---

## 3. 实施步骤

### 步骤 1：创建 ESDFMap 类

1. 创建 `include/esdf_map.hpp`
2. 创建 `src/esdf_map.cpp`
3. 实现所有 SDFmap 兼容接口

### 步骤 2：重构 ESDFBuilderPlugin

1. 简化 `esdf_builder_plugin.hpp`
2. 简化 `esdf_builder_plugin.cpp`
3. 使用组合方式调用 `ESDFMap`

### 步骤 3：测试验证

1. 编译测试
2. 运行测试
3. 验证 JPS 可以使用

---

## 4. 关键注意事项

### 4.1 坐标系统

- **原点**：NavSim 使用动态原点（以自车为中心）
- **SDFmap**：使用固定全局原点
- **需要适配**：在 `coord2gridIndex` 等函数中处理

### 4.2 数据结构

- **gridmap_**：`uint8_t` 数组，存储占据状态
- **distance_buffer_all_**：`double` 数组，存储距离场
- **枚举值**：`Unknown=0, Unoccupied=1, Occupied=2`

### 4.3 性能优化

- 使用 `inline` 关键字标记高频函数
- 避免不必要的拷贝
- 使用引用传递

---

## 5. 总结

### 需要移植的函数总数

- ✅ **必须移植**：28 个函数（JPS 高频/中频使用）
- 🔶 **可选移植**：5 个函数（JPS 低频使用）
- ❌ **不需要移植**：4 个函数（JPS 不使用）

### 工作量估计

- **创建 ESDFMap 类**：4-6 小时
- **重构 ESDFBuilderPlugin**：2-3 小时
- **测试验证**：2-3 小时
- **总计**：8-12 小时（1-1.5 天）

### 预期效果

1. ✅ 代码结构清晰（插件接口 vs 算法实现）
2. ✅ JPS 可以无缝使用（完整的 SDFmap 接口）
3. ✅ 易于维护和扩展
4. ✅ 性能不受影响

