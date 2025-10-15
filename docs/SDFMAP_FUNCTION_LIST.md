# SDFmap 完整函数清单

## 📋 说明

本文档列出原始 `sdf_map.cpp` 中所有公有函数的完整签名、功能说明和位置。

---

## 1. 坐标转换函数

### 1.1 gridIndex2coordd (重载1)

```cpp
Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index)
```

- **功能**：栅格索引转世界坐标
- **参数**：
  - `index`：栅格索引 (x, y)
- **返回**：世界坐标 (x, y) 米
- **位置**：`sdf_map.cpp:453`
- **实现**：
  ```cpp
  Eigen::Vector2d pt;
  pt(0) = ((double)index(0) + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)index(1) + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 1.2 gridIndex2coordd (重载2)

```cpp
Eigen::Vector2d gridIndex2coordd(const int &x, const int &y)
```

- **功能**：栅格索引转世界坐标（分离参数版本）
- **参数**：
  - `x`：栅格 X 索引
  - `y`：栅格 Y 索引
- **返回**：世界坐标 (x, y) 米
- **位置**：`sdf_map.cpp:460`
- **实现**：
  ```cpp
  Eigen::Vector2d pt;
  pt(0) = ((double)x + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)y + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 1.3 coord2gridIndex

```cpp
Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt)
```

- **功能**：世界坐标转栅格索引
- **参数**：
  - `pt`：世界坐标 (x, y) 米
- **返回**：栅格索引 (x, y)
- **位置**：`sdf_map.cpp:467`
- **实现**：
  ```cpp
  Eigen::Vector2i idx;
  idx(0) = std::floor((pt(0) - global_x_lower_) * inv_grid_interval_);
  idx(1) = std::floor((pt(1) - global_y_lower_) * inv_grid_interval_);
  return idx;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 1.4 ESDFcoord2gridIndex

```cpp
Eigen::Vector2i ESDFcoord2gridIndex(const Eigen::Vector2d &pt)
```

- **功能**：ESDF 坐标转栅格索引（带 0.5 偏移）
- **参数**：
  - `pt`：世界坐标 (x, y) 米
- **返回**：栅格索引 (x, y)
- **位置**：`sdf_map.cpp:731`
- **实现**：
  ```cpp
  Eigen::Vector2i idx;
  idx(0) = static_cast<int>((pt(0) - global_x_lower_) * inv_grid_interval_ + 0.5);
  idx(1) = static_cast<int>((pt(1) - global_y_lower_) * inv_grid_interval_ + 0.5);
  return idx;
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

## 2. 索引转换函数

### 2.1 Index2Vectornum (重载1)

```cpp
int Index2Vectornum(const int &x, const int &y)
```

- **功能**：2D 栅格索引转 1D 数组索引
- **参数**：
  - `x`：栅格 X 索引
  - `y`：栅格 Y 索引
- **返回**：1D 数组索引
- **位置**：`sdf_map.cpp:503`
- **实现**：
  ```cpp
  return x + y * GLX_SIZE_;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 2.2 Index2Vectornum (重载2)

```cpp
int Index2Vectornum(const Eigen::Vector2i &index)
```

- **功能**：2D 栅格索引转 1D 数组索引（Eigen 版本）
- **参数**：
  - `index`：栅格索引 (x, y)
- **返回**：1D 数组索引
- **位置**：`sdf_map.cpp:507`
- **实现**：
  ```cpp
  return index(0) + index(1) * GLX_SIZE_;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 2.3 vectornum2gridIndex

```cpp
Eigen::Vector2i vectornum2gridIndex(const int &num)
```

- **功能**：1D 数组索引转 2D 栅格索引
- **参数**：
  - `num`：1D 数组索引
- **返回**：栅格索引 (x, y)
- **位置**：`sdf_map.cpp:496`
- **实现**：
  ```cpp
  Eigen::Vector2i index;
  index(0) = num % GLX_SIZE_;
  index(1) = num / GLX_SIZE_;
  return index;
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

## 3. 碰撞检测函数

### 3.1 isOccupied (重载1)

```cpp
bool isOccupied(const Eigen::Vector2i &index)
```

- **功能**：检查栅格是否被占据
- **参数**：
  - `index`：栅格索引 (x, y)
- **返回**：`true` = 占据，`false` = 自由/未知
- **位置**：`sdf_map.cpp:896`
- **实现**：
  ```cpp
  return gridmap_[Index2Vectornum(index)] == Occupied;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 3.2 isOccupied (重载2)

```cpp
bool isOccupied(const int &idx, const int &idy)
```

- **功能**：检查栅格是否被占据（分离参数版本）
- **参数**：
  - `idx`：栅格 X 索引
  - `idy`：栅格 Y 索引
- **返回**：`true` = 占据，`false` = 自由/未知
- **位置**：`sdf_map.cpp:900`
- **实现**：
  ```cpp
  return gridmap_[Index2Vectornum(idx, idy)] == Occupied;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 3.3 isUnOccupied (重载1)

```cpp
bool isUnOccupied(const int &idx, const int &idy)
```

- **功能**：检查栅格是否自由
- **参数**：
  - `idx`：栅格 X 索引
  - `idy`：栅格 Y 索引
- **返回**：`true` = 自由，`false` = 占据/未知
- **位置**：`sdf_map.cpp:904`
- **实现**：
  ```cpp
  return gridmap_[Index2Vectornum(idx, idy)] == Unoccupied;
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

### 3.4 isUnOccupied (重载2)

```cpp
bool isUnOccupied(const Eigen::Vector2i &index)
```

- **功能**：检查栅格是否自由（Eigen 版本）
- **参数**：
  - `index`：栅格索引 (x, y)
- **返回**：`true` = 自由，`false` = 占据/未知
- **位置**：`sdf_map.cpp:908`
- **实现**：
  ```cpp
  return gridmap_[Index2Vectornum(index)] == Unoccupied;
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

### 3.5 isUnknown (重载1)

```cpp
bool isUnknown(const Eigen::Vector2i &index)
```

- **功能**：检查栅格是否未知
- **参数**：
  - `index`：栅格索引 (x, y)
- **返回**：`true` = 未知，`false` = 占据/自由
- **位置**：`sdf_map.cpp:912`
- **实现**：
  ```cpp
  return gridmap_[Index2Vectornum(index)] == Unknown;
  ```
- **JPS 使用频率**：⭐⭐ 低频

---

### 3.6 isUnknown (重载2)

```cpp
bool isUnknown(const int &idx, const int &idy)
```

- **功能**：检查栅格是否未知（分离参数版本）
- **参数**：
  - `idx`：栅格 X 索引
  - `idy`：栅格 Y 索引
- **返回**：`true` = 未知，`false` = 占据/自由
- **位置**：`sdf_map.cpp:916`
- **实现**：
  ```cpp
  return gridmap_[Index2Vectornum(idx, idy)] == Unknown;
  ```
- **JPS 使用频率**：⭐⭐ 低频

---

### 3.7 isOccWithSafeDis (重载1)

```cpp
bool isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis)
```

- **功能**：检查栅格是否在安全距离内被占据
- **参数**：
  - `index`：栅格索引 (x, y)
  - `safe_dis`：安全距离（米）
- **返回**：`true` = 在安全距离内有障碍物，`false` = 安全
- **位置**：`sdf_map.cpp:920`
- **实现**：
  ```cpp
  return getDistance(index) < safe_dis;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 3.8 isOccWithSafeDis (重载2)

```cpp
bool isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis)
```

- **功能**：检查栅格是否在安全距离内被占据（分离参数版本）
- **参数**：
  - `idx`：栅格 X 索引
  - `idy`：栅格 Y 索引
  - `safe_dis`：安全距离（米）
- **返回**：`true` = 在安全距离内有障碍物，`false` = 安全
- **位置**：`sdf_map.cpp:924`
- **实现**：
  ```cpp
  return getDistance(idx, idy) < safe_dis;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 3.9 CheckCollisionBycoord (重载1)

```cpp
uint8_t CheckCollisionBycoord(const Eigen::Vector2d &pt)
```

- **功能**：按世界坐标检查碰撞状态
- **参数**：
  - `pt`：世界坐标 (x, y) 米
- **返回**：`Occupied` / `Unoccupied` / `Unknown`
- **位置**：`sdf_map.cpp:551`
- **实现**：
  ```cpp
  Eigen::Vector2i idx = coord2gridIndex(pt);
  return gridmap_[Index2Vectornum(idx)];
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

### 3.10 CheckCollisionBycoord (重载2)

```cpp
uint8_t CheckCollisionBycoord(const double ptx, const double pty)
```

- **功能**：按世界坐标检查碰撞状态（分离参数版本）
- **参数**：
  - `ptx`：世界 X 坐标（米）
  - `pty`：世界 Y 坐标（米）
- **返回**：`Occupied` / `Unoccupied` / `Unknown`
- **位置**：`sdf_map.cpp:560`
- **实现**：
  ```cpp
  Eigen::Vector2d pt(ptx, pty);
  return CheckCollisionBycoord(pt);
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

## 4. 距离场函数

### 4.1 getDistanceReal

```cpp
double getDistanceReal(const Eigen::Vector2d& pos)
```

- **功能**：获取世界坐标点的距离场值
- **参数**：
  - `pos`：世界坐标 (x, y) 米
- **返回**：距离值（米），正值=自由空间，负值=障碍物内部
- **位置**：`sdf_map.cpp:843`
- **实现**：
  ```cpp
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos);
  return getDistance(idx) * grid_interval_;
  ```
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频

---

### 4.2 getDistance (重载1)

```cpp
double getDistance(const Eigen::Vector2i& id)
```

- **功能**：获取栅格索引的距离场值
- **参数**：
  - `id`：栅格索引 (x, y)
- **返回**：距离值（栅格单位）
- **位置**：`sdf_map.cpp:717`
- **实现**：
  ```cpp
  return distance_buffer_all_[Index2Vectornum(id)];
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

### 4.3 getDistance (重载2)

```cpp
double getDistance(const int& idx, const int& idy)
```

- **功能**：获取栅格索引的距离场值（分离参数版本）
- **参数**：
  - `idx`：栅格 X 索引
  - `idy`：栅格 Y 索引
- **返回**：距离值（栅格单位）
- **位置**：`sdf_map.cpp:724`
- **实现**：
  ```cpp
  return distance_buffer_all_[Index2Vectornum(idx, idy)];
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

### 4.4 getDistWithGradBilinear (重载1)

```cpp
double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad)
```

- **功能**：双线性插值获取距离场值和梯度
- **参数**：
  - `pos`：世界坐标 (x, y) 米
  - `grad`：输出梯度向量（引用）
- **返回**：距离值（米）
- **位置**：`sdf_map.cpp:738`
- **实现**：使用双线性插值计算距离和梯度
- **JPS 使用频率**：⭐⭐⭐ 中频

---

### 4.5 getDistWithGradBilinear (重载2)

```cpp
double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double &mindis)
```

- **功能**：双线性插值获取距离场值和梯度（带最小距离限制）
- **参数**：
  - `pos`：世界坐标 (x, y) 米
  - `grad`：输出梯度向量（引用）
  - `mindis`：最小距离阈值（米）
- **返回**：距离值（米）
- **位置**：`sdf_map.cpp:774`
- **实现**：使用双线性插值，距离小于 mindis 时返回 mindis
- **JPS 使用频率**：⭐⭐ 低频

---

### 4.6 getDistWithGradBilinear (重载3)

```cpp
double getDistWithGradBilinear(const Eigen::Vector2d &pos)
```

- **功能**：双线性插值获取距离场值（仅距离，不计算梯度）
- **参数**：
  - `pos`：世界坐标 (x, y) 米
- **返回**：距离值（米）
- **位置**：`sdf_map.cpp:814`
- **实现**：使用双线性插值计算距离
- **JPS 使用频率**：⭐⭐ 低频

---

### 4.7 getUnkonwnGradBilinear

```cpp
double getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad)
```

- **功能**：双线性插值获取未知区域的梯度
- **参数**：
  - `pos`：世界坐标 (x, y) 米
  - `grad`：输出梯度向量（引用）
- **返回**：距离值（米）
- **位置**：`sdf_map.cpp:851`
- **实现**：类似 getDistWithGradBilinear，但用于未知区域
- **JPS 使用频率**：❌ 不使用

---

## 5. 地图边界函数

### 5.1 isInGloMap

```cpp
bool isInGloMap(const Eigen::Vector2d &pt)
```

- **功能**：检查世界坐标点是否在地图范围内
- **参数**：
  - `pt`：世界坐标 (x, y) 米
- **返回**：`true` = 在地图内，`false` = 超出边界
- **位置**：`sdf_map.cpp:569`
- **实现**：
  ```cpp
  return pt(0) >= global_x_lower_ && pt(0) <= global_x_upper_ &&
         pt(1) >= global_y_lower_ && pt(1) <= global_y_upper_;
  ```
- **JPS 使用频率**：⭐⭐⭐ 中频

---

### 5.2 closetPointInMap

```cpp
Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos)
```

- **功能**：获取最近的地图内点
- **参数**：
  - `pt`：目标点世界坐标（米）
  - `pos`：参考点世界坐标（米）
- **返回**：最近的地图内点坐标（米）
- **位置**：`sdf_map.cpp:573`
- **实现**：将点限制在地图边界内
- **JPS 使用频率**：⭐⭐ 低频

---

## 6. 工具函数

### 6.1 getGridsBetweenPoints2D

```cpp
std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end)
```

- **功能**：Bresenham 直线算法，获取两点之间的所有栅格
- **参数**：
  - `start`：起点栅格索引 (x, y)
  - `end`：终点栅格索引 (x, y)
- **返回**：栅格索引向量
- **位置**：`sdf_map.cpp:387`
- **实现**：经典 Bresenham 算法
- **JPS 使用频率**：⭐⭐⭐⭐⭐ 高频（路径碰撞检测）

---

### 6.2 normalize_angle

```cpp
double normalize_angle(double angle)
```

- **功能**：角度归一化到 [-π, π]
- **参数**：
  - `angle`：输入角度（弧度）
- **返回**：归一化后的角度（弧度）
- **位置**：`sdf_map.cpp:890`
- **实现**：
  ```cpp
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
  ```
- **JPS 使用频率**：⭐⭐ 低频

---

### 6.3 get_update_odom

```cpp
Eigen::Vector2d get_update_odom()
```

- **功能**：获取更新位置（里程计）
- **参数**：无
- **返回**：更新位置 (x, y) 米
- **位置**：`sdf_map.cpp:886`
- **实现**：
  ```cpp
  return update_odom_;
  ```
- **JPS 使用频率**：❌ 不使用

---

## 7. ESDF 计算函数

### 7.1 updateESDF2d

```cpp
void updateESDF2d()
```

- **功能**：更新 2D ESDF（主函数）
- **参数**：无
- **返回**：无
- **位置**：`sdf_map.cpp:596`
- **实现**：调用 fillESDF 进行两次扫描（X 和 Y 方向）
- **JPS 使用频率**：✅ 需要（但在插件层调用）

---

### 7.2 fillESDF

```cpp
template <typename F_get_val, typename F_set_val>
void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
```

- **功能**：Felzenszwalb 距离变换（模板函数）
- **参数**：
  - `f_get_val`：获取值的函数对象
  - `f_set_val`：设置值的函数对象
  - `start`：起始索引
  - `end`：结束索引
  - `dim`：维度大小
- **返回**：无
- **位置**：`sdf_map.cpp:660`
- **实现**：O(n) 距离变换算法
- **JPS 使用频率**：✅ 需要（但在插件层调用）

---

## 8. 设置障碍物函数（不需要移植）

### 8.1 setObs (重载1)

```cpp
void setObs(const Eigen::Vector3d coord)
```

- **功能**：设置障碍物（3D 坐标）
- **参数**：
  - `coord`：世界坐标 (x, y, z) 米
- **返回**：无
- **位置**：`sdf_map.cpp:474`
- **JPS 使用频率**：❌ 不使用

---

### 8.2 setObs (重载2)

```cpp
void setObs(const Eigen::Vector2d coord)
```

- **功能**：设置障碍物（2D 坐标）
- **参数**：
  - `coord`：世界坐标 (x, y) 米
- **返回**：无
- **位置**：`sdf_map.cpp:485`
- **JPS 使用频率**：❌ 不使用

---

### 8.3 grid_insertbox

```cpp
void grid_insertbox(Eigen::Vector3d location, Eigen::Matrix3d euler, Eigen::Vector3d size)
```

- **功能**：插入盒子形状的障碍物
- **参数**：
  - `location`：位置 (x, y, z) 米
  - `euler`：旋转矩阵
  - `size`：尺寸 (length, width, height) 米
- **返回**：无
- **位置**：`sdf_map.cpp:436`
- **JPS 使用频率**：❌ 不使用

---

### 8.4 setCacheOccupancy (重载1)

```cpp
int setCacheOccupancy(Eigen::Vector2d pos, int occ)
```

- **功能**：设置缓存占据状态（世界坐标）
- **参数**：
  - `pos`：世界坐标 (x, y) 米
  - `occ`：占据状态
- **返回**：状态码
- **位置**：`sdf_map.cpp:352`
- **JPS 使用频率**：❌ 不使用

---

### 8.5 setCacheOccupancy (重载2)

```cpp
int setCacheOccupancy(Eigen::Vector2i idx, int occ)
```

- **功能**：设置缓存占据状态（栅格索引）
- **参数**：
  - `idx`：栅格索引 (x, y)
  - `occ`：占据状态
- **返回**：状态码
- **位置**：`sdf_map.cpp:370`
- **JPS 使用频率**：❌ 不使用

---

## 9. ROS 相关函数（不需要移植）

### 9.1 pointCloudCallback

```cpp
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
```

- **功能**：点云回调函数
- **位置**：`sdf_map.cpp:8`
- **JPS 使用频率**：❌ 不使用（ROS 特定）

---

### 9.2 updateOccupancyCallback

```cpp
void updateOccupancyCallback(const ros::TimerEvent& /*event*/)
```

- **功能**：占据栅格更新回调
- **位置**：`sdf_map.cpp:35`
- **JPS 使用频率**：❌ 不使用（ROS 特定）

---

### 9.3 updateESDFCallback

```cpp
void updateESDFCallback(const ros::TimerEvent& /*event*/)
```

- **功能**：ESDF 更新回调
- **位置**：`sdf_map.cpp:416`
- **JPS 使用频率**：❌ 不使用（ROS 特定）

---

### 9.4 visCallback

```cpp
void visCallback(const ros::TimerEvent& /*event*/)
```

- **功能**：可视化回调
- **位置**：`sdf_map.cpp:425`
- **JPS 使用频率**：❌ 不使用（ROS 特定）

---

### 9.5 publish_gridmap

```cpp
void publish_gridmap()
```

- **功能**：发布栅格地图（ROS）
- **位置**：`sdf_map.cpp:511`
- **JPS 使用频率**：❌ 不使用（ROS 特定）

---

### 9.6 publish_ESDF

```cpp
void publish_ESDF()
```

- **功能**：发布 ESDF（ROS）
- **位置**：`sdf_map.cpp:696`
- **JPS 使用频率**：❌ 不使用（ROS 特定）

---

### 9.7 publish_ESDFGrad

```cpp
void publish_ESDFGrad()
```

- **功能**：发布 ESDF 梯度（ROS）
- **位置**：`sdf_map.cpp:928`
- **JPS 使用频率**：❌ 不使用（ROS 特定）

---

## 10. 内部处理函数（不需要移植）

### 10.1 raycastProcess

```cpp
void raycastProcess()
```

- **功能**：光线投射处理
- **位置**：`sdf_map.cpp:132`
- **JPS 使用频率**：❌ 不使用（内部处理）

---

### 10.2 cirSupRaycastProcess

```cpp
void cirSupRaycastProcess()
```

- **功能**：圆形支持光线投射处理
- **位置**：`sdf_map.cpp:178`
- **JPS 使用频率**：❌ 不使用（内部处理）

---

### 10.3 updateOccupancyMap

```cpp
void updateOccupancyMap()
```

- **功能**：更新占据栅格地图
- **位置**：`sdf_map.cpp:281`
- **JPS 使用频率**：❌ 不使用（内部处理）

---

### 10.4 RemoveOutliers

```cpp
void RemoveOutliers()
```

- **功能**：移除离群点
- **位置**：`sdf_map.cpp:316`
- **JPS 使用频率**：❌ 不使用（内部处理）

---

## 📊 总结

### 函数分类统计

| 类别 | 数量 | JPS 使用 |
|------|------|---------|
| **坐标转换** | 4 | ✅ 全部需要 |
| **索引转换** | 3 | ✅ 全部需要 |
| **碰撞检测** | 10 | ✅ 全部需要 |
| **距离场** | 7 | ✅ 6 个需要 |
| **地图边界** | 2 | ✅ 全部需要 |
| **工具函数** | 3 | ✅ 2 个需要 |
| **ESDF 计算** | 2 | ✅ 需要（插件层） |
| **设置障碍物** | 5 | ❌ 不需要 |
| **ROS 相关** | 7 | ❌ 不需要 |
| **内部处理** | 4 | ❌ 不需要 |
| **总计** | **47** | **28 个需要** |

### JPS 使用频率分类

- ⭐⭐⭐⭐⭐ **高频**（15 个）：核心功能，必须优化
- ⭐⭐⭐ **中频**（9 个）：常用功能
- ⭐⭐ **低频**（4 个）：辅助功能
- ❌ **不使用**（19 个）：可以不移植

