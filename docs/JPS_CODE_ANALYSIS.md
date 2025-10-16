# JPS 规划器代码分析

## 📁 文件结构

```
navsim-local/plugins/planning/jps_planner/
├── include/
│   ├── graph_search.h          (282 行) - JPS 搜索算法核心
│   ├── jps_planner.h           (101 行) - JPS 规划器主类
│   └── traj_representation.h   (81 行)  - 数据结构定义
└── src/
    ├── graph_search.cpp        (935 行) - JPS 搜索算法实现
    └── jps_planner.cpp         (525 行) - JPS 规划器实现
```

**总代码量**：约 1924 行

---

## 📋 文件 1：graph_search.h (282 行)

### 数据结构

#### 1. `compare_state<T>` (模板结构体)
**用途**：堆元素比较器
**成员函数**：
- `bool operator()(T a1, T a2) const` - 比较两个状态的 f 值（f = g + h）

#### 2. `State` (结构体)
**用途**：图搜索中的节点
**成员变量**：
- `int id` - 节点 ID
- `int x, y, z` - 栅格坐标
- `int dx, dy, dz` - 方向
- `int parentId` - 父节点 ID
- `priorityQueue::handle_type heapkey` - 堆位置指针
- `double g` - g 代价
- `double h` - 启发式代价
- `bool opened` - 是否已打开
- `bool closed` - 是否已关闭

**构造函数**：
- `State(int id, int x, int y, int dx, int dy)` - 2D 构造函数
- `State(int id, int x, int y, int z, int dx, int dy, int dz)` - 3D 构造函数

#### 3. `JPS2DNeib` (结构体)
**用途**：2D JPS 邻居搜索和剪枝
**成员变量**：
- `int ns[9][2][8]` - 总是添加的邻居
- `int f1[9][2][2]` - 强制邻居检查
- `int f2[9][2][2]` - 如果 f1 强制则添加的邻居
- `static constexpr int nsz[3][2]` - 邻居数量

**成员函数**：
- `JPS2DNeib()` - 构造函数
- `void print()` - 打印调试信息
- `void Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty)` - 私有
- `void FNeib(int dx, int dy, int norm1, int dev, int& fx, int& fy, int& nx, int& ny)` - 私有

#### 4. `JPS3DNeib` (结构体)
**用途**：3D JPS 邻居搜索和剪枝（当前未使用）
**成员变量**：
- `int ns[27][3][26]` - 总是添加的邻居
- `int f1[27][3][12]` - 强制邻居检查
- `int f2[27][3][12]` - 如果 f1 强制则添加的邻居
- `static constexpr int nsz[4][2]` - 邻居数量

**成员函数**：
- `JPS3DNeib()` - 构造函数
- `void Neib(...)` - 私有
- `void FNeib(...)` - 私有

---

### 核心类：GraphSearch

#### 公有成员函数（9 个）

1. **构造函数**
   ```cpp
   GraphSearch(std::shared_ptr<SDFmap> Map, const double &safe_dis);
   ```

2. **2D 规划**
   ```cpp
   bool plan(int xStart, int yStart, int xGoal, int yGoal, bool useJps, int maxExpand = -1);
   ```

3. **3D 规划**（已注释）
   ```cpp
   bool plan(int xStart, int yStart, int zStart, int xGoal, int yGoal, int zGoal, bool useJps, int maxExpand = -1);
   ```

4. **获取路径**
   ```cpp
   std::vector<StatePtr> getPath() const;
   ```

5. **获取 Open Set**
   ```cpp
   std::vector<StatePtr> getOpenSet() const;
   ```

6. **获取 Close Set**
   ```cpp
   std::vector<StatePtr> getCloseSet() const;
   ```

7. **获取所有状态**
   ```cpp
   std::vector<StatePtr> getAllSet() const;
   ```

8. **设置安全距离**
   ```cpp
   void SetSafeDis(const double &safe_dis);
   ```

9. **获取安全距离**
   ```cpp
   double GetSafeDis();
   ```

#### 私有成员函数（11 个）

1. **主规划循环**
   ```cpp
   bool plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id);
   ```

2. **A* 获取后继节点**
   ```cpp
   void getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
   ```

3. **JPS 获取后继节点**
   ```cpp
   void getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
   ```

4. **恢复路径**
   ```cpp
   std::vector<StatePtr> recoverPath(StatePtr node, int id);
   ```

5. **坐标转 ID**
   ```cpp
   int coordToId(int x, int y) const;
   ```

6. **检查是否自由**
   ```cpp
   bool isFree(int x, int y) const;
   ```

7. **检查是否未占据**
   ```cpp
   bool isUnoccupied(int x, int y) const;
   ```

8. **检查是否占据**
   ```cpp
   bool isOccupied(int x, int y) const;
   ```

9. **计算启发式**
   ```cpp
   double getHeur(int x, int y) const;
   ```

10. **检查是否有强制邻居**
    ```cpp
    bool hasForced(int x, int y, int dx, int dy);
    ```

11. **2D 跳跃**
    ```cpp
    bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y);
    ```

#### 成员变量（14 个）

- `std::shared_ptr<SDFmap> map_` - **SDFmap 指针**（需替换为 ESDFMap）
- `int xDim_, yDim_, zDim_` - 地图维度
- `double eps_` - 启发式权重
- `bool verbose_` - 调试输出标志
- `double safe_dis_` - 安全距离
- `const char val_free_ = 0` - 自由空间值
- `int xGoal_, yGoal_, zGoal_` - 目标坐标
- `bool use_2d_` - 是否使用 2D
- `bool use_jps_` - 是否使用 JPS
- `priorityQueue pq_` - 优先队列
- `std::vector<StatePtr> hm_` - 哈希表
- `std::vector<bool> seen_` - 已访问标记
- `std::vector<StatePtr> path_` - 路径
- `std::vector<std::vector<int>> ns_` - 邻居
- `std::shared_ptr<JPS2DNeib> jn2d_` - 2D JPS 邻居
- `std::shared_ptr<JPS3DNeib> jn3d_` - 3D JPS 邻居

---

## 📋 文件 2：jps_planner.h (101 行)

### 核心类：JPSPlanner

#### 公有成员函数（13 个）

1. **构造函数**
   ```cpp
   JPSPlanner(std::shared_ptr<SDFmap> map, const ros::NodeHandle &nh);
   ```

2. **规划**
   ```cpp
   bool plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);
   ```

3. **获取小分辨率路径**
   ```cpp
   void get_small_resolution_path_();
   ```

4. **发布路径**
   ```cpp
   void pubPath(const std::vector<Eigen::Vector2d> &path, const ros::Publisher &pub);
   ```

5. **移除拐角点**
   ```cpp
   std::vector<Eigen::Vector2d> removeCornerPts(const std::vector<Eigen::Vector2d> &path);
   ```

6. **检查直线碰撞**
   ```cpp
   bool checkLineCollision(const Eigen::Vector2d &start, const Eigen::Vector2d &end);
   ```

7. **Bresenham 直线算法**
   ```cpp
   std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
   ```

8. **获取运动学节点（带起始路径）**
   ```cpp
   void getKinoNodeWithStartPath(const std::vector<Eigen::Vector3d> &start_path, const bool if_forward, 
                                 const Eigen::Vector3d &current_state_VAJ, const Eigen::Vector3d &current_state_OAJ);
   ```

9. **获取采样轨迹**
   ```cpp
   void getSampleTraj();
   ```

10. **获取带时间的轨迹**
    ```cpp
    void getTrajsWithTime();
    ```

11. **归一化角度**
    ```cpp
    void normalizeAngle(const double &ref_angle, double &angle);
    ```

12. **评估持续时间**
    ```cpp
    double evaluateDuration(const double &length, const double &startV, const double &endV, const double &maxV, const double &maxA);
    ```

13. **评估长度**
    ```cpp
    double evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA);
    ```

14. **评估速度**
    ```cpp
    double evaluateVel(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA);
    ```

15. **评估位置的时间**
    ```cpp
    double evaluteTimeOfPos(const double &pos, const double &locallength, const double &startV, const double &endV, const double &maxV, const double &maxA);
    ```

16. **JPS 碰撞检查**
    ```cpp
    bool JPS_check_if_collision(const Eigen::Vector2d &pos);
    ```

#### 私有成员变量（18 个）

**参数**：
- `double safe_dis_` - 安全距离
- `double max_jps_dis_` - 最大 JPS 距离
- `double distance_weight_` - 距离权重
- `double yaw_weight_` - 偏航权重
- `double trajCutLength_` - 轨迹截断长度
- `double max_vel_` - 最大速度
- `double max_acc_` - 最大加速度
- `double max_omega_` - 最大角速度
- `double max_domega_` - 最大角加速度
- `double sampletime_` - 采样时间
- `int mintrajNum_` - 最小轨迹数量

**数据**：
- `Eigen::Vector3d start_state_` - 起始状态 (x, y, yaw)
- `Eigen::Vector3d current_state_VAJ_` - 当前状态 VAJ
- `Eigen::Vector3d current_state_OAJ_` - 当前状态 OAJ
- `Eigen::Vector3d end_state_` - 结束状态 (x, y, yaw)
- `bool if_first_point_cut_` - 是否第一个点被截断

**ROS 相关**（需移除）：
- `ros::NodeHandle nh_` - ROS 节点句柄
- `ros::Publisher path_pub_` - 路径发布器
- `ros::Publisher init_path_pub_` - 初始路径发布器
- `ros::Publisher normal_vector_pub_` - 法向量发布器

**核心对象**：
- `std::shared_ptr<SDFmap> map_util_` - **SDFmap 指针**（需替换为 ESDFMap）
- `std::shared_ptr<GraphSearch> graph_search_` - 图搜索对象

**路径数据**：
- `int status_` - 状态
- `std::vector<Eigen::Vector2d> raw_path_` - 原始路径
- `std::vector<Eigen::Vector2d> path_` - 优化后路径
- `std::vector<Eigen::Vector2d> Unoccupied_path_` - 未占据路径
- `std::vector<Eigen::VectorXd> Unoccupied_sample_trajs_` - 未占据采样轨迹
- `std::vector<Eigen::VectorXd> cut_Unoccupied_sample_trajs_` - 截断的未占据采样轨迹
- `std::vector<Eigen::Vector2i> small_resolution_path_` - 小分辨率路径

#### 公有成员变量（2 个）

- `FlatTrajData flat_traj_` - 平坦轨迹数据
- `double jps_truncation_time_` - JPS 截断时间

---

## 📋 文件 3：traj_representation.h (81 行)

### 数据结构

#### 1. `PathNode` (类)
**用途**：路径节点（似乎未在 JPS 中使用）
**成员变量**：
- `Eigen::Vector2i index` - 栅格索引
- `int yaw_idx` - 偏航索引
- `Eigen::Vector3d state` - 状态 (x, y, theta)
- `double g_score, f_score` - 代价
- `double penalty_score` - 惩罚代价
- `Eigen::Vector2d input` - 控制输入
- `PathNode* parent` - 父节点
- `char node_state` - 节点状态

#### 2. `FlatTrajData` (结构体)
**用途**：平坦轨迹数据
**成员变量**：
- `std::vector<Eigen::Vector3d> UnOccupied_traj_pts` - 未占据轨迹点 (yaw, s, t)
- `double UnOccupied_initT` - 未占据初始时间
- `std::vector<Eigen::Vector3d> UnOccupied_positions` - 未占据位置 (x, y, yaw)
- `Eigen::MatrixXd start_state` - 起始状态 (pva)
- `Eigen::MatrixXd final_state` - 结束状态
- `Eigen::Vector3d start_state_XYTheta` - 起始状态 (x, y, theta)
- `Eigen::Vector3d final_state_XYTheta` - 结束状态 (x, y, theta)
- `bool if_cut` - 是否截断

**成员函数**：
- `void printFlatTrajData()` - 打印调试信息

---

## 🔍 SDFmap 依赖分析

### GraphSearch 类中的 SDFmap 调用

| 函数 | 调用位置 | 用途 |
|------|---------|------|
| `map_->GLX_SIZE_` | 构造函数 (line 43) | 获取地图宽度 |
| `map_->GLY_SIZE_` | 构造函数 (line 44) | 获取地图高度 |
| `map_->Index2Vectornum(x,y)` | coordToId (line 61) | 坐标转 ID |
| `map_->isOccWithSafeDis(x,y,safe_dis_)` | isFree (line 71) | 检查是否自由 |
| `map_->isUnOccupied(x,y)` | isUnoccupied (line 77) | 检查是否未占据 |
| `map_->isOccupied(x,y)` | isOccupied (line 85) | 检查是否占据 |

**总计**：6 个 SDFmap 函数调用

### JPSPlanner 类中的 SDFmap 调用

| 函数 | 调用位置 | 用途 |
|------|---------|------|
| `map_util_->coord2gridIndex()` | plan (line 36, 37) | 世界坐标转栅格坐标 |
| `map_util_->gridIndex2coordd()` | plan (line 39, 40, 55) | 栅格坐标转世界坐标 |
| `map_util_->getDistanceReal()` | plan (line 39, 40), JPS_check_if_collision (line 524) | 获取距离 |
| `map_util_->coord2gridIndex()` | get_small_resolution_path_ (line 74, 75) | 坐标转换 |
| `map_util_->coord2gridIndex()` | checkLineCollision (line 140) | 坐标转换 |
| `map_util_->isOccWithSafeDis()` | checkLineCollision (line 142) | 碰撞检查 |

**总计**：4 个 SDFmap 函数调用

### 所有 SDFmap 依赖汇总

1. `GLX_SIZE_` - 地图宽度（成员变量）
2. `GLY_SIZE_` - 地图高度（成员变量）
3. `Index2Vectornum(x, y)` - 坐标转 ID
4. `isOccWithSafeDis(x, y, safe_dis)` - 检查是否占据（带安全距离）
5. `isUnOccupied(x, y)` - 检查是否未占据
6. `isOccupied(x, y)` - 检查是否占据
7. `coord2gridIndex(pos)` - 世界坐标转栅格坐标
8. `gridIndex2coordd(index)` - 栅格坐标转世界坐标
9. `getDistanceReal(pos)` - 获取世界坐标的距离

**✅ 所有函数都在 ESDFMap 中已实现！**

---

## 🎯 总结

### 代码规模
- **总行数**：约 1924 行
- **核心算法**：GraphSearch (935 行) + JPSPlanner (525 行) = 1460 行
- **数据结构**：363 行

### 核心组件
1. **GraphSearch** - JPS/A* 搜索算法（完全独立，可直接移植）
2. **JPSPlanner** - 路径优化、轨迹生成、时间规划（需移除 ROS 依赖）
3. **数据结构** - State, JPS2DNeib, FlatTrajData（可直接复用）

### SDFmap 依赖
- **9 个函数/成员变量**
- **✅ 全部在 ESDFMap 中已实现**
- **替换方式**：`std::shared_ptr<SDFmap>` → `std::shared_ptr<ESDFMap>`

### ROS 依赖
- **需移除**：`ros::NodeHandle`, `ros::Publisher`, `ros::getParam`
- **替换方式**：JSON 配置 + NavSim 日志系统

### 下一步
1. 设计插件架构（算法与插件解耦）
2. 创建详细的移植计划
3. 实现插件接口层

---

## 📊 graph_search.cpp 实现分析 (935 行)

### 主要函数实现

#### 1. 构造函数 (line 40-58)
```cpp
GraphSearch::GraphSearch(std::shared_ptr<SDFmap> Map, const double &safe_dis)
```
- 初始化地图指针、安全距离
- 从 SDFmap 获取地图维度
- 初始化哈希表和访问标记
- 初始化 2D 邻居

#### 2. plan() - 2D 规划 (line 92-115)
```cpp
bool plan(int xStart, int yStart, int xGoal, int yGoal, bool useJps, int maxExpand)
```
- 设置起点和终点
- 初始化起始节点
- 调用主规划循环

#### 3. plan() - 主循环 (line 117-211)
```cpp
bool plan(StatePtr& currNode_ptr, int maxExpand, int start_id, int goal_id)
```
- A*/JPS 搜索主循环
- 优先队列管理
- 后继节点扩展
- 路径恢复

#### 4. getSucc() - A* 后继 (line 225-263)
```cpp
void getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs)
```
- 遍历 8 个邻居
- 检查是否自由
- 计算代价

#### 5. getJpsSucc() - JPS 后继 (line 265-348)
```cpp
void getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs)
```
- 使用 JPS2DNeib 剪枝
- 调用 jump() 函数
- 处理强制邻居

#### 6. jump() - JPS 跳跃 (line 351-372)
```cpp
bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y)
```
- 递归跳跃
- 检查强制邻居
- 返回跳点

#### 7. JPS2DNeib 构造 (line 502-590)
- 初始化邻居表
- 初始化强制邻居表

---

## 📊 jps_planner.cpp 实现分析 (525 行)

### 主要函数实现

#### 1. 构造函数 (line 5-29)
```cpp
JPSPlanner::JPSPlanner(std::shared_ptr<SDFmap> map, const ros::NodeHandle &nh)
```
- 初始化 ROS 发布器（需移除）
- 从 ROS 参数服务器读取参数（需替换为 JSON）
- 创建 GraphSearch 对象

#### 2. plan() - 主规划函数 (line 31-67)
```cpp
bool plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
```
- 坐标转换（世界 → 栅格）
- 调用 GraphSearch::plan()
- 路径转换（栅格 → 世界）
- 发布路径（需移除）

#### 3. removeCornerPts() - 路径优化 (line 96-137)
```cpp
std::vector<Eigen::Vector2d> removeCornerPts(const std::vector<Eigen::Vector2d> &path)
```
- 移除 zigzag 路径段
- 直线碰撞检查
- 路径平滑

#### 4. checkLineCollision() - 碰撞检查 (line 139-147)
```cpp
bool checkLineCollision(const Eigen::Vector2d &start, const Eigen::Vector2d &end)
```
- Bresenham 直线算法
- 逐点碰撞检查

#### 5. getGridsBetweenPoints2D() - Bresenham (line 149-176)
```cpp
std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end)
```
- Bresenham 直线算法实现
- 返回直线上的所有栅格

#### 6. getSampleTraj() - 采样轨迹 (line 216-255)
```cpp
void getSampleTraj()
```
- 生成采样轨迹点
- 计算角度和距离
- 填充 5D 状态 (x, y, theta, dtheta, ds)

#### 7. getTrajsWithTime() - 时间规划 (line 257-365)
```cpp
void getTrajsWithTime()
```
- 梯形速度规划
- 时间分配
- 轨迹截断
- 生成 FlatTrajData

#### 8. evaluateDuration() - 持续时间 (line 377-397)
```cpp
double evaluateDuration(const double &length, const double &startV, const double &endV, const double &maxV, const double &maxA)
```
- 梯形速度曲线
- 计算总时间

#### 9. evaluateLength() - 长度评估 (line 402-440)
```cpp
double evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA)
```
- 根据时间计算距离
- 梯形速度曲线

#### 10. evaluateVel() - 速度评估 (line 442-479)
```cpp
double evaluateVel(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA)
```
- 根据时间计算速度
- 梯形速度曲线

---

## 🔧 需要修改的内容

### GraphSearch 类

#### 需要替换
- `std::shared_ptr<SDFmap> map_` → `std::shared_ptr<ESDFMap> map_`
- `#include <plan_env/sdf_map.h>` → `#include <esdf_map.hpp>`

#### 保持不变
- 所有算法逻辑
- 所有数据结构
- 所有函数签名（除了构造函数）

### JPSPlanner 类

#### 需要移除
- `ros::NodeHandle nh_`
- `ros::Publisher path_pub_`, `init_path_pub_`, `normal_vector_pub_`
- `void pubPath(...)` 函数
- 所有 `nh_.getParam()` 调用
- 所有 `pub.publish()` 调用
- `#include <ros/ros.h>`, `#include <nav_msgs/Path.h>`, 等

#### 需要替换
- `std::shared_ptr<SDFmap> map_util_` → `std::shared_ptr<ESDFMap> map_util_`
- ROS 参数读取 → JSON 配置读取
- ROS 日志 → NavSim 日志系统

#### 保持不变
- 所有算法逻辑（plan, removeCornerPts, getSampleTraj, getTrajsWithTime, 等）
- 所有数据结构
- 所有工具函数（normalizeAngle, evaluateDuration, 等）

---

## 📦 数据结构依赖

### 外部依赖
- `Eigen` - ✅ NavSim 已有
- `boost::heap::d_ary_heap` - ✅ NavSim 已有
- `std::shared_ptr`, `std::vector`, `std::unordered_map` - ✅ C++ 标准库

### 内部依赖
- `State`, `JPS2DNeib`, `JPS3DNeib` - ✅ 可直接复用
- `FlatTrajData`, `PathNode` - ✅ 可直接复用

**结论**：无需额外依赖，可直接移植！

