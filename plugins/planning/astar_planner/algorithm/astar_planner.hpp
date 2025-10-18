#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace astar_planner {
namespace algorithm {

/**
 * @brief 栅格地图接口
 *
 * 这是算法层需要的最小接口，不依赖具体的地图实现
 */
class GridMapInterface {
public:
  virtual ~GridMapInterface() = default;

  /**
   * @brief 检查位置是否被占用
   * @param x X 坐标 (m)
   * @param y Y 坐标 (m)
   * @return true 如果被占用，false 如果空闲
   */
  virtual bool isOccupied(double x, double y) const = 0;

  /**
   * @brief 获取地图边界
   * @param min_x 最小 X 坐标 (m)
   * @param max_x 最大 X 坐标 (m)
   * @param min_y 最小 Y 坐标 (m)
   * @param max_y 最大 Y 坐标 (m)
   */
  virtual void getBounds(double& min_x, double& max_x,
                        double& min_y, double& max_y) const = 0;

  /**
   * @brief 获取地图分辨率
   * @return 分辨率 (m/cell)
   */
  virtual double getResolution() const = 0;
};

/**
 * @brief AstarPlanner 算法核心实现
 * 
 * 这是纯算法层，不依赖任何平台 API。
 * 只使用标准库和 Eigen，便于复用到其他项目。
 * 
 * 设计原则：
 * - 输入输出都是简单的数据结构（Eigen 向量、STL 容器）
 * - 无状态或状态可重置
 * - 易于单元测试
 */
class AstarPlanner {
public:
  /**
   * @brief 算法配置参数
   *
   * 注意：这是纯数据结构，不包含 JSON 解析逻辑。
   * JSON 解析应该在 adapter 层完成。
   */
  struct Config {
    double max_velocity = 2.0;           // 最大速度 (m/s)
    double max_acceleration = 2.0;       // 最大加速度 (m/s²)
    double step_size = 0.5;              // 搜索步长 (m)
    int max_iterations = 10000;          // 最大迭代次数
    double heuristic_weight = 1.0;       // 启发式权重（1.0 = A*, >1.0 = 加权A*）
    bool allow_diagonal = true;          // 是否允许对角线移动
    double goal_tolerance = 0.5;         // 目标容差 (m)
    double obstacle_inflation = 0.2;     // 障碍物膨胀距离 (m)
  };
  
  /**
   * @brief 路径点
   */
  struct Waypoint {
    Eigen::Vector3d position;  // (x, y, yaw)
    double velocity = 0.0;     // 速度 (m/s)
    double timestamp = 0.0;    // 时间戳 (s)
  };
  
  /**
   * @brief 规划结果
   */
  struct Result {
    bool success = false;
    std::vector<Waypoint> path;
    std::string failure_reason;
    double computation_time_ms = 0.0;
  };
  
  /**
   * @brief 构造函数
   */
  AstarPlanner() = default;
  explicit AstarPlanner(const Config& config);
  
  /**
   * @brief 设置配置
   */
  void setConfig(const Config& config);

  /**
   * @brief 设置地图
   * @param map 栅格地图接口
   */
  void setMap(std::shared_ptr<GridMapInterface> map);

  /**
   * @brief 规划路径
   *
   * @param start 起点 (x, y, yaw)
   * @param goal 终点 (x, y, yaw)
   * @return 规划结果
   *
   * 示例：
   * ```cpp
   * AstarPlanner planner(config);
   * planner.setMap(grid_map);
   * auto result = planner.plan(
   *   Eigen::Vector3d(0, 0, 0),
   *   Eigen::Vector3d(10, 10, 0)
   * );
   * if (result.success) {
   *   for (const auto& wp : result.path) {
   *     std::cout << "(" << wp.position.x() << ", " << wp.position.y() << ")" << std::endl;
   *   }
   * }
   * ```
   */
  Result plan(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal);

  /**
   * @brief 重置算法状态
   */
  void reset();
  
private:
  // ========== 内部数据结构 ==========

  /**
   * @brief 栅格节点
   */
  struct GridNode {
    int x = 0;           // 栅格 X 坐标
    int y = 0;           // 栅格 Y 坐标
    double g = 0.0;      // 从起点到当前节点的代价
    double h = 0.0;      // 从当前节点到终点的启发式代价
    double f = 0.0;      // f = g + h
    GridNode* parent = nullptr;  // 父节点

    GridNode() = default;
    GridNode(int x_, int y_) : x(x_), y(y_) {}
  };

  /**
   * @brief 节点比较器（用于优先队列）
   */
  struct NodeComparator {
    bool operator()(const GridNode* a, const GridNode* b) const {
      return a->f > b->f;  // 最小堆
    }
  };

  /**
   * @brief 节点哈希函数
   */
  struct NodeHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
      return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
  };

  // ========== 成员变量 ==========

  Config config_;
  std::shared_ptr<GridMapInterface> map_;

  // ========== 辅助方法 ==========

  /**
   * @brief 世界坐标转栅格坐标
   */
  void worldToGrid(double wx, double wy, int& gx, int& gy) const;

  /**
   * @brief 栅格坐标转世界坐标
   */
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  /**
   * @brief 计算启发式代价（欧几里得距离）
   */
  double heuristic(int x1, int y1, int x2, int y2) const;

  /**
   * @brief 检查节点是否有效（在地图内且未被占用）
   */
  bool isValid(int x, int y) const;

  /**
   * @brief 获取邻居节点
   */
  std::vector<std::pair<int, int>> getNeighbors(int x, int y) const;

  /**
   * @brief 重建路径
   */
  std::vector<Waypoint> reconstructPath(GridNode* goal_node) const;
};

} // namespace algorithm
} // namespace astar_planner

