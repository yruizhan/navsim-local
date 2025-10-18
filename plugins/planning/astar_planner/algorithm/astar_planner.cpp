#include "astar_planner.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

namespace astar_planner {
namespace algorithm {

// ========== AstarPlanner ==========

AstarPlanner::AstarPlanner(const Config& config)
    : config_(config) {
}

void AstarPlanner::setConfig(const Config& config) {
  config_ = config;
}

void AstarPlanner::setMap(std::shared_ptr<GridMapInterface> map) {
  map_ = map;
}

AstarPlanner::Result AstarPlanner::plan(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal) {

  auto start_time = std::chrono::high_resolution_clock::now();

  Result result;

  // 检查地图是否设置
  if (!map_) {
    result.success = false;
    result.failure_reason = "Grid map not set";
    return result;
  }

  // 转换为栅格坐标
  int start_x, start_y, goal_x, goal_y;
  worldToGrid(start.x(), start.y(), start_x, start_y);
  worldToGrid(goal.x(), goal.y(), goal_x, goal_y);

  // 检查起点和终点是否有效
  if (!isValid(start_x, start_y)) {
    result.success = false;
    result.failure_reason = "Start position is occupied or out of bounds";
    return result;
  }

  if (!isValid(goal_x, goal_y)) {
    result.success = false;
    result.failure_reason = "Goal position is occupied or out of bounds";
    return result;
  }

  // A* 搜索
  std::priority_queue<GridNode*, std::vector<GridNode*>, NodeComparator> open_list;
  std::unordered_map<std::pair<int, int>, GridNode*, NodeHash> all_nodes;
  std::unordered_map<std::pair<int, int>, bool, NodeHash> closed_list;

  // 创建起点节点
  GridNode* start_node = new GridNode(start_x, start_y);
  start_node->g = 0.0;
  start_node->h = heuristic(start_x, start_y, goal_x, goal_y);
  start_node->f = start_node->g + config_.heuristic_weight * start_node->h;
  start_node->parent = nullptr;

  open_list.push(start_node);
  all_nodes[{start_x, start_y}] = start_node;

  GridNode* goal_node = nullptr;
  int iterations = 0;

  while (!open_list.empty() && iterations < config_.max_iterations) {
    iterations++;

    // 获取 f 值最小的节点
    GridNode* current = open_list.top();
    open_list.pop();

    // 检查是否已在 closed list 中
    if (closed_list[{current->x, current->y}]) {
      continue;
    }

    // 添加到 closed list
    closed_list[{current->x, current->y}] = true;

    // 检查是否到达目标
    double dist_to_goal = heuristic(current->x, current->y, goal_x, goal_y);
    if (dist_to_goal * map_->getResolution() <= config_.goal_tolerance) {
      goal_node = current;
      break;
    }

    // 扩展邻居节点
    auto neighbors = getNeighbors(current->x, current->y);
    for (const auto& [nx, ny] : neighbors) {
      // 跳过已访问的节点
      if (closed_list[{nx, ny}]) {
        continue;
      }

      // 计算新的 g 值
      double dx = nx - current->x;
      double dy = ny - current->y;
      double move_cost = std::sqrt(dx * dx + dy * dy) * map_->getResolution();
      double new_g = current->g + move_cost;

      // 检查是否已在 open list 中
      auto it = all_nodes.find({nx, ny});
      if (it != all_nodes.end()) {
        // 如果新路径更短，更新节点
        if (new_g < it->second->g) {
          it->second->g = new_g;
          it->second->f = new_g + config_.heuristic_weight * it->second->h;
          it->second->parent = current;
          open_list.push(it->second);
        }
      } else {
        // 创建新节点
        GridNode* neighbor = new GridNode(nx, ny);
        neighbor->g = new_g;
        neighbor->h = heuristic(nx, ny, goal_x, goal_y);
        neighbor->f = neighbor->g + config_.heuristic_weight * neighbor->h;
        neighbor->parent = current;

        open_list.push(neighbor);
        all_nodes[{nx, ny}] = neighbor;
      }
    }
  }

  // 清理内存并生成结果
  if (goal_node) {
    result.path = reconstructPath(goal_node);
    result.success = true;
  } else {
    result.success = false;
    if (iterations >= config_.max_iterations) {
      result.failure_reason = "Max iterations reached";
    } else {
      result.failure_reason = "No path found";
    }
  }

  // 清理所有节点
  for (auto& [key, node] : all_nodes) {
    delete node;
  }

  // 计算耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  result.computation_time_ms = duration.count() / 1000.0;

  return result;
}

void AstarPlanner::reset() {
  // A* 算法是无状态的，不需要重置
}

// ========== 辅助方法 ==========

void AstarPlanner::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  if (!map_) {
    gx = gy = 0;
    return;
  }

  double min_x, max_x, min_y, max_y;
  map_->getBounds(min_x, max_x, min_y, max_y);
  double resolution = map_->getResolution();

  gx = static_cast<int>((wx - min_x) / resolution);
  gy = static_cast<int>((wy - min_y) / resolution);
}

void AstarPlanner::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  if (!map_) {
    wx = wy = 0.0;
    return;
  }

  double min_x, max_x, min_y, max_y;
  map_->getBounds(min_x, max_x, min_y, max_y);
  double resolution = map_->getResolution();

  wx = min_x + (gx + 0.5) * resolution;
  wy = min_y + (gy + 0.5) * resolution;
}

double AstarPlanner::heuristic(int x1, int y1, int x2, int y2) const {
  // 欧几里得距离
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy) * map_->getResolution();
}

bool AstarPlanner::isValid(int x, int y) const {
  if (!map_) {
    return false;
  }

  // 转换为世界坐标
  double wx, wy;
  gridToWorld(x, y, wx, wy);

  // 检查是否在地图边界内
  double min_x, max_x, min_y, max_y;
  map_->getBounds(min_x, max_x, min_y, max_y);

  if (wx < min_x || wx > max_x || wy < min_y || wy > max_y) {
    return false;
  }

  // 检查是否被占用（考虑膨胀）
  if (config_.obstacle_inflation > 0.0) {
    // 检查周围区域
    int inflation_cells = static_cast<int>(config_.obstacle_inflation / map_->getResolution()) + 1;
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        double check_wx, check_wy;
        gridToWorld(x + dx, y + dy, check_wx, check_wy);
        if (map_->isOccupied(check_wx, check_wy)) {
          return false;
        }
      }
    }
  } else {
    if (map_->isOccupied(wx, wy)) {
      return false;
    }
  }

  return true;
}

std::vector<std::pair<int, int>> AstarPlanner::getNeighbors(int x, int y) const {
  std::vector<std::pair<int, int>> neighbors;

  // 4-连通或8-连通
  static const std::vector<std::pair<int, int>> directions_4 = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}
  };

  static const std::vector<std::pair<int, int>> directions_8 = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  const auto& directions = config_.allow_diagonal ? directions_8 : directions_4;

  for (const auto& [dx, dy] : directions) {
    int nx = x + dx;
    int ny = y + dy;

    if (isValid(nx, ny)) {
      neighbors.push_back({nx, ny});
    }
  }

  return neighbors;
}

std::vector<AstarPlanner::Waypoint> AstarPlanner::reconstructPath(GridNode* goal_node) const {
  std::vector<Waypoint> path;

  // 从目标节点回溯到起点
  GridNode* current = goal_node;
  while (current != nullptr) {
    double wx, wy;
    gridToWorld(current->x, current->y, wx, wy);

    Waypoint wp;
    wp.position = Eigen::Vector3d(wx, wy, 0.0);
    wp.velocity = config_.max_velocity;
    wp.timestamp = 0.0;  // 稍后计算

    path.push_back(wp);
    current = current->parent;
  }

  // 反转路径（从起点到终点）
  std::reverse(path.begin(), path.end());

  // 计算时间戳
  double total_time = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    path[i].timestamp = total_time;

    if (i + 1 < path.size()) {
      double dist = (path[i + 1].position.head<2>() - path[i].position.head<2>()).norm();
      total_time += dist / config_.max_velocity;
    }
  }

  return path;
}

} // namespace algorithm
} // namespace astar_planner

