#include "core/planning_context.hpp"
#include <cmath>

namespace navsim {
namespace planning {

// ========== OccupancyGrid 工具函数 ==========

bool OccupancyGrid::isOccupied(int x, int y, uint8_t threshold) const {
  if (x < 0 || x >= config.width || y < 0 || y >= config.height) {
    return true;  // 边界外视为占据
  }

  int index = y * config.width + x;
  return data[index] >= threshold;
}

Point2d OccupancyGrid::cellToWorld(int x, int y) const {
  return {
    config.origin.x + x * config.resolution,
    config.origin.y + y * config.resolution
  };
}

std::pair<int, int> OccupancyGrid::worldToCell(const Point2d& point) const {
  int x = static_cast<int>((point.x - config.origin.x) / config.resolution);
  int y = static_cast<int>((point.y - config.origin.y) / config.resolution);
  return {x, y};
}

// ========== ESDFMap 工具函数 ==========

double ESDFMap::getDistance(int x, int y) const {
  if (x < 0 || x >= config.width || y < 0 || y >= config.height) {
    return config.max_distance;  // 边界外返回最大距离
  }

  int index = y * config.width + x;
  return data[index];
}

double ESDFMap::getDistanceInterpolated(const Point2d& point) const {
  // 将世界坐标转换为栅格坐标（浮点数）
  double fx = (point.x - config.origin.x) / config.resolution - 0.5;
  double fy = (point.y - config.origin.y) / config.resolution - 0.5;

  // 获取整数部分（左下角栅格）
  int x0 = static_cast<int>(std::floor(fx));
  int y0 = static_cast<int>(std::floor(fy));

  // 边界检查
  if (x0 < 0 || x0 >= config.width - 1 || y0 < 0 || y0 >= config.height - 1) {
    return config.max_distance;
  }

  // 获取小数部分（插值权重）
  double dx = fx - x0;
  double dy = fy - y0;

  // 获取四个角点的距离值
  double d00 = getDistance(x0, y0);
  double d10 = getDistance(x0 + 1, y0);
  double d01 = getDistance(x0, y0 + 1);
  double d11 = getDistance(x0 + 1, y0 + 1);

  // 双线性插值
  double d0 = (1 - dx) * d00 + dx * d10;
  double d1 = (1 - dx) * d01 + dx * d11;
  double dist = (1 - dy) * d0 + dy * d1;

  return dist;
}

double ESDFMap::getDistanceWithGradient(const Point2d& point, Point2d& gradient) const {
  // 将世界坐标转换为栅格坐标（浮点数）
  double fx = (point.x - config.origin.x) / config.resolution - 0.5;
  double fy = (point.y - config.origin.y) / config.resolution - 0.5;

  // 获取整数部分（左下角栅格）
  int x0 = static_cast<int>(std::floor(fx));
  int y0 = static_cast<int>(std::floor(fy));

  // 边界检查
  if (x0 < 0 || x0 >= config.width - 1 || y0 < 0 || y0 >= config.height - 1) {
    gradient = {0.0, 0.0};
    return config.max_distance;
  }

  // 获取小数部分（插值权重）
  double dx = fx - x0;
  double dy = fy - y0;

  // 获取四个角点的距离值
  double d00 = getDistance(x0, y0);
  double d10 = getDistance(x0 + 1, y0);
  double d01 = getDistance(x0, y0 + 1);
  double d11 = getDistance(x0 + 1, y0 + 1);

  // 双线性插值计算距离
  double d0 = (1 - dx) * d00 + dx * d10;
  double d1 = (1 - dx) * d01 + dx * d11;
  double dist = (1 - dy) * d0 + dy * d1;

  // 计算梯度（数值微分）
  double inv_resolution = 1.0 / config.resolution;
  gradient.x = ((1 - dy) * (d10 - d00) + dy * (d11 - d01)) * inv_resolution;
  gradient.y = (d1 - d0) * inv_resolution;

  return dist;
}

bool ESDFMap::isWithinSafeDistance(int x, int y, double safe_distance) const {
  double dist = getDistance(x, y);
  return dist < safe_distance;
}

Point2d ESDFMap::cellToWorld(int x, int y) const {
  return {
    config.origin.x + (x + 0.5) * config.resolution,
    config.origin.y + (y + 0.5) * config.resolution
  };
}

std::pair<int, int> ESDFMap::worldToCell(const Point2d& point) const {
  int x = static_cast<int>((point.x - config.origin.x) / config.resolution);
  int y = static_cast<int>((point.y - config.origin.y) / config.resolution);
  return {x, y};
}

} // namespace planning
} // namespace navsim