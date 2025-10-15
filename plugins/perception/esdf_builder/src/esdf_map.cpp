#include "esdf_map.hpp"
#include <algorithm>
#include <cmath>

namespace navsim {
namespace perception {

ESDFMap::ESDFMap() {
  // 默认构造函数
}

void ESDFMap::initialize(const Config& config) {
  // 保存配置
  grid_interval_ = config.resolution;
  inv_grid_interval_ = 1.0 / grid_interval_;
  max_distance_ = config.max_distance;

  // 计算地图尺寸（栅格数）
  GLX_SIZE_ = static_cast<int>(std::ceil(config.map_width / grid_interval_));
  GLY_SIZE_ = static_cast<int>(std::ceil(config.map_height / grid_interval_));
  GLXY_SIZE_ = GLX_SIZE_ * GLY_SIZE_;

  // 分配内存
  gridmap_.resize(GLXY_SIZE_, Unknown);
  distance_buffer_all_.resize(GLXY_SIZE_, std::numeric_limits<double>::max());

  // 初始化边界（将在 buildFromOccupancyGrid 中更新）
  global_x_lower_ = 0.0;
  global_x_upper_ = config.map_width;
  global_y_lower_ = 0.0;
  global_y_upper_ = config.map_height;
}

void ESDFMap::buildFromOccupancyGrid(const std::vector<uint8_t>& occupancy_grid,
                                     const Eigen::Vector2d& origin) {
  // 保存原点
  origin_ = origin;

  // 更新地图边界
  global_x_lower_ = origin(0);
  global_x_upper_ = origin(0) + GLX_SIZE_ * grid_interval_;
  global_y_lower_ = origin(1);
  global_y_upper_ = origin(1) + GLY_SIZE_ * grid_interval_;

  // 转换占据栅格到内部格式
  // occupancy_grid: 0 = 自由, 100 = 占据
  // gridmap_: Unknown=0, Unoccupied=1, Occupied=2
  for (int i = 0; i < GLXY_SIZE_; ++i) {
    if (i < static_cast<int>(occupancy_grid.size())) {
      if (occupancy_grid[i] > 50) {
        gridmap_[i] = Occupied;
      } else if (occupancy_grid[i] == 0) {
        gridmap_[i] = Unoccupied;
      } else {
        gridmap_[i] = Unknown;
      }
    } else {
      gridmap_[i] = Unknown;
    }
  }
}

void ESDFMap::computeESDF() {
  // ⚠️ 此实现参考 sdf_map.cpp 中的 SDFmap::updateESDF2d()
  // 但简化为全局更新（不使用局部更新）

  std::vector<double> tmp_buffer1(GLXY_SIZE_, 0.0);
  std::vector<double> distance_buffer_pos(GLXY_SIZE_, 0.0);
  std::vector<double> distance_buffer_neg(GLXY_SIZE_, 0.0);

  // ========== 计算正距离场（自由空间到障碍物的距离） ==========

  // X 方向扫描
  for (int x = 0; x < GLX_SIZE_; x++) {
    fillESDF(
      [&](int y) {
        return gridmap_[x * GLY_SIZE_ + y] == Occupied ?
          0.0 : std::numeric_limits<double>::max();
      },
      [&](int y, double val) { tmp_buffer1[x * GLY_SIZE_ + y] = val; },
      0,
      GLY_SIZE_ - 1,
      GLY_SIZE_
    );
  }

  // Y 方向扫描
  for (int y = 0; y < GLY_SIZE_; y++) {
    fillESDF(
      [&](int x) { return tmp_buffer1[x * GLY_SIZE_ + y]; },
      [&](int x, double val) {
        distance_buffer_pos[x * GLY_SIZE_ + y] = grid_interval_ * std::sqrt(val);
      },
      0,
      GLX_SIZE_ - 1,
      GLX_SIZE_
    );
  }

  // ========== 计算负距离场（障碍物内部到自由空间的距离） ==========

  // X 方向扫描
  for (int x = 0; x < GLX_SIZE_; x++) {
    fillESDF(
      [&](int y) {
        int state = gridmap_[x * GLY_SIZE_ + y];
        return (state == Unoccupied || state == Unknown) ?
          0.0 : std::numeric_limits<double>::max();
      },
      [&](int y, double val) { tmp_buffer1[x * GLY_SIZE_ + y] = val; },
      0,
      GLY_SIZE_ - 1,
      GLY_SIZE_
    );
  }

  // Y 方向扫描
  for (int y = 0; y < GLY_SIZE_; y++) {
    fillESDF(
      [&](int x) { return tmp_buffer1[x * GLY_SIZE_ + y]; },
      [&](int x, double val) {
        distance_buffer_neg[x * GLY_SIZE_ + y] = grid_interval_ * std::sqrt(val);
      },
      0,
      GLX_SIZE_ - 1,
      GLX_SIZE_
    );
  }

  // ========== 合并正负距离场 ==========
  for (int x = 0; x < GLX_SIZE_; x++) {
    for (int y = 0; y < GLY_SIZE_; y++) {
      int idx = x * GLY_SIZE_ + y;
      distance_buffer_all_[idx] = distance_buffer_pos[idx];

      if (distance_buffer_neg[idx] > 0.0) {
        distance_buffer_all_[idx] += (-distance_buffer_neg[idx] + grid_interval_);
      }
    }
  }
}

template <typename F_get_val, typename F_set_val>
void ESDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size) {
  // Felzenszwalb 距离变换算法
  // 参考：Distance Transforms of Sampled Functions (Felzenszwalb & Huttenlocher, 2012)
  //
  // ⚠️ 此实现必须与 sdf_map.cpp 中的 SDFmap::fillESDF() 完全一致
  // 唯一的改动：使用 std::vector 代替 VLA（可变长度数组）

  std::vector<int> v(dim_size);
  std::vector<double> z(dim_size + 1);

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;
    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;
  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
  // 注意：不在这里开平方，由调用者负责
}

double ESDFMap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad) const {
  // 双线性插值计算距离和梯度
  Eigen::Vector2i idx = coord2gridIndex(pos);

  if (!isValidIndex(idx)) {
    grad.setZero();
    return 0.0;
  }

  // 计算插值权重
  Eigen::Vector2d pos_grid = (pos - Eigen::Vector2d(global_x_lower_, global_y_lower_)) * inv_grid_interval_;
  double wx = pos_grid(0) - static_cast<double>(idx(0));
  double wy = pos_grid(1) - static_cast<double>(idx(1));

  // 限制在 [0, 1] 范围内
  wx = std::max(0.0, std::min(1.0, wx));
  wy = std::max(0.0, std::min(1.0, wy));

  // 获取四个角点的距离值
  double d00 = getDistance(idx(0), idx(1));
  double d10 = getDistance(idx(0) + 1, idx(1));
  double d01 = getDistance(idx(0), idx(1) + 1);
  double d11 = getDistance(idx(0) + 1, idx(1) + 1);

  // 双线性插值距离
  double dist = (1.0 - wx) * (1.0 - wy) * d00 +
                wx * (1.0 - wy) * d10 +
                (1.0 - wx) * wy * d01 +
                wx * wy * d11;

  // 计算梯度（数值微分）
  grad(0) = ((1.0 - wy) * (d10 - d00) + wy * (d11 - d01)) * inv_grid_interval_;
  grad(1) = ((1.0 - wx) * (d01 - d00) + wx * (d11 - d10)) * inv_grid_interval_;

  return dist * grid_interval_;
}

double ESDFMap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double &mindis) const {
  double dist = getDistWithGradBilinear(pos, grad);
  return std::max(dist, mindis);
}

double ESDFMap::getDistWithGradBilinear(const Eigen::Vector2d &pos) const {
  Eigen::Vector2d grad;
  return getDistWithGradBilinear(pos, grad);
}

Eigen::Vector2d ESDFMap::closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos) const {
  Eigen::Vector2d result = pt;
  
  // 限制在地图边界内
  result(0) = std::max(global_x_lower_, std::min(global_x_upper_, pt(0)));
  result(1) = std::max(global_y_lower_, std::min(global_y_upper_, pt(1)));
  
  return result;
}

std::vector<Eigen::Vector2i> ESDFMap::getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end) const {
  // Bresenham 直线算法
  std::vector<Eigen::Vector2i> grids;

  int x0 = start(0), y0 = start(1);
  int x1 = end(0), y1 = end(1);

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    grids.emplace_back(x0, y0);

    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  return grids;
}

} // namespace perception
} // namespace navsim

