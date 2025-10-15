#include "grid_map_builder_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <cmath>
#include <chrono>
#include <iostream>

namespace navsim {
namespace plugins {
namespace perception {

// ========== Config ==========

GridMapBuilderPlugin::Config GridMapBuilderPlugin::Config::fromJson(
    const nlohmann::json& json) {
  Config config;
  
  if (json.contains("resolution")) {
    config.resolution = json["resolution"].get<double>();
  }
  if (json.contains("map_width")) {
    config.map_width = json["map_width"].get<double>();
  }
  if (json.contains("map_height")) {
    config.map_height = json["map_height"].get<double>();
  }
  if (json.contains("obstacle_cost")) {
    config.obstacle_cost = json["obstacle_cost"].get<uint8_t>();
  }
  if (json.contains("inflation_radius")) {
    config.inflation_radius = json["inflation_radius"].get<double>();
  }
  
  return config;
}

// ========== GridMapBuilderPlugin ==========

GridMapBuilderPlugin::GridMapBuilderPlugin()
    : config_(Config{}) {
}

GridMapBuilderPlugin::GridMapBuilderPlugin(const Config& config)
    : config_(config) {
}

plugin::PerceptionPluginMetadata GridMapBuilderPlugin::getMetadata() const {
  plugin::PerceptionPluginMetadata metadata;
  metadata.name = "GridMapBuilder";
  metadata.version = "1.0.0";
  metadata.description = "Build occupancy grid map from BEV obstacles";
  metadata.author = "NavSim Team";
  metadata.requires_raw_data = false;
  metadata.output_data_types = {"occupancy_grid"};
  return metadata;
}

bool GridMapBuilderPlugin::initialize(const nlohmann::json& config) {
  try {
    config_ = Config::fromJson(config);
    
    std::cout << "[GridMapBuilderPlugin] Initialized with config:" << std::endl;
    std::cout << "  - resolution: " << config_.resolution << " m/cell" << std::endl;
    std::cout << "  - map_size: " << config_.map_width << "x" << config_.map_height << " m" << std::endl;
    std::cout << "  - inflation_radius: " << config_.inflation_radius << " m" << std::endl;
    
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[GridMapBuilderPlugin] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool GridMapBuilderPlugin::process(const plugin::PerceptionInput& input,
                                   planning::PlanningContext& context) {
  auto start_time = std::chrono::steady_clock::now();
  
  // 创建栅格地图
  auto grid = std::make_unique<planning::OccupancyGrid>();
  
  // 配置地图参数
  grid->config.resolution = config_.resolution;
  grid->config.width = static_cast<int>(config_.map_width / config_.resolution);
  grid->config.height = static_cast<int>(config_.map_height / config_.resolution);
  
  // 以自车为中心的地图
  grid->config.origin.x = input.ego.pose.x - config_.map_width / 2.0;
  grid->config.origin.y = input.ego.pose.y - config_.map_height / 2.0;
  
  // 初始化栅格数据
  grid->data.resize(grid->config.width * grid->config.height, 0);

  // 添加 BEV 静态障碍物
  addBEVObstacles(input.bev_obstacles, *grid);

  // 🔧 添加动态障碍物
  addDynamicObstacles(input.dynamic_obstacles, *grid);

  // 膨胀处理
  inflateObstacles(*grid);
  
  // 保存到上下文
  context.occupancy_grid = std::move(grid);
  
  // 更新统计信息
  auto end_time = std::chrono::steady_clock::now();
  double elapsed_ms = 
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  stats_.total_processed++;
  stats_.average_time_ms = 
      (stats_.average_time_ms * (stats_.total_processed - 1) + elapsed_ms) / 
      stats_.total_processed;
  
  return true;
}

void GridMapBuilderPlugin::reset() {
  stats_ = Statistics();
}

nlohmann::json GridMapBuilderPlugin::getStatistics() const {
  nlohmann::json stats;
  stats["total_processed"] = stats_.total_processed;
  stats["total_obstacles"] = stats_.total_obstacles;
  stats["average_time_ms"] = stats_.average_time_ms;
  return stats;
}

bool GridMapBuilderPlugin::isAvailable() const {
  return true;
}

// ========== Private Methods ==========

void GridMapBuilderPlugin::addBEVObstacles(const planning::BEVObstacles& bev_obstacles,
                                          planning::OccupancyGrid& grid) {
  // 添加圆形障碍物
  for (const auto& circle : bev_obstacles.circles) {
    addCircleObstacle(circle, grid);
    stats_.total_obstacles++;
  }

  // 添加矩形障碍物
  for (const auto& rect : bev_obstacles.rectangles) {
    addRectangleObstacle(rect, grid);
    stats_.total_obstacles++;
  }

  // 添加多边形障碍物
  for (const auto& polygon : bev_obstacles.polygons) {
    addPolygonObstacle(polygon, grid);
    stats_.total_obstacles++;
  }
}

void GridMapBuilderPlugin::addDynamicObstacles(
    const std::vector<planning::DynamicObstacle>& dynamic_obstacles,
    planning::OccupancyGrid& grid) {
  // 🔧 添加动态障碍物的当前位置到栅格地图
  for (const auto& dyn_obs : dynamic_obstacles) {
    if (dyn_obs.shape_type == "circle") {
      // 圆形动态障碍物
      planning::BEVObstacles::Circle circle;
      circle.center.x = dyn_obs.current_pose.x;
      circle.center.y = dyn_obs.current_pose.y;
      // 使用 length 和 width 的平均值作为半径
      circle.radius = (dyn_obs.length + dyn_obs.width) / 4.0;
      circle.confidence = 1.0;

      addCircleObstacle(circle, grid);
      stats_.total_obstacles++;
    } else if (dyn_obs.shape_type == "rectangle") {
      // 矩形动态障碍物
      planning::BEVObstacles::Rectangle rect;
      rect.pose.x = dyn_obs.current_pose.x;
      rect.pose.y = dyn_obs.current_pose.y;
      rect.pose.yaw = dyn_obs.current_pose.yaw;
      rect.width = dyn_obs.width;
      rect.height = dyn_obs.length;  // 注意：DynamicObstacle 的 length 对应矩形的 height
      rect.confidence = 1.0;

      addRectangleObstacle(rect, grid);
      stats_.total_obstacles++;
    } else {
      // 未知形状，使用包围盒的对角线作为圆形半径
      planning::BEVObstacles::Circle circle;
      circle.center.x = dyn_obs.current_pose.x;
      circle.center.y = dyn_obs.current_pose.y;
      circle.radius = std::sqrt(dyn_obs.length * dyn_obs.length +
                                dyn_obs.width * dyn_obs.width) / 2.0;
      circle.confidence = 1.0;

      addCircleObstacle(circle, grid);
      stats_.total_obstacles++;
    }
  }
}

void GridMapBuilderPlugin::addCircleObstacle(
    const planning::BEVObstacles::Circle& circle,
    planning::OccupancyGrid& grid) {
  // 🔧 修复：精确计算哪些栅格格子的中心点在圆内

  // 计算圆形在栅格中的范围（边界框）
  int min_x, min_y, max_x, max_y;
  if (!worldToGrid(circle.center.x - circle.radius, circle.center.y - circle.radius, grid, min_x, min_y)) {
    min_x = 0;
    min_y = 0;
  }
  if (!worldToGrid(circle.center.x + circle.radius, circle.center.y + circle.radius, grid, max_x, max_y)) {
    max_x = grid.config.width - 1;
    max_y = grid.config.height - 1;
  }

  // 限制在地图范围内
  min_x = std::max(0, min_x);
  min_y = std::max(0, min_y);
  max_x = std::min(grid.config.width - 1, max_x);
  max_y = std::min(grid.config.height - 1, max_y);

  // 遍历边界框内的所有栅格
  for (int gy = min_y; gy <= max_y; ++gy) {
    for (int gx = min_x; gx <= max_x; ++gx) {
      // 计算栅格格子中心点的世界坐标
      double cell_center_x = grid.config.origin.x + (gx + 0.5) * grid.config.resolution;
      double cell_center_y = grid.config.origin.y + (gy + 0.5) * grid.config.resolution;

      // 计算格子中心到圆心的距离
      double dx = cell_center_x - circle.center.x;
      double dy = cell_center_y - circle.center.y;
      double dist_sq = dx * dx + dy * dy;

      // 如果格子中心在圆内，标记为占据
      if (dist_sq <= circle.radius * circle.radius) {
        setGridCell(gx, gy, config_.obstacle_cost, grid);
      }
    }
  }
}

void GridMapBuilderPlugin::addRectangleObstacle(
    const planning::BEVObstacles::Rectangle& rect,
    planning::OccupancyGrid& grid) {
  // 🔧 修复：精确计算旋转矩形内部的栅格格子

  // 计算矩形的包围盒（考虑旋转）
  double cos_yaw = std::cos(rect.pose.yaw);
  double sin_yaw = std::sin(rect.pose.yaw);

  // 矩形的四个角点（在局部坐标系中）
  double half_width = rect.width / 2.0;
  double half_height = rect.height / 2.0;

  // 计算包围盒
  double max_extent = std::sqrt(half_width * half_width + half_height * half_height);

  int min_x, min_y, max_x, max_y;
  if (!worldToGrid(rect.pose.x - max_extent, rect.pose.y - max_extent, grid, min_x, min_y)) {
    min_x = 0;
    min_y = 0;
  }
  if (!worldToGrid(rect.pose.x + max_extent, rect.pose.y + max_extent, grid, max_x, max_y)) {
    max_x = grid.config.width - 1;
    max_y = grid.config.height - 1;
  }

  // 限制在地图范围内
  min_x = std::max(0, min_x);
  min_y = std::max(0, min_y);
  max_x = std::min(grid.config.width - 1, max_x);
  max_y = std::min(grid.config.height - 1, max_y);

  // 遍历包围盒内的所有栅格
  for (int gy = min_y; gy <= max_y; ++gy) {
    for (int gx = min_x; gx <= max_x; ++gx) {
      // 计算栅格格子中心点的世界坐标
      double cell_center_x = grid.config.origin.x + (gx + 0.5) * grid.config.resolution;
      double cell_center_y = grid.config.origin.y + (gy + 0.5) * grid.config.resolution;

      // 将格子中心点转换到矩形的局部坐标系
      double dx = cell_center_x - rect.pose.x;
      double dy = cell_center_y - rect.pose.y;

      // 旋转到矩形的局部坐标系（逆旋转）
      double local_x = dx * cos_yaw + dy * sin_yaw;
      double local_y = -dx * sin_yaw + dy * cos_yaw;

      // 检查是否在矩形内部
      if (std::abs(local_x) <= half_width && std::abs(local_y) <= half_height) {
        setGridCell(gx, gy, config_.obstacle_cost, grid);
      }
    }
  }
}

void GridMapBuilderPlugin::addPolygonObstacle(
    const planning::BEVObstacles::Polygon& polygon,
    planning::OccupancyGrid& grid) {
  if (polygon.vertices.empty()) return;

  // 🔧 修复：使用射线法精确判断点是否在多边形内部

  // 计算多边形的包围盒
  double min_x = polygon.vertices[0].x;
  double min_y = polygon.vertices[0].y;
  double max_x = polygon.vertices[0].x;
  double max_y = polygon.vertices[0].y;

  for (const auto& vertex : polygon.vertices) {
    min_x = std::min(min_x, vertex.x);
    min_y = std::min(min_y, vertex.y);
    max_x = std::max(max_x, vertex.x);
    max_y = std::max(max_y, vertex.y);
  }

  // 转换到栅格坐标
  int grid_min_x, grid_min_y, grid_max_x, grid_max_y;
  if (!worldToGrid(min_x, min_y, grid, grid_min_x, grid_min_y)) {
    grid_min_x = 0;
    grid_min_y = 0;
  }
  if (!worldToGrid(max_x, max_y, grid, grid_max_x, grid_max_y)) {
    grid_max_x = grid.config.width - 1;
    grid_max_y = grid.config.height - 1;
  }

  // 限制在地图范围内
  grid_min_x = std::max(0, grid_min_x);
  grid_min_y = std::max(0, grid_min_y);
  grid_max_x = std::min(grid.config.width - 1, grid_max_x);
  grid_max_y = std::min(grid.config.height - 1, grid_max_y);

  // 遍历包围盒内的所有栅格
  for (int gy = grid_min_y; gy <= grid_max_y; ++gy) {
    for (int gx = grid_min_x; gx <= grid_max_x; ++gx) {
      // 计算栅格格子中心点的世界坐标
      double cell_center_x = grid.config.origin.x + (gx + 0.5) * grid.config.resolution;
      double cell_center_y = grid.config.origin.y + (gy + 0.5) * grid.config.resolution;

      // 使用射线法判断点是否在多边形内部
      if (isPointInPolygon(cell_center_x, cell_center_y, polygon.vertices)) {
        setGridCell(gx, gy, config_.obstacle_cost, grid);
      }
    }
  }
}

// 🔧 新增：射线法判断点是否在多边形内部
bool GridMapBuilderPlugin::isPointInPolygon(double px, double py,
                                           const std::vector<planning::Point2d>& vertices) const {
  if (vertices.size() < 3) return false;

  int crossings = 0;
  size_t n = vertices.size();

  for (size_t i = 0; i < n; ++i) {
    size_t j = (i + 1) % n;

    const auto& vi = vertices[i];
    const auto& vj = vertices[j];

    // 检查射线是否与边相交
    if (((vi.y > py) != (vj.y > py)) &&
        (px < (vj.x - vi.x) * (py - vi.y) / (vj.y - vi.y) + vi.x)) {
      crossings++;
    }
  }

  // 奇数次相交表示在多边形内部
  return (crossings % 2) == 1;
}

void GridMapBuilderPlugin::inflateObstacles(planning::OccupancyGrid& grid) {
  if (config_.inflation_radius <= 0.0) return;
  
  int inflation_cells = static_cast<int>(std::ceil(config_.inflation_radius / grid.config.resolution));
  std::vector<uint8_t> inflated_data = grid.data;
  
  for (int y = 0; y < grid.config.height; ++y) {
    for (int x = 0; x < grid.config.width; ++x) {
      int index = y * grid.config.width + x;
      if (grid.data[index] >= config_.obstacle_cost) {
        // 膨胀障碍物
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            if (dx * dx + dy * dy <= inflation_cells * inflation_cells) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx >= 0 && nx < grid.config.width && ny >= 0 && ny < grid.config.height) {
                int nindex = ny * grid.config.width + nx;
                inflated_data[nindex] = std::max(inflated_data[nindex],
                                                static_cast<uint8_t>(config_.obstacle_cost / 2));
              }
            }
          }
        }
      }
    }
  }
  
  grid.data = std::move(inflated_data);
}

bool GridMapBuilderPlugin::worldToGrid(double world_x, double world_y,
                                      const planning::OccupancyGrid& grid,
                                      int& grid_x, int& grid_y) const {
  grid_x = static_cast<int>((world_x - grid.config.origin.x) / grid.config.resolution);
  grid_y = static_cast<int>((world_y - grid.config.origin.y) / grid.config.resolution);
  
  return (grid_x >= 0 && grid_x < grid.config.width &&
          grid_y >= 0 && grid_y < grid.config.height);
}

void GridMapBuilderPlugin::setGridCell(int grid_x, int grid_y, uint8_t value,
                                      planning::OccupancyGrid& grid) {
  if (grid_x >= 0 && grid_x < grid.config.width &&
      grid_y >= 0 && grid_y < grid.config.height) {
    int index = grid_y * grid.config.width + grid_x;
    grid.data[index] = std::max(grid.data[index], value);
  }
}

} // namespace perception
} // namespace plugins
} // namespace navsim

// 注册插件
namespace {
static navsim::plugin::PerceptionPluginRegistrar<navsim::plugins::perception::GridMapBuilderPlugin>
    grid_map_builder_registrar("GridMapBuilder");
}

