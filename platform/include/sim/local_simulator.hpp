#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

#include "core/planning_context.hpp"
#include "world_tick.pb.h"

namespace navsim {
namespace sim {

// ========== 仿真配置 ==========

struct PhysicsConfig {
  std::string integration_method = "rk4";  // "euler", "rk4"
  bool collision_detection = true;
  double friction_coefficient = 0.8;
};

struct SimulatorConfig {
  double time_step = 0.01;                 // 默认仿真步长 (s)
  double max_time_step = 0.1;              // 最大仿真步长 (s)
  double time_scale = 1.0;                 // 时间缩放 (1.0=实时)
  bool enable_adaptive_stepping = true;    // 自适应步长
  PhysicsConfig physics;
};

struct VisualizationConfig {
  double send_frequency_hz = 30.0;         // 可视化数据发送频率
  bool include_debug_info = true;          // 包含调试信息
  bool compress_data = false;              // 数据压缩
};

// ========== 动态障碍物 ==========

struct DynamicObstacle {
  std::string id;
  planning::Pose2d pose;
  planning::Twist2d twist;

  // 形状信息 (简化版本，后续可扩展)
  enum class Shape { CIRCLE, RECTANGLE } shape = Shape::CIRCLE;
  double radius = 0.3;        // 圆形半径 (m)
  double width = 1.0;         // 矩形宽度 (m)
  double height = 1.0;        // 矩形高度 (m)

  std::string model = "cv";   // "cv"=恒速, "ca"=恒加速, "custom"=自定义

  DynamicObstacle() = default;
  DynamicObstacle(const std::string& id_) : id(id_) {}
};

// ========== 静态障碍物 ==========

struct StaticObstacle {
  enum class Type { CIRCLE, POLYGON } type = Type::CIRCLE;

  // 圆形障碍物
  struct Circle {
    planning::Point2d center;
    double radius;
  } circle;

  // 多边形障碍物
  struct Polygon {
    std::vector<planning::Point2d> points;
  } polygon;

  StaticObstacle() = default;
  StaticObstacle(const planning::Point2d& center, double radius)
    : type(Type::CIRCLE) {
    circle.center = center;
    circle.radius = radius;
  }
};

// ========== 世界状态 ==========

struct WorldState {
  // 时间信息
  double timestamp = 0.0;                  // 仿真时间 (s)
  uint64_t frame_id = 0;                   // 帧ID

  // 自车状态
  planning::Pose2d ego_pose;
  planning::Twist2d ego_twist;
  double ego_acceleration = 0.0;           // 线性加速度 (m/s²)
  double ego_curvature = 0.0;              // 曲率 (1/m)

  // 任务目标
  planning::Pose2d goal_pose;
  double goal_tolerance_pos = 0.3;         // 位置容差 (m)
  double goal_tolerance_yaw = 0.2;         // 朝向容差 (rad)

  // 环境信息
  std::vector<StaticObstacle> static_obstacles;
  std::vector<DynamicObstacle> dynamic_obstacles;

  // 地图版本 (用于检测地图变更)
  uint32_t map_version = 1;

  // 底盘配置
  proto::ChassisConfig chassis_config;

  // 获取自车状态的便捷函数
  planning::EgoVehicle get_ego_state() const {
    planning::EgoVehicle ego;
    ego.pose = ego_pose;
    ego.twist = ego_twist;
    return ego;
  }

  // 获取任务信息的便捷函数
  planning::PlanningTask get_task_info() const {
    planning::PlanningTask task;
    task.goal_pose = goal_pose;
    return task;
  }
};

// ========== 仿真事件回调 ==========

using SimulationStateCallback = std::function<void(bool is_running)>;
using FrameUpdateCallback = std::function<void(const WorldState& world)>;
using CollisionCallback = std::function<void(const std::string& obstacle_id)>;

// ========== 本地仿真器 ==========

/**
 * @brief 本地仿真引擎
 *
 * 职责：
 * - 管理仿真世界状态
 * - 执行物理仿真步进
 * - 处理动态障碍物运动
 * - 提供场景编辑接口
 * - 支持仿真加速/减速
 */
class LocalSimulator {
public:
  LocalSimulator();
  ~LocalSimulator();

  // ========== 初始化与配置 ==========

  /**
   * @brief 初始化仿真器
   * @param config 仿真配置
   * @return 是否成功
   */
  bool initialize(const SimulatorConfig& config);

  /**
   * @brief 从场景文件加载
   * @param scenario_file JSON场景文件路径
   * @param log_callback 可选的日志回调函数，用于向 UI 添加日志
   * @return 是否成功
   */
  bool load_scenario(const std::string& scenario_file,
                     std::function<void(const std::string&)> log_callback = nullptr);

  /**
   * @brief 获取当前配置
   */
  const SimulatorConfig& get_config() const;

  // ========== 仿真控制 ==========

  /**
   * @brief 开始仿真
   */
  void start();

  /**
   * @brief 暂停仿真
   */
  void pause();

  /**
   * @brief 重置仿真到初始状态
   */
  void reset();

  /**
   * @brief 执行单步仿真
   * @param dt 时间步长 (s)，如果为0则使用配置的默认步长
   * @return 是否成功
   */
  bool step(double dt = 0.0);

  /**
   * @brief 检查仿真是否运行中
   */
  bool is_running() const;

  // ========== 状态访问 ==========

  /**
   * @brief 获取当前世界状态
   */
  const WorldState& get_world_state() const;

  /**
   * @brief 获取仿真时间
   */
  double get_simulation_time() const;

  /**
   * @brief 获取帧ID
   */
  uint64_t get_frame_id() const;

  // ========== 场景编辑 ==========

  /**
   * @brief 设置自车位置
   * @param pose 位姿
   */
  void set_ego_pose(const planning::Pose2d& pose);

  /**
   * @brief 设置自车速度
   * @param twist 速度
   */
  void set_ego_twist(const planning::Twist2d& twist);

  /**
   * @brief 直接应用自车状态（用于轨迹回放）
   */
  void apply_ego_state(const planning::Pose2d& pose, const planning::Twist2d& twist);

  /**
   * @brief 设置目标位置
   * @param pose 目标位姿
   */
  void set_goal_pose(const planning::Pose2d& pose);

  /**
   * @brief 设置目标容差
   * @param pos_tol 位置容差 (m)
   * @param yaw_tol 朝向容差 (rad)
   */
  void set_goal_tolerance(double pos_tol, double yaw_tol);

  /**
   * @brief 添加静态障碍物
   * @param obstacle 静态障碍物
   */
  void add_static_obstacle(const StaticObstacle& obstacle);

  /**
   * @brief 添加动态障碍物
   * @param obstacle 动态障碍物
   */
  void add_dynamic_obstacle(const DynamicObstacle& obstacle);

  /**
   * @brief 移除动态障碍物
   * @param id 障碍物ID
   */
  void remove_dynamic_obstacle(const std::string& id);

  /**
   * @brief 清空所有障碍物
   */
  void clear_obstacles();

  /**
   * @brief 清空静态障碍物
   */
  void clear_static_obstacles();

  /**
   * @brief 清空动态障碍物
   */
  void clear_dynamic_obstacles();

  // ========== 时间控制 ==========

  /**
   * @brief 设置时间缩放
   * @param scale 时间缩放 (1.0=实时, >1.0=加速, <1.0=减速)
   */
  void set_time_scale(double scale);

  /**
   * @brief 获取时间缩放
   */
  double get_time_scale() const;

  // ========== 回调设置 ==========

  /**
   * @brief 设置仿真状态变化回调
   * @param callback 回调函数
   */
  void set_simulation_state_callback(const SimulationStateCallback& callback);

  /**
   * @brief 设置帧更新回调
   * @param callback 回调函数
   */
  void set_frame_update_callback(const FrameUpdateCallback& callback);

  /**
   * @brief 设置碰撞回调
   * @param callback 回调函数
   */
  void set_collision_callback(const CollisionCallback& callback);

  // ========== 碰撞检测 ==========

  /**
   * @brief 检查自车是否与障碍物碰撞
   * @return 是否碰撞
   */
  bool check_collision() const;

  /**
   * @brief 检查指定位置是否与障碍物碰撞
   * @param pose 要检查的位姿
   * @return 是否碰撞
   */
  bool check_collision_at(const planning::Pose2d& pose) const;

  // ========== 转换到 Planning Context ==========

  /**
   * @brief 转换为规划上下文
   * @return 规划上下文
   */
  planning::PlanningContext to_planning_context() const;

  /**
   * @brief 从 protobuf WorldTick 更新状态
   * @param world_tick protobuf世界状态
   */
  void from_world_tick(const proto::WorldTick& world_tick);

  /**
   * @brief 转换为 protobuf WorldTick
   * @return protobuf世界状态
   */
  proto::WorldTick to_world_tick() const;

private:
  // 使用 Pimpl 模式隐藏实现细节
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace sim
} // namespace navsim