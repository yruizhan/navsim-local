/**
 * @file test_local_simulator.cpp
 * @brief LocalSimulator 基础测试
 */

#include "sim/local_simulator.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <nlohmann/json.hpp>

using namespace navsim::sim;
using namespace navsim::planning;

class LocalSimulatorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // 创建基础配置
    config_.time_step = 0.01;
    config_.time_scale = 1.0;
    config_.enable_adaptive_stepping = false;
    config_.physics.collision_detection = true;

    simulator_ = std::make_unique<LocalSimulator>();
  }

  void TearDown() override {
    simulator_.reset();
  }

  // 创建简单测试场景JSON
  std::string create_test_scenario() {
    nlohmann::json scenario = {
      {"scenario_name", "test_scenario"},
      {"timestamp", 0.0},
      {"ego", {
        {"pose", {{"x", 0.0}, {"y", 0.0}, {"yaw", 0.0}}},
        {"twist", {{"vx", 0.0}, {"vy", 0.0}, {"omega", 0.0}}},
        {"chassis_model", "differential"}
      }},
      {"task", {
        {"goal_pose", {{"x", 10.0}, {"y", 0.0}, {"yaw", 0.0}}},
        {"type", "GOTO_GOAL"}
      }},
      {"obstacles", {
        {
          {"type", "circle"},
          {"center", {{"x", 5.0}, {"y", 0.0}}},
          {"radius", 1.0}
        }
      }},
      {"dynamic_obstacles", {
        {
          {"id", "dyn1"},
          {"type", "vehicle"},
          {"shape_type", "circle"},
          {"current_pose", {{"x", 8.0}, {"y", 0.0}, {"yaw", 0.0}}},
          {"current_twist", {{"vx", 1.0}, {"vy", 0.0}, {"omega", 0.0}}},
          {"shape", {{"radius", 0.5}}},
          {"model", "cv"}
        }
      }}
    };

    std::string filename = "/tmp/test_scenario.json";
    std::ofstream file(filename);
    file << scenario.dump(2);
    file.close();

    return filename;
  }

  SimulatorConfig config_;
  std::unique_ptr<LocalSimulator> simulator_;
};

// ========== 基础功能测试 ==========

TEST_F(LocalSimulatorTest, InitializationTest) {
  EXPECT_TRUE(simulator_->initialize(config_));
  EXPECT_FALSE(simulator_->is_running());
  EXPECT_EQ(simulator_->get_simulation_time(), 0.0);
  EXPECT_EQ(simulator_->get_frame_id(), 0);
}

TEST_F(LocalSimulatorTest, ConfigurationTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  const auto& loaded_config = simulator_->get_config();
  EXPECT_EQ(loaded_config.time_step, config_.time_step);
  EXPECT_EQ(loaded_config.time_scale, config_.time_scale);
  EXPECT_EQ(loaded_config.enable_adaptive_stepping, config_.enable_adaptive_stepping);
}

TEST_F(LocalSimulatorTest, SimulationControlTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  // 测试启动
  simulator_->start();
  EXPECT_TRUE(simulator_->is_running());

  // 测试暂停
  simulator_->pause();
  EXPECT_FALSE(simulator_->is_running());

  // 测试重置
  simulator_->reset();
  EXPECT_FALSE(simulator_->is_running());
  EXPECT_EQ(simulator_->get_simulation_time(), 0.0);
}

// ========== 场景加载测试 ==========

TEST_F(LocalSimulatorTest, ScenarioLoadingTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  std::string scenario_file = create_test_scenario();
  EXPECT_TRUE(simulator_->load_scenario(scenario_file));

  const auto& world = simulator_->get_world_state();

  // 检查自车位置
  EXPECT_EQ(world.ego_pose.x, 0.0);
  EXPECT_EQ(world.ego_pose.y, 0.0);
  EXPECT_EQ(world.ego_pose.yaw, 0.0);

  // 检查目标位置
  EXPECT_EQ(world.goal_pose.x, 10.0);
  EXPECT_EQ(world.goal_pose.y, 0.0);
  EXPECT_EQ(world.goal_pose.yaw, 0.0);

  // 检查动态障碍物
  EXPECT_EQ(world.dynamic_obstacles.size(), 1);
  if (!world.dynamic_obstacles.empty()) {
    const auto& dyn_obs = world.dynamic_obstacles[0];
    EXPECT_EQ(dyn_obs.id, "dyn1");
    EXPECT_EQ(dyn_obs.pose.x, 8.0);
    EXPECT_EQ(dyn_obs.pose.y, 0.0);
    EXPECT_EQ(dyn_obs.twist.vx, 1.0);
  }

  // 清理
  std::remove(scenario_file.c_str());
}

// ========== 仿真步进测试 ==========

TEST_F(LocalSimulatorTest, SimulationSteppingTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  // 添加一个动态障碍物
  navsim::sim::DynamicObstacle obs;
  obs.id = "test_obs";
  obs.pose = {0.0, 0.0, 0.0};
  obs.twist = {1.0, 0.0, 0.0};  // 1 m/s 向前
  obs.model = "cv";
  simulator_->add_dynamic_obstacle(obs);

  simulator_->start();

  uint64_t initial_frame = simulator_->get_frame_id();
  double initial_time = simulator_->get_simulation_time();

  // 执行10步仿真
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(simulator_->step());
  }

  // 检查时间推进
  EXPECT_GT(simulator_->get_frame_id(), initial_frame);
  EXPECT_GT(simulator_->get_simulation_time(), initial_time);

  // 检查动态障碍物是否移动
  const auto& world = simulator_->get_world_state();
  EXPECT_EQ(world.dynamic_obstacles.size(), 1);
  if (!world.dynamic_obstacles.empty()) {
    const auto& moved_obs = world.dynamic_obstacles[0];
    EXPECT_GT(moved_obs.pose.x, 0.0);  // 应该向前移动了
  }
}

TEST_F(LocalSimulatorTest, PausedSimulationTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  // 添加一个动态障碍物
  navsim::sim::DynamicObstacle obs;
  obs.id = "test_obs";
  obs.pose = {0.0, 0.0, 0.0};
  obs.twist = {1.0, 0.0, 0.0};
  obs.model = "cv";
  simulator_->add_dynamic_obstacle(obs);

  // 保持暂停状态
  EXPECT_FALSE(simulator_->is_running());

  double initial_x = obs.pose.x;
  double initial_time = simulator_->get_simulation_time();

  // 执行几步仿真（暂停状态）
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(simulator_->step());
  }

  // 检查时间和位置没有变化（因为暂停）
  const auto& world = simulator_->get_world_state();
  EXPECT_EQ(simulator_->get_simulation_time(), initial_time);
  if (!world.dynamic_obstacles.empty()) {
    EXPECT_EQ(world.dynamic_obstacles[0].pose.x, initial_x);
  }
}

// ========== 碰撞检测测试 ==========

TEST_F(LocalSimulatorTest, CollisionDetectionTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  // 设置自车位置
  simulator_->set_ego_pose({0.0, 0.0, 0.0});

  // 添加一个距离较远的障碍物
  StaticObstacle far_obs({5.0, 0.0}, 0.5);
  simulator_->add_static_obstacle(far_obs);
  EXPECT_FALSE(simulator_->check_collision());

  // 添加一个重叠的障碍物
  StaticObstacle close_obs({0.0, 0.0}, 1.0);
  simulator_->add_static_obstacle(close_obs);
  EXPECT_TRUE(simulator_->check_collision());

  // 测试特定位置的碰撞检测
  EXPECT_FALSE(simulator_->check_collision_at({10.0, 10.0, 0.0}));  // 远离障碍物
  EXPECT_TRUE(simulator_->check_collision_at({0.5, 0.0, 0.0}));     // 接近障碍物
}

// ========== 时间缩放测试 ==========

TEST_F(LocalSimulatorTest, TimeScaleTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  // 测试时间缩放设置
  simulator_->set_time_scale(2.0);
  EXPECT_EQ(simulator_->get_time_scale(), 2.0);

  // 测试最小缩放限制
  simulator_->set_time_scale(0.001);
  EXPECT_GE(simulator_->get_time_scale(), 0.01);
}

// ========== 障碍物操作测试 ==========

TEST_F(LocalSimulatorTest, ObstacleManagementTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  // 添加静态障碍物
  StaticObstacle static_obs({1.0, 1.0}, 0.5);
  simulator_->add_static_obstacle(static_obs);

  // 添加动态障碍物
  navsim::sim::DynamicObstacle dyn_obs1("obs1");
  dyn_obs1.pose = {2.0, 2.0, 0.0};
  simulator_->add_dynamic_obstacle(dyn_obs1);

  navsim::sim::DynamicObstacle dyn_obs2("obs2");
  dyn_obs2.pose = {3.0, 3.0, 0.0};
  simulator_->add_dynamic_obstacle(dyn_obs2);

  const auto& world = simulator_->get_world_state();
  EXPECT_EQ(world.static_obstacles.size(), 1);
  EXPECT_EQ(world.dynamic_obstacles.size(), 2);

  // 移除特定动态障碍物
  simulator_->remove_dynamic_obstacle("obs1");
  EXPECT_EQ(simulator_->get_world_state().dynamic_obstacles.size(), 1);

  // 清空所有障碍物
  simulator_->clear_obstacles();
  EXPECT_EQ(simulator_->get_world_state().static_obstacles.size(), 0);
  EXPECT_EQ(simulator_->get_world_state().dynamic_obstacles.size(), 0);
}

// ========== 状态转换测试 ==========

TEST_F(LocalSimulatorTest, StateConversionTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  // 设置测试状态
  simulator_->set_ego_pose({1.0, 2.0, 0.5});
  simulator_->set_goal_pose({5.0, 6.0, 1.0});

  // 转换为 PlanningContext
  auto context = simulator_->to_planning_context();
  EXPECT_EQ(context.ego.pose.x, 1.0);
  EXPECT_EQ(context.ego.pose.y, 2.0);
  EXPECT_EQ(context.ego.pose.yaw, 0.5);
  EXPECT_EQ(context.task.goal_pose.x, 5.0);
  EXPECT_EQ(context.task.goal_pose.y, 6.0);
  EXPECT_EQ(context.task.goal_pose.yaw, 1.0);

  // 转换为 WorldTick protobuf
  auto world_tick = simulator_->to_world_tick();
  EXPECT_EQ(world_tick.ego().pose().x(), 1.0);
  EXPECT_EQ(world_tick.ego().pose().y(), 2.0);
  EXPECT_EQ(world_tick.ego().pose().yaw(), 0.5);
  EXPECT_EQ(world_tick.goal().pose().x(), 5.0);
  EXPECT_EQ(world_tick.goal().pose().y(), 6.0);
  EXPECT_EQ(world_tick.goal().pose().yaw(), 1.0);
}

// ========== 回调函数测试 ==========

TEST_F(LocalSimulatorTest, CallbackTest) {
  EXPECT_TRUE(simulator_->initialize(config_));

  bool state_callback_called = false;
  bool frame_callback_called = false;

  // 设置回调
  simulator_->set_simulation_state_callback([&](bool running) {
    state_callback_called = true;
    EXPECT_TRUE(running);
  });

  simulator_->set_frame_update_callback([&](const WorldState& world) {
    frame_callback_called = true;
    EXPECT_GE(world.frame_id, 0);
  });

  // 触发回调
  simulator_->start();
  EXPECT_TRUE(state_callback_called);

  simulator_->step();
  EXPECT_TRUE(frame_callback_called);
}

// ========== 主函数 ==========

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}