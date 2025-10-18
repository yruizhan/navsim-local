#include "core/scenario_loader.hpp"
#include <fstream>
#include <iostream>

namespace navsim {
namespace planning {

// ========== 公共接口 ==========

bool ScenarioLoader::loadFromFile(const std::string& file_path, PlanningContext& context) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "[ScenarioLoader] Failed to open file: " << file_path << std::endl;
    return false;
  }

  nlohmann::json json;
  try {
    file >> json;
  } catch (const nlohmann::json::exception& e) {
    std::cerr << "[ScenarioLoader] JSON parse error: " << e.what() << std::endl;
    return false;
  }

  return loadFromJson(json, context);
}

bool ScenarioLoader::loadFromString(const std::string& json_str, PlanningContext& context) {
  nlohmann::json json;
  try {
    json = nlohmann::json::parse(json_str);
  } catch (const nlohmann::json::exception& e) {
    std::cerr << "[ScenarioLoader] JSON parse error: " << e.what() << std::endl;
    return false;
  }

  return loadFromJson(json, context);
}

bool ScenarioLoader::loadFromJson(const nlohmann::json& json, PlanningContext& context) {
  try {
    // 解析时间戳
    if (json.contains("timestamp")) {
      context.timestamp = json["timestamp"].get<double>();
    }

    // 解析规划时域
    if (json.contains("planning_horizon")) {
      context.planning_horizon = json["planning_horizon"].get<double>();
    }

    // 解析自车状态
    if (json.contains("ego")) {
      if (!parseEgo(json["ego"], context.ego)) {
        std::cerr << "[ScenarioLoader] Failed to parse ego vehicle" << std::endl;
        return false;
      }
    }

    // 解析任务目标
    if (json.contains("task")) {
      if (!parseTask(json["task"], context.task)) {
        std::cerr << "[ScenarioLoader] Failed to parse task" << std::endl;
        return false;
      }
    }

    // 解析静态障碍物
    if (json.contains("obstacles")) {
      if (!parseObstacles(json["obstacles"], context)) {
        std::cerr << "[ScenarioLoader] Failed to parse obstacles" << std::endl;
        return false;
      }
    }

    // 解析动态障碍物
    if (json.contains("dynamic_obstacles")) {
      if (!parseDynamicObstacles(json["dynamic_obstacles"], context)) {
        std::cerr << "[ScenarioLoader] Failed to parse dynamic obstacles" << std::endl;
        return false;
      }
    }

    std::cout << "[ScenarioLoader] Successfully loaded scenario" << std::endl;
    return true;

  } catch (const nlohmann::json::exception& e) {
    std::cerr << "[ScenarioLoader] JSON error: " << e.what() << std::endl;
    return false;
  }
}

bool ScenarioLoader::saveToFile(const std::string& file_path, const PlanningContext& context) {
  nlohmann::json json = toJson(context);

  std::ofstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "[ScenarioLoader] Failed to open file for writing: " << file_path << std::endl;
    return false;
  }

  file << json.dump(2);  // 2-space indentation
  std::cout << "[ScenarioLoader] Successfully saved scenario to: " << file_path << std::endl;
  return true;
}

nlohmann::json ScenarioLoader::toJson(const PlanningContext& context) {
  nlohmann::json json;

  json["timestamp"] = context.timestamp;
  json["planning_horizon"] = context.planning_horizon;
  json["ego"] = egoToJson(context.ego);
  json["task"] = taskToJson(context.task);
  json["obstacles"] = obstaclesToJson(context);
  json["dynamic_obstacles"] = dynamicObstaclesToJson(context);

  return json;
}

// ========== 基础类型转换 ==========

Point2d ScenarioLoader::parsePoint2d(const nlohmann::json& json) {
  Point2d point;
  point.x = json.value("x", 0.0);
  point.y = json.value("y", 0.0);
  return point;
}

Pose2d ScenarioLoader::parsePose2d(const nlohmann::json& json) {
  Pose2d pose;
  pose.x = json.value("x", 0.0);
  pose.y = json.value("y", 0.0);
  pose.yaw = json.value("yaw", 0.0);
  return pose;
}

Twist2d ScenarioLoader::parseTwist2d(const nlohmann::json& json) {
  Twist2d twist;
  twist.vx = json.value("vx", 0.0);
  twist.vy = json.value("vy", 0.0);
  twist.omega = json.value("omega", 0.0);
  return twist;
}

nlohmann::json ScenarioLoader::point2dToJson(const Point2d& point) {
  return {{"x", point.x}, {"y", point.y}};
}

nlohmann::json ScenarioLoader::pose2dToJson(const Pose2d& pose) {
  return {{"x", pose.x}, {"y", pose.y}, {"yaw", pose.yaw}};
}

nlohmann::json ScenarioLoader::twist2dToJson(const Twist2d& twist) {
  return {{"vx", twist.vx}, {"vy", twist.vy}, {"omega", twist.omega}};
}

// ========== Ego 解析 ==========

bool ScenarioLoader::parseEgo(const nlohmann::json& json, EgoVehicle& ego) {
  if (json.contains("pose")) {
    ego.pose = parsePose2d(json["pose"]);
  }

  if (json.contains("twist")) {
    ego.twist = parseTwist2d(json["twist"]);
  }

  if (json.contains("timestamp")) {
    ego.timestamp = json["timestamp"].get<double>();
  }

  if (json.contains("chassis_model")) {
    ego.chassis_model = json["chassis_model"].get<std::string>();
  }

  // 解析运动学参数
  if (json.contains("kinematics")) {
    const auto& kin = json["kinematics"];
    ego.kinematics.wheelbase = kin.value("wheelbase", 2.8);
    ego.kinematics.track_width = kin.value("track_width", 2.0);
    ego.kinematics.front_overhang = kin.value("front_overhang", 1.0);
    ego.kinematics.rear_overhang = kin.value("rear_overhang", 1.0);
    ego.kinematics.width = kin.value("width", 2.0);
    ego.kinematics.height = kin.value("height", 1.8);
    ego.kinematics.body_length = kin.value("body_length", 4.8);
    ego.kinematics.body_width = kin.value("body_width", 2.0);
    ego.kinematics.wheel_radius = kin.value("wheel_radius", 0.3);
  }

  // 解析动力学约束
  if (json.contains("limits")) {
    const auto& lim = json["limits"];
    ego.limits.max_velocity = lim.value("max_velocity", 15.0);
    ego.limits.max_acceleration = lim.value("max_acceleration", 3.0);
    ego.limits.max_deceleration = lim.value("max_deceleration", 8.0);
    ego.limits.max_steer_angle = lim.value("max_steer_angle", 0.6);
    ego.limits.max_steer_rate = lim.value("max_steer_rate", 1.0);
    ego.limits.max_jerk = lim.value("max_jerk", 3.0);
    ego.limits.max_curvature = lim.value("max_curvature", 0.2);
  }

  return true;
}

nlohmann::json ScenarioLoader::egoToJson(const EgoVehicle& ego) {
  nlohmann::json json;
  json["pose"] = pose2dToJson(ego.pose);
  json["twist"] = twist2dToJson(ego.twist);
  json["timestamp"] = ego.timestamp;
  json["chassis_model"] = ego.chassis_model;

  json["kinematics"] = {
    {"wheelbase", ego.kinematics.wheelbase},
    {"track_width", ego.kinematics.track_width},
    {"front_overhang", ego.kinematics.front_overhang},
    {"rear_overhang", ego.kinematics.rear_overhang},
    {"width", ego.kinematics.width},
    {"height", ego.kinematics.height},
    {"body_length", ego.kinematics.body_length},
    {"body_width", ego.kinematics.body_width},
    {"wheel_radius", ego.kinematics.wheel_radius}
  };

  json["limits"] = {
    {"max_velocity", ego.limits.max_velocity},
    {"max_acceleration", ego.limits.max_acceleration},
    {"max_deceleration", ego.limits.max_deceleration},
    {"max_steer_angle", ego.limits.max_steer_angle},
    {"max_steer_rate", ego.limits.max_steer_rate},
    {"max_jerk", ego.limits.max_jerk},
    {"max_curvature", ego.limits.max_curvature}
  };

  return json;
}

// ========== Task 解析 ==========

bool ScenarioLoader::parseTask(const nlohmann::json& json, PlanningTask& task) {
  if (json.contains("goal_pose")) {
    task.goal_pose = parsePose2d(json["goal_pose"]);
  }

  if (json.contains("type")) {
    std::string type_str = json["type"].get<std::string>();
    if (type_str == "GOTO_GOAL") {
      task.type = PlanningTask::Type::GOTO_GOAL;
    } else if (type_str == "LANE_FOLLOWING") {
      task.type = PlanningTask::Type::LANE_FOLLOWING;
    } else if (type_str == "LANE_CHANGE") {
      task.type = PlanningTask::Type::LANE_CHANGE;
    } else if (type_str == "PARKING") {
      task.type = PlanningTask::Type::PARKING;
    } else if (type_str == "EMERGENCY_STOP") {
      task.type = PlanningTask::Type::EMERGENCY_STOP;
    }
  }

  if (json.contains("tolerance")) {
    const auto& tol = json["tolerance"];
    task.tolerance.position = tol.value("position", 0.5);
    task.tolerance.yaw = tol.value("yaw", 0.2);
  }

  return true;
}

nlohmann::json ScenarioLoader::taskToJson(const PlanningTask& task) {
  nlohmann::json json;
  json["goal_pose"] = pose2dToJson(task.goal_pose);

  // 转换任务类型
  std::string type_str;
  switch (task.type) {
    case PlanningTask::Type::GOTO_GOAL:
      type_str = "GOTO_GOAL";
      break;
    case PlanningTask::Type::LANE_FOLLOWING:
      type_str = "LANE_FOLLOWING";
      break;
    case PlanningTask::Type::LANE_CHANGE:
      type_str = "LANE_CHANGE";
      break;
    case PlanningTask::Type::PARKING:
      type_str = "PARKING";
      break;
    case PlanningTask::Type::EMERGENCY_STOP:
      type_str = "EMERGENCY_STOP";
      break;
  }
  json["type"] = type_str;

  json["tolerance"] = {
    {"position", task.tolerance.position},
    {"yaw", task.tolerance.yaw}
  };

  return json;
}

// ========== 静态障碍物解析 ==========

bool ScenarioLoader::parseObstacles(const nlohmann::json& json, PlanningContext& context) {
  if (!json.is_array()) {
    std::cerr << "[ScenarioLoader] Obstacles must be an array" << std::endl;
    return false;
  }

  // 创建 BEV 障碍物容器
  if (!context.bev_obstacles) {
    context.bev_obstacles = std::make_unique<BEVObstacles>();
  }

  for (const auto& obs : json) {
    std::string type = obs.value("type", "");

    if (type == "circle") {
      BEVObstacles::Circle circle;
      circle.center = parsePoint2d(obs["center"]);
      circle.radius = obs.value("radius", 1.0);
      circle.confidence = obs.value("confidence", 1.0);
      context.bev_obstacles->circles.push_back(circle);

    } else if (type == "rectangle") {
      BEVObstacles::Rectangle rect;
      rect.pose = parsePose2d(obs["pose"]);
      rect.width = obs.value("width", 2.0);
      rect.height = obs.value("height", 1.0);
      rect.confidence = obs.value("confidence", 1.0);
      context.bev_obstacles->rectangles.push_back(rect);

    } else if (type == "polygon") {
      BEVObstacles::Polygon poly;
      if (obs.contains("vertices") && obs["vertices"].is_array()) {
        for (const auto& vertex : obs["vertices"]) {
          poly.vertices.push_back(parsePoint2d(vertex));
        }
      }
      poly.confidence = obs.value("confidence", 1.0);
      context.bev_obstacles->polygons.push_back(poly);

    } else {
      std::cerr << "[ScenarioLoader] Unknown obstacle type: " << type << std::endl;
    }
  }

  return true;
}

nlohmann::json ScenarioLoader::obstaclesToJson(const PlanningContext& context) {
  nlohmann::json json = nlohmann::json::array();

  if (!context.bev_obstacles) {
    return json;
  }

  // 圆形障碍物
  for (const auto& circle : context.bev_obstacles->circles) {
    nlohmann::json obs;
    obs["type"] = "circle";
    obs["center"] = point2dToJson(circle.center);
    obs["radius"] = circle.radius;
    obs["confidence"] = circle.confidence;
    json.push_back(obs);
  }

  // 矩形障碍物
  for (const auto& rect : context.bev_obstacles->rectangles) {
    nlohmann::json obs;
    obs["type"] = "rectangle";
    obs["pose"] = pose2dToJson(rect.pose);
    obs["width"] = rect.width;
    obs["height"] = rect.height;
    obs["confidence"] = rect.confidence;
    json.push_back(obs);
  }

  // 多边形障碍物
  for (const auto& poly : context.bev_obstacles->polygons) {
    nlohmann::json obs;
    obs["type"] = "polygon";
    obs["vertices"] = nlohmann::json::array();
    for (const auto& vertex : poly.vertices) {
      obs["vertices"].push_back(point2dToJson(vertex));
    }
    obs["confidence"] = poly.confidence;
    json.push_back(obs);
  }

  return json;
}

// ========== 动态障碍物解析 ==========

bool ScenarioLoader::parseDynamicObstacles(const nlohmann::json& json, PlanningContext& context) {
  if (!json.is_array()) {
    std::cerr << "[ScenarioLoader] Dynamic obstacles must be an array" << std::endl;
    return false;
  }

  for (const auto& obs : json) {
    DynamicObstacle dyn_obs;

    dyn_obs.id = obs.value("id", 0);
    dyn_obs.type = obs.value("type", "vehicle");
    dyn_obs.shape_type = obs.value("shape_type", "rectangle");

    if (obs.contains("current_pose")) {
      dyn_obs.current_pose = parsePose2d(obs["current_pose"]);
    }

    if (obs.contains("current_twist")) {
      dyn_obs.current_twist = parseTwist2d(obs["current_twist"]);
    }

    dyn_obs.length = obs.value("length", 4.5);
    dyn_obs.width = obs.value("width", 2.0);
    dyn_obs.height = obs.value("height", 1.8);

    // 解析预测轨迹（如果有）
    if (obs.contains("predicted_trajectories") && obs["predicted_trajectories"].is_array()) {
      for (const auto& traj_json : obs["predicted_trajectories"]) {
        DynamicObstacle::Trajectory traj;
        traj.probability = traj_json.value("probability", 1.0);

        if (traj_json.contains("poses") && traj_json["poses"].is_array()) {
          for (const auto& pose_json : traj_json["poses"]) {
            traj.poses.push_back(parsePose2d(pose_json));
          }
        }

        if (traj_json.contains("timestamps") && traj_json["timestamps"].is_array()) {
          for (const auto& ts : traj_json["timestamps"]) {
            traj.timestamps.push_back(ts.get<double>());
          }
        }

        dyn_obs.predicted_trajectories.push_back(traj);
      }
    }

    context.dynamic_obstacles.push_back(dyn_obs);
  }

  return true;
}

nlohmann::json ScenarioLoader::dynamicObstaclesToJson(const PlanningContext& context) {
  nlohmann::json json = nlohmann::json::array();

  for (const auto& dyn_obs : context.dynamic_obstacles) {
    nlohmann::json obs;
    obs["id"] = dyn_obs.id;
    obs["type"] = dyn_obs.type;
    obs["shape_type"] = dyn_obs.shape_type;
    obs["current_pose"] = pose2dToJson(dyn_obs.current_pose);
    obs["current_twist"] = twist2dToJson(dyn_obs.current_twist);
    obs["length"] = dyn_obs.length;
    obs["width"] = dyn_obs.width;
    obs["height"] = dyn_obs.height;

    // 序列化预测轨迹
    obs["predicted_trajectories"] = nlohmann::json::array();
    for (const auto& traj : dyn_obs.predicted_trajectories) {
      nlohmann::json traj_json;
      traj_json["probability"] = traj.probability;

      traj_json["poses"] = nlohmann::json::array();
      for (const auto& pose : traj.poses) {
        traj_json["poses"].push_back(pose2dToJson(pose));
      }

      traj_json["timestamps"] = traj.timestamps;

      obs["predicted_trajectories"].push_back(traj_json);
    }

    json.push_back(obs);
  }

  return json;
}

} // namespace planning
} // namespace navsim

