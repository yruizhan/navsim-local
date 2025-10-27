#include "control/trajectory_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>

namespace navsim {
namespace control {

TrajectoryTracker::TrajectoryTracker() {
    config_ = Config{};
    reset();
}

TrajectoryTracker::TrajectoryTracker(const Config& config)
    : config_(config) {
    reset();
}

void TrajectoryTracker::setConfig(const Config& config) {
    config_ = config;
    reset();
}

void TrajectoryTracker::setTrajectory(const std::vector<plugin::TrajectoryPoint>& trajectory) {
    trajectory_ = trajectory;
    has_trajectory_ = !trajectory_.empty();

    if (has_trajectory_) {
        // 验证轨迹时间单调性
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            if (trajectory_[i].time_from_start <= trajectory_[i-1].time_from_start) {
                std::cerr << "[TrajectoryTracker] Warning: Non-monotonic trajectory times at index "
                         << i << std::endl;
            }
        }

        std::cout << "[TrajectoryTracker] Loaded trajectory with " << trajectory_.size()
                  << " points, duration: " << getTrajectoryDuration() << "s" << std::endl;
    }

    reset();
}

void TrajectoryTracker::setTrajectoryFromProto(const proto::PlanUpdate& plan_update) {
    std::vector<plugin::TrajectoryPoint> trajectory;
    trajectory.reserve(plan_update.trajectory_size());

    for (const auto& proto_point : plan_update.trajectory()) {
        plugin::TrajectoryPoint point;

        // 位置和朝向
        point.pose.x = proto_point.x();
        point.pose.y = proto_point.y();
        point.pose.yaw = proto_point.yaw();

        // 速度
        point.twist.vx = proto_point.vx();
        point.twist.vy = proto_point.vy();
        point.twist.omega = proto_point.omega();

        // 其他属性
        point.acceleration = proto_point.acceleration();
        point.curvature = proto_point.curvature();
        point.time_from_start = proto_point.t();
        point.path_length = proto_point.path_length();

        trajectory.push_back(point);
    }

    setTrajectory(trajectory);
}

bool TrajectoryTracker::hasValidTrajectory() const {
    return has_trajectory_ && trajectory_.size() >= 2;
}

double TrajectoryTracker::getTrajectoryDuration() const {
    if (!hasValidTrajectory()) return 0.0;
    return trajectory_.back().time_from_start;
}

planning::Twist2d TrajectoryTracker::getControlCommand(double sim_time) {
    if (!hasValidTrajectory()) {
        return planning::Twist2d{};  // 零速度
    }

    planning::Twist2d command;

    switch (config_.mode) {
        case TrackingMode::TIME_SYNC:
            command = interpolateVelocity(sim_time);
            break;

        case TrackingMode::LOOKAHEAD:
            command = lookaheadControl(sim_time);
            break;

        case TrackingMode::PREDICTIVE:
            // 预测控制需要额外的位姿信息，这里简化为前瞻控制
            command = lookaheadControl(sim_time);
            break;

        case TrackingMode::HYBRID:
        default:
            // 混合策略：近距离用时间同步，远距离用前瞻控制
            if (sim_time < config_.lookahead_time) {
                command = interpolateVelocity(sim_time);
            } else {
                command = lookaheadControl(sim_time);
            }
            break;
    }

    // 应用平滑滤波
    command = applySmoothingFilter(command);

    // 应用动力学约束
    command = applyDynamicConstraints(command);

    // 更新历史记录
    last_command_ = command;
    command_history_.push_back(command);
    if (command_history_.size() > 100) {  // 保持最近100个命令
        command_history_.erase(command_history_.begin());
    }

    return command;
}

plugin::TrajectoryPoint TrajectoryTracker::getTargetState(double sim_time) const {
    if (!hasValidTrajectory()) {
        return plugin::TrajectoryPoint{};
    }

    // 边界处理
    if (sim_time <= trajectory_.front().time_from_start) {
        return trajectory_.front();
    }
    if (sim_time >= trajectory_.back().time_from_start) {
        return trajectory_.back();
    }

    // 查找周围的点
    auto indices = findSurroundingIndices(sim_time);
    size_t prev_idx = indices.first;
    size_t next_idx = indices.second;

    if (prev_idx == next_idx) {
        return trajectory_[prev_idx];
    }

    // 线性插值
    const auto& p1 = trajectory_[prev_idx];
    const auto& p2 = trajectory_[next_idx];

    double dt = p2.time_from_start - p1.time_from_start;
    if (dt < 1e-6) return p1;

    double ratio = (sim_time - p1.time_from_start) / dt;
    ratio = std::clamp(ratio, 0.0, 1.0);

    plugin::TrajectoryPoint result;

    // 插值位置
    result.pose.x = p1.pose.x + ratio * (p2.pose.x - p1.pose.x);
    result.pose.y = p1.pose.y + ratio * (p2.pose.y - p1.pose.y);
    result.pose.yaw = p1.pose.yaw + ratio * normalizeAngle(p2.pose.yaw - p1.pose.yaw);

    // 插值速度
    result.twist.vx = p1.twist.vx + ratio * (p2.twist.vx - p1.twist.vx);
    result.twist.vy = p1.twist.vy + ratio * (p2.twist.vy - p1.twist.vy);
    result.twist.omega = p1.twist.omega + ratio * (p2.twist.omega - p1.twist.omega);

    // 插值其他属性
    result.acceleration = p1.acceleration + ratio * (p2.acceleration - p1.acceleration);
    result.curvature = p1.curvature + ratio * (p2.curvature - p1.curvature);
    result.time_from_start = sim_time;
    result.path_length = p1.path_length + ratio * (p2.path_length - p1.path_length);

    return result;
}

planning::Twist2d TrajectoryTracker::getPredictiveControl(
    const planning::Pose2d& current_pose,
    const planning::Twist2d& current_twist,
    double sim_time) {

    if (!hasValidTrajectory()) {
        return planning::Twist2d{};
    }

    // 查找最近的轨迹点
    size_t closest_idx = findClosestTrajectoryPoint(current_pose);

    // 计算前瞻目标点
    auto target_point = calculateLookaheadTarget(current_pose, sim_time);

    // 计算到目标点的控制指令
    planning::Twist2d command;

    // 简化的控制律：基于位置误差的比例控制
    double dx = target_point.pose.x - current_pose.x;
    double dy = target_point.pose.y - current_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > 1e-3) {
        // 计算期望的移动方向
        double target_angle = std::atan2(dy, dx);

        // 角度误差
        double angle_error = normalizeAngle(target_angle - current_pose.yaw);

        // 控制律
        double speed_gain = 2.0;
        double angular_gain = 3.0;

        command.vx = std::min(speed_gain * distance, target_point.twist.vx);
        command.omega = angular_gain * angle_error;

        // 限制角速度
        command.omega = std::clamp(command.omega, -config_.max_angular_velocity,
                                  config_.max_angular_velocity);
    } else {
        // 距离很近，直接使用目标速度
        command = target_point.twist;
    }

    return command;
}

void TrajectoryTracker::updateQualityAssessment(
    const planning::Pose2d& actual_pose,
    const planning::Twist2d& actual_twist,
    double sim_time) {

    if (!config_.enable_quality_assessment || !hasValidTrajectory()) {
        return;
    }

    // 获取目标状态
    auto target_state = getTargetState(sim_time);

    // 更新跟踪状态
    tracking_state_.current_time = sim_time;
    tracking_state_.target_point = target_state;
    tracking_state_.actual_point.pose = actual_pose;
    tracking_state_.actual_point.twist = actual_twist;
    tracking_state_.actual_point.time_from_start = sim_time;

    auto& quality = tracking_state_.quality;

    // 计算位置误差
    quality.position_error = calculatePositionError(actual_pose, target_state.pose);

    // 计算速度误差
    quality.velocity_error = calculateVelocityError(actual_twist, target_state.twist);

    // 计算航向误差
    quality.heading_error = std::abs(normalizeAngle(actual_pose.yaw - target_state.pose.yaw));

    // 检查动力学约束违反
    double actual_speed = std::sqrt(actual_twist.vx * actual_twist.vx +
                                   actual_twist.vy * actual_twist.vy);
    quality.velocity_limit_violated = (actual_speed > config_.max_velocity);
    quality.angular_velocity_limit_violated =
        (std::abs(actual_twist.omega) > config_.max_angular_velocity);

    // 计算jerk（如果有历史数据）
    if (command_history_.size() >= 2) {
        const auto& curr_cmd = command_history_.back();
        const auto& prev_cmd = command_history_[command_history_.size() - 2];

        double dt = 1.0 / 30.0;  // 假设30Hz控制频率
        quality.acceleration_jerk = std::abs(curr_cmd.vx - prev_cmd.vx) / dt;
        quality.angular_jerk = std::abs(curr_cmd.omega - prev_cmd.omega) / dt;
    }

    // 计算平滑度评分
    quality.smoothness_score = calculateSmoothnessScore();

    // 更新综合评分
    updateOverallScore();

    // 保存质量历史
    quality_history_.push_back(quality);
    if (quality_history_.size() > 300) {  // 保持最近10秒的数据（30Hz）
        quality_history_.erase(quality_history_.begin());
    }
}

void TrajectoryTracker::reset() {
    tracking_state_ = TrackingState{};
    last_command_ = planning::Twist2d{};
    command_history_.clear();
    quality_history_.clear();
    last_update_time_ = std::chrono::steady_clock::now();
    initialized_ = true;
}

bool TrajectoryTracker::isTrajectoryCompleted(double sim_time) const {
    if (!hasValidTrajectory()) return true;
    return sim_time >= trajectory_.back().time_from_start;
}

double TrajectoryTracker::getCompletionPercentage(double sim_time) const {
    if (!hasValidTrajectory()) return 100.0;

    double duration = getTrajectoryDuration();
    if (duration <= 0) return 100.0;

    return std::clamp(100.0 * sim_time / duration, 0.0, 100.0);
}

// ========== 私有方法实现 ==========

std::pair<size_t, size_t> TrajectoryTracker::findSurroundingIndices(double target_time) const {
    if (trajectory_.empty()) return {0, 0};

    // 二分查找
    auto it = std::lower_bound(trajectory_.begin(), trajectory_.end(), target_time,
        [](const plugin::TrajectoryPoint& point, double time) {
            return point.time_from_start < time;
        });

    size_t next_idx = std::distance(trajectory_.begin(), it);

    if (next_idx == 0) {
        return {0, std::min(size_t(1), trajectory_.size() - 1)};
    }
    if (next_idx >= trajectory_.size()) {
        return {trajectory_.size() - 1, trajectory_.size() - 1};
    }

    return {next_idx - 1, next_idx};
}

planning::Twist2d TrajectoryTracker::interpolateVelocity(double target_time) {
    auto target_state = getTargetState(target_time);
    return target_state.twist;
}

planning::Twist2d TrajectoryTracker::lookaheadControl(double sim_time) {
    // 查找前瞻时间后的目标点
    double lookahead_time = sim_time + config_.lookahead_time;
    auto target_state = getTargetState(lookahead_time);
    return target_state.twist;
}

size_t TrajectoryTracker::findClosestTrajectoryPoint(const planning::Pose2d& current_pose) const {
    if (trajectory_.empty()) return 0;

    size_t closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < trajectory_.size(); ++i) {
        double distance = calculatePositionError(current_pose, trajectory_[i].pose);
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }

    return closest_idx;
}

plugin::TrajectoryPoint TrajectoryTracker::calculateLookaheadTarget(
    const planning::Pose2d& current_pose, double sim_time) const {

    // 基于距离的前瞻
    size_t start_idx = findClosestTrajectoryPoint(current_pose);

    for (size_t i = start_idx; i < trajectory_.size(); ++i) {
        double distance = calculatePositionError(current_pose, trajectory_[i].pose);
        if (distance >= config_.lookahead_distance) {
            return trajectory_[i];
        }
    }

    // 如果没找到足够远的点，使用时间前瞻
    double lookahead_time = sim_time + config_.lookahead_time;
    return getTargetState(lookahead_time);
}

planning::Twist2d TrajectoryTracker::applySmoothingFilter(const planning::Twist2d& raw_command) {
    if (!initialized_ || command_history_.empty()) {
        return raw_command;
    }

    // 简单的低通滤波
    planning::Twist2d filtered;
    double alpha = config_.smoothing_factor;

    filtered.vx = alpha * raw_command.vx + (1.0 - alpha) * last_command_.vx;
    filtered.vy = alpha * raw_command.vy + (1.0 - alpha) * last_command_.vy;
    filtered.omega = alpha * raw_command.omega + (1.0 - alpha) * last_command_.omega;

    return filtered;
}

planning::Twist2d TrajectoryTracker::applyDynamicConstraints(const planning::Twist2d& command) {
    planning::Twist2d constrained = command;

    // 速度限制
    double speed = std::sqrt(constrained.vx * constrained.vx + constrained.vy * constrained.vy);
    if (speed > config_.max_velocity) {
        double scale = config_.max_velocity / speed;
        constrained.vx *= scale;
        constrained.vy *= scale;
    }

    // 角速度限制
    constrained.omega = std::clamp(constrained.omega,
                                  -config_.max_angular_velocity,
                                  config_.max_angular_velocity);

    // 加速度限制（如果有历史数据）
    if (!command_history_.empty()) {
        double dt = 1.0 / 30.0;  // 假设30Hz
        double max_dv = config_.max_acceleration * dt;

        double dv_x = constrained.vx - last_command_.vx;
        double dv_y = constrained.vy - last_command_.vy;
        double dv_magnitude = std::sqrt(dv_x * dv_x + dv_y * dv_y);

        if (dv_magnitude > max_dv) {
            double scale = max_dv / dv_magnitude;
            constrained.vx = last_command_.vx + dv_x * scale;
            constrained.vy = last_command_.vy + dv_y * scale;
        }
    }

    return constrained;
}

double TrajectoryTracker::calculatePositionError(
    const planning::Pose2d& actual, const planning::Pose2d& target) const {
    double dx = actual.x - target.x;
    double dy = actual.y - target.y;
    return std::sqrt(dx * dx + dy * dy);
}

double TrajectoryTracker::calculateVelocityError(
    const planning::Twist2d& actual, const planning::Twist2d& target) const {
    double dvx = actual.vx - target.vx;
    double dvy = actual.vy - target.vy;
    return std::sqrt(dvx * dvx + dvy * dvy);
}

double TrajectoryTracker::calculateSmoothnessScore() const {
    if (command_history_.size() < 10) return 50.0;  // 默认中等分数

    // 计算速度变化的标准差
    std::vector<double> velocity_changes;
    for (size_t i = 1; i < command_history_.size(); ++i) {
        double dv = calculateVelocityError(command_history_[i], command_history_[i-1]);
        velocity_changes.push_back(dv);
    }

    // 计算标准差
    double mean = std::accumulate(velocity_changes.begin(), velocity_changes.end(), 0.0)
                  / velocity_changes.size();

    double variance = 0.0;
    for (double change : velocity_changes) {
        variance += (change - mean) * (change - mean);
    }
    variance /= velocity_changes.size();
    double std_dev = std::sqrt(variance);

    // 转换为0-100分数（标准差越小，平滑度越高）
    double smoothness = std::max(0.0, 100.0 - std_dev * 50.0);
    return std::min(100.0, smoothness);
}

void TrajectoryTracker::updateOverallScore() {
    auto& quality = tracking_state_.quality;

    // 加权计算综合评分
    double accuracy_weight = 0.3;
    double smoothness_weight = 0.3;
    double safety_weight = 0.4;

    // 精度评分（位置和速度误差）
    double accuracy_score = std::max(0.0, 100.0 - quality.position_error * 20.0 -
                                           quality.velocity_error * 10.0);

    // 平滑度评分
    double smoothness_score = quality.smoothness_score;

    // 安全性评分（基于约束违反）
    double safety_score = 100.0;
    if (quality.velocity_limit_violated) safety_score -= 30.0;
    if (quality.acceleration_limit_violated) safety_score -= 30.0;
    if (quality.angular_velocity_limit_violated) safety_score -= 20.0;
    if (quality.collision_risk) safety_score -= 50.0;

    quality.overall_score = accuracy_weight * accuracy_score +
                           smoothness_weight * smoothness_score +
                           safety_weight * safety_score;

    quality.overall_score = std::clamp(quality.overall_score, 0.0, 100.0);
}

double TrajectoryTracker::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace control
} // namespace navsim