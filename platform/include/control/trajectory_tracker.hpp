#pragma once

#include "core/planning_context.hpp"
#include "plugin/data/planning_result.hpp"
#include "world_tick.pb.h"
#include "plan_update.pb.h"
#include <vector>
#include <memory>
#include <chrono>

namespace navsim {
namespace control {

/**
 * @brief 轨迹跟踪质量评估指标
 */
struct TrackingQualityMetrics {
    // 跟踪精度
    double position_error = 0.0;      // 位置误差 (m)
    double velocity_error = 0.0;      // 速度误差 (m/s)
    double heading_error = 0.0;       // 航向误差 (rad)

    // 运动平滑度
    double acceleration_jerk = 0.0;   // 加速度变化率 (m/s³)
    double angular_jerk = 0.0;        // 角加速度变化率 (rad/s³)
    double smoothness_score = 0.0;    // 平滑度评分 (0-100)

    // 动力学约束满足度
    bool velocity_limit_violated = false;
    bool acceleration_limit_violated = false;
    bool angular_velocity_limit_violated = false;

    // 安全性指标
    double min_obstacle_distance = std::numeric_limits<double>::max();
    bool collision_risk = false;

    // 效率指标
    double path_efficiency = 0.0;     // 路径效率 (实际路径长度/最短路径长度)
    double time_efficiency = 0.0;     // 时间效率

    // 综合评分
    double overall_score = 0.0;       // 综合质量评分 (0-100)
};

/**
 * @brief 轨迹跟踪状态
 */
struct TrackingState {
    double current_time = 0.0;
    size_t current_segment = 0;
    plugin::TrajectoryPoint target_point;
    plugin::TrajectoryPoint actual_point;
    bool target_is_reverse = false;
    TrackingQualityMetrics quality;
};

/**
 * @brief 高精度轨迹跟踪器
 *
 * 特性：
 * - 时间同步的精确插值
 * - 多种跟踪策略支持
 * - 实时质量评估
 * - 平滑滤波和预测控制
 */
class TrajectoryTracker {
public:
    enum class TrackingMode {
        TIME_SYNC,        // 时间同步插值
        LOOKAHEAD,        // 前瞻控制
        PREDICTIVE,       // 预测控制
        HYBRID            // 混合策略
    };

    struct Config {
        TrackingMode mode = TrackingMode::HYBRID;
        double lookahead_time = 0.3;           // 前瞻时间 (s)
        double lookahead_distance = 1.0;       // 前瞻距离 (m)
        double interpolation_tolerance = 0.01; // 插值容差 (s)
        double smoothing_factor = 0.1;         // 平滑滤波因子
        bool enable_prediction = true;         // 启用预测控制
        bool enable_quality_assessment = true; // 启用质量评估

        // 动力学限制
        double max_velocity = 3.0;             // 最大速度 (m/s)
        double max_acceleration = 2.0;         // 最大加速度 (m/s²)
        double max_angular_velocity = 2.0;     // 最大角速度 (rad/s)
        double max_jerk = 5.0;                 // 最大jerk (m/s³)

        // 倒车支持
        bool enable_reverse_tracking = true;       // 启用倒车跟踪
        double reverse_detection_threshold = 0.05; // 判定倒车的速度阈值 (m/s)
        double reverse_lookahead_time = 0.2;       // 倒车前瞻时间 (s)
        double reverse_lookahead_distance = 0.5;   // 倒车前瞻距离 (m)
        double max_reverse_velocity = 1.0;         // 最大倒车速度 (m/s)
    };

public:
    TrajectoryTracker();
    explicit TrajectoryTracker(const Config& config);
    ~TrajectoryTracker() = default;

    // ========== 配置管理 ==========

    /**
     * @brief 设置跟踪配置
     */
    void setConfig(const Config& config);

    /**
     * @brief 获取当前配置
     */
    const Config& getConfig() const { return config_; }

    // ========== 轨迹管理 ==========

    /**
     * @brief 设置要跟踪的轨迹
     */
    void setTrajectory(const std::vector<plugin::TrajectoryPoint>& trajectory);

    /**
     * @brief 从protobuf格式设置轨迹
     */
    void setTrajectoryFromProto(const proto::PlanUpdate& plan_update,
                                double trajectory_start_time = 0.0);

    /**
     * @brief 设置轨迹的起始仿真时间
     */
    void setTrajectoryStartTime(double start_time);

    /**
     * @brief 检查是否有有效轨迹
     */
    bool hasValidTrajectory() const;

    /**
     * @brief 获取轨迹总时长
     */
    double getTrajectoryDuration() const;

    // ========== 控制指令生成 ==========

    /**
     * @brief 获取指定时间的控制指令
     * @param sim_time 当前仿真时间
     * @return 速度控制指令
     */
    planning::Twist2d getControlCommand(double sim_time);

    /**
     * @brief 获取指定时间的目标状态
     * @param sim_time 当前仿真时间
     * @return 目标轨迹点
     */
    plugin::TrajectoryPoint getTargetState(double sim_time) const;

    /**
     * @brief 预测性控制指令生成
     * @param current_pose 当前位姿
     * @param current_twist 当前速度
     * @param sim_time 当前仿真时间
     * @return 控制指令
     */
    planning::Twist2d getPredictiveControl(
        const planning::Pose2d& current_pose,
        const planning::Twist2d& current_twist,
        double sim_time);

    // ========== 质量评估 ==========

    /**
     * @brief 更新跟踪质量评估
     * @param actual_pose 实际位姿
     * @param actual_twist 实际速度
     * @param sim_time 当前仿真时间
     */
    void updateQualityAssessment(
        const planning::Pose2d& actual_pose,
        const planning::Twist2d& actual_twist,
        double sim_time);

    /**
     * @brief 获取当前跟踪状态
     */
    const TrackingState& getTrackingState() const { return tracking_state_; }

    /**
     * @brief 获取质量评估指标
     */
    const TrackingQualityMetrics& getQualityMetrics() const {
        return tracking_state_.quality;
    }

    // ========== 实用工具 ==========

    /**
     * @brief 重置跟踪器状态
     */
    void reset();

    /**
     * @brief 检查轨迹是否完成
     */
    bool isTrajectoryCompleted(double sim_time) const;

    /**
     * @brief 获取轨迹完成百分比
     */
    double getCompletionPercentage(double sim_time) const;

    /**
     * @brief 当前轨迹是否包含倒车片段
     */
    bool hasReverseSegments() const { return has_reverse_segments_; }

private:
    // ========== 内部方法 ==========

    /**
     * @brief 查找指定时间周围的轨迹点
     */
    std::pair<size_t, size_t> findSurroundingIndices(double target_time) const;

    /**
     * @brief 时间插值生成控制指令
     */
    planning::Twist2d interpolateVelocity(double target_time);

    /**
     * @brief 前瞻控制策略
     */
    planning::Twist2d lookaheadControl(double trajectory_time);

    /**
     * @brief 查找最近的轨迹点
     */
    size_t findClosestTrajectoryPoint(const planning::Pose2d& current_pose) const;

    /**
     * @brief 计算前瞻目标点
     */
    plugin::TrajectoryPoint calculateLookaheadTarget(
        const planning::Pose2d& current_pose,
        double trajectory_time) const;

    /**
     * @brief 应用平滑滤波
     */
    planning::Twist2d applySmoothingFilter(const planning::Twist2d& raw_command);

    /**
     * @brief 应用动力学约束
     */
    planning::Twist2d applyDynamicConstraints(const planning::Twist2d& command);

    /**
     * @brief 计算位置误差
     */
    double calculatePositionError(
        const planning::Pose2d& actual,
        const planning::Pose2d& target) const;

    /**
     * @brief 计算速度误差
     */
    double calculateVelocityError(
        const planning::Twist2d& actual,
        const planning::Twist2d& target) const;

    /**
     * @brief 计算平滑度评分
     */
    double calculateSmoothnessScore() const;

    /**
     * @brief 更新综合质量评分
     */
    void updateOverallScore();

    /**
     * @brief 角度标准化
     */
    double normalizeAngle(double angle) const;

    /**
     * @brief 将仿真时间转换为轨迹时间
     */
    double toTrajectoryTime(double sim_time) const;

    /**
     * @brief 获取指定轨迹时间的目标状态
     */
    plugin::TrajectoryPoint getTargetStateAtTrajectoryTime(double trajectory_time) const;

    /**
     * @brief 判断给定时间是否处于倒车片段
     */
    bool isReverseAtTime(double trajectory_time) const;

    /**
     * @brief 判断指定索引是否为倒车片段
     */
    bool isReverseSegment(size_t index) const;

    /**
     * @brief 获取指定时间的轨迹路径长度
     */
    double getPathLengthAt(double trajectory_time) const;

    /**
     * @brief 获取指定索引对应的修正航向
     */
    double getEffectiveYaw(size_t index) const;

private:
    Config config_;
    std::vector<plugin::TrajectoryPoint> trajectory_;
    std::vector<bool> reverse_flags_;
    std::vector<double> effective_yaws_;
    bool has_reverse_segments_ = false;
    double total_path_length_ = 0.0;
    TrackingState tracking_state_;

    // 滤波和历史数据
    planning::Twist2d last_command_;
    std::vector<planning::Twist2d> command_history_;
    std::vector<TrackingQualityMetrics> quality_history_;

    // 时间管理
    std::chrono::steady_clock::time_point last_update_time_;
    double trajectory_start_sim_time_ = 0.0;
    bool has_start_time_ = false;

    // 初始化标志
    bool initialized_ = false;
    bool has_trajectory_ = false;
};

} // namespace control
} // namespace navsim
