#include "control/trajectory_tracker.hpp"
#include "plugin/data/planning_result.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace navsim;

// 创建测试轨迹
std::vector<plugin::TrajectoryPoint> createTestTrajectory() {
    std::vector<plugin::TrajectoryPoint> trajectory;

    // 创建一个简单的直线轨迹，从 (0,0) 到 (10,0)
    const int num_points = 50;
    const double total_time = 5.0;  // 5秒
    const double dt = total_time / (num_points - 1);

    for (int i = 0; i < num_points; ++i) {
        plugin::TrajectoryPoint point;

        double t = i * dt;
        double progress = static_cast<double>(i) / (num_points - 1);

        // 位置: 从(0,0)到(10,0)的直线
        point.pose.x = progress * 10.0;
        point.pose.y = 0.0;
        point.pose.yaw = 0.0;

        // 速度: 恒定速度 2 m/s
        point.twist.vx = 2.0;
        point.twist.vy = 0.0;
        point.twist.omega = 0.0;

        // 时间
        point.time_from_start = t;

        // 路径长度
        point.path_length = progress * 10.0;

        trajectory.push_back(point);
    }

    return trajectory;
}

int main() {
    std::cout << "=== 轨迹跟踪器测试 ===" << std::endl;

    // 1. 创建轨迹跟踪器
    control::TrajectoryTracker::Config config;
    config.mode = control::TrajectoryTracker::TrackingMode::HYBRID;
    config.lookahead_time = 0.3;
    config.enable_quality_assessment = true;
    config.max_velocity = 3.0;
    config.max_acceleration = 2.0;

    control::TrajectoryTracker tracker(config);
    std::cout << "✅ 轨迹跟踪器已初始化" << std::endl;

    // 2. 设置测试轨迹
    auto test_trajectory = createTestTrajectory();
    tracker.setTrajectory(test_trajectory);
    std::cout << "✅ 测试轨迹已加载: " << test_trajectory.size() << " 个点" << std::endl;
    std::cout << "   轨迹时长: " << tracker.getTrajectoryDuration() << " 秒" << std::endl;

    // 3. 模拟轨迹跟踪
    std::cout << "\n=== 开始模拟轨迹跟踪 ===" << std::endl;

    const double sim_dt = 0.033;  // 30Hz仿真频率
    const double sim_duration = 6.0;  // 仿真6秒

    planning::Pose2d actual_pose{0.0, 0.0, 0.0};  // 实际位姿
    planning::Twist2d actual_twist{0.0, 0.0, 0.0};  // 实际速度

    for (double sim_time = 0.0; sim_time <= sim_duration; sim_time += sim_dt) {
        // 获取控制指令
        auto control_cmd = tracker.getControlCommand(sim_time);

        // 模拟车辆运动（简化的运动学模型）
        actual_twist = control_cmd;  // 假设完美跟踪速度
        actual_pose.x += actual_twist.vx * sim_dt;
        actual_pose.y += actual_twist.vy * sim_dt;
        actual_pose.yaw += actual_twist.omega * sim_dt;

        // 更新质量评估
        tracker.updateQualityAssessment(actual_pose, actual_twist, sim_time);

        // 每秒打印一次状态
        if (static_cast<int>(sim_time * 10) % 10 == 0) {
            auto quality = tracker.getQualityMetrics();
            auto target_state = tracker.getTargetState(sim_time);
            double completion = tracker.getCompletionPercentage(sim_time);

            std::cout << "时间: " << std::fixed << std::setprecision(1) << sim_time << "s" << std::endl;
            std::cout << "  目标位置: (" << std::setprecision(2) << target_state.pose.x
                      << ", " << target_state.pose.y << ")" << std::endl;
            std::cout << "  实际位置: (" << actual_pose.x << ", " << actual_pose.y << ")" << std::endl;
            std::cout << "  位置误差: " << quality.position_error * 1000 << " mm" << std::endl;
            std::cout << "  速度误差: " << quality.velocity_error * 1000 << " mm/s" << std::endl;
            std::cout << "  平滑度: " << static_cast<int>(quality.smoothness_score) << "/100" << std::endl;
            std::cout << "  综合评分: " << static_cast<int>(quality.overall_score) << "/100" << std::endl;
            std::cout << "  完成度: " << static_cast<int>(completion) << "%" << std::endl;
            std::cout << "  控制指令: vx=" << control_cmd.vx << " m/s" << std::endl;
            std::cout << std::endl;
        }
    }

    // 4. 最终结果
    std::cout << "=== 跟踪完成 ===" << std::endl;
    auto final_quality = tracker.getQualityMetrics();
    std::cout << "最终质量评估:" << std::endl;
    std::cout << "  平均位置误差: " << final_quality.position_error * 1000 << " mm" << std::endl;
    std::cout << "  平均速度误差: " << final_quality.velocity_error * 1000 << " mm/s" << std::endl;
    std::cout << "  平滑度评分: " << static_cast<int>(final_quality.smoothness_score) << "/100" << std::endl;
    std::cout << "  综合评分: " << static_cast<int>(final_quality.overall_score) << "/100" << std::endl;

    if (final_quality.overall_score > 80) {
        std::cout << "🎉 轨迹跟踪质量优秀!" << std::endl;
    } else if (final_quality.overall_score > 60) {
        std::cout << "👍 轨迹跟踪质量良好" << std::endl;
    } else {
        std::cout << "⚠️  轨迹跟踪质量需要改进" << std::endl;
    }

    return 0;
}