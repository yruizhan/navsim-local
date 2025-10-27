#include "control/trajectory_tracker.hpp"
#include <iostream>
#include <vector>

using namespace navsim;

int main() {
    std::cout << "=== 简化轨迹跟踪测试 ===" << std::endl;

    // 创建跟踪器
    control::TrajectoryTracker::Config config;
    config.mode = control::TrajectoryTracker::TrackingMode::HYBRID;
    control::TrajectoryTracker tracker(config);

    // 创建简单测试轨迹
    std::vector<plugin::TrajectoryPoint> trajectory;
    for (int i = 0; i < 10; ++i) {
        plugin::TrajectoryPoint point;
        point.pose.x = i * 0.5;  // 每0.5米一个点
        point.pose.y = 0.0;
        point.pose.yaw = 0.0;
        point.twist.vx = 1.0;  // 1 m/s恒定速度
        point.twist.vy = 0.0;
        point.twist.omega = 0.0;
        point.time_from_start = i * 0.5;  // 对应的时间
        trajectory.push_back(point);
    }

    std::cout << "轨迹点数量: " << trajectory.size() << std::endl;

    // 设置轨迹
    tracker.setTrajectory(trajectory);
    std::cout << "✅ 轨迹设置成功" << std::endl;
    std::cout << "轨迹时长: " << tracker.getTrajectoryDuration() << " 秒" << std::endl;

    // 测试几个时间点的跟踪
    std::vector<double> test_times = {0.0, 0.5, 1.0, 1.5, 2.0};

    for (double test_time : test_times) {
        std::cout << "\n--- 时间 " << test_time << " s ---" << std::endl;

        // 获取控制指令
        auto control_cmd = tracker.getControlCommand(test_time);
        std::cout << "控制指令: vx=" << control_cmd.vx
                  << ", vy=" << control_cmd.vy
                  << ", omega=" << control_cmd.omega << std::endl;

        // 获取目标状态
        auto target_state = tracker.getTargetState(test_time);
        std::cout << "目标位置: (" << target_state.pose.x
                  << ", " << target_state.pose.y << ")" << std::endl;
        std::cout << "目标速度: vx=" << target_state.twist.vx
                  << ", vy=" << target_state.twist.vy << std::endl;

        // 模拟实际位置（稍微偏离目标）
        planning::Pose2d actual_pose{target_state.pose.x + 0.1, target_state.pose.y + 0.05, target_state.pose.yaw};
        planning::Twist2d actual_twist{control_cmd.vx * 0.9, control_cmd.vy, control_cmd.omega};

        // 更新质量评估
        tracker.updateQualityAssessment(actual_pose, actual_twist, test_time);

        // 获取质量指标
        auto quality = tracker.getQualityMetrics();
        std::cout << "跟踪质量:" << std::endl;
        std::cout << "  位置误差: " << quality.position_error * 1000 << " mm" << std::endl;
        std::cout << "  速度误差: " << quality.velocity_error * 1000 << " mm/s" << std::endl;
        std::cout << "  综合评分: " << quality.overall_score << "/100" << std::endl;

        double completion = tracker.getCompletionPercentage(test_time);
        std::cout << "完成度: " << completion << "%" << std::endl;
    }

    std::cout << "\n🎉 轨迹跟踪测试完成!" << std::endl;
    return 0;
}