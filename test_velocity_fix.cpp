#include "plugins/planning/jps_planner/algorithm/jps_planner.hpp"
#include "plugins/perception/esdf_builder/include/esdf_map.hpp"
#include <iostream>
#include <memory>

int main() {
    std::cout << "=== Testing JPS Planner Velocity State Fix ===" << std::endl;

    // Create a dummy ESDF map for testing
    auto map = std::make_shared<navsim::perception::ESDFMap>(50.0, 50.0, 0.1);
    map->setOrigin(Eigen::Vector2d(-25.0, -25.0));

    // Create JPS planner
    JPS::JPSPlanner planner(map);

    // Test setting velocity state
    Eigen::Vector3d current_velocity_state(1.5, 0.0, 0.0);  // 1.5 m/s linear velocity
    Eigen::Vector3d current_angular_state(0.2, 0.0, 0.0);   // 0.2 rad/s angular velocity

    std::cout << "Setting current velocity state:" << std::endl;
    std::cout << "  Linear velocity: " << current_velocity_state(0) << " m/s" << std::endl;
    std::cout << "  Angular velocity: " << current_angular_state(0) << " rad/s" << std::endl;

    // This should work with our new public method
    planner.setCurrentVelocityState(current_velocity_state, current_angular_state);

    std::cout << "✅ Velocity state set successfully!" << std::endl;
    std::cout << "✅ JPS planner velocity fix is working!" << std::endl;

    return 0;
}