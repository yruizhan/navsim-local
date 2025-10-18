# JpsPlanner Plugin CMakeLists.txt

# Find dependencies
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

# Create plugin library
add_library(jps_planner_new_plugin SHARED
    # Algorithm layer (pure algorithm, no platform dependencies)
    algorithm/jps_data_structures.hpp
    algorithm/graph_search.hpp
    algorithm/graph_search.cpp
    algorithm/jps_planner.hpp
    algorithm/jps_planner.cpp

    # Adapter layer (platform interface adaptation)
    adapter/jps_planner_plugin.hpp
    adapter/jps_planner_plugin.cpp
    adapter/register.cpp
)

# Set library properties
set_target_properties(jps_planner_new_plugin PROPERTIES
    OUTPUT_NAME "jps_planner_plugin"
    VERSION 1.0.0
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Include directories
target_include_directories(jps_planner_new_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algorithm>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/adapter>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/platform/include
        ${CMAKE_SOURCE_DIR}/plugins/perception/esdf_builder/include
        ${Boost_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(jps_planner_new_plugin
    PUBLIC
        navsim_plugin_framework
        Eigen3::Eigen
        Boost::boost
    PRIVATE
        esdf_builder_plugin
)

# Install rules
install(TARGETS jps_planner_new_plugin
    LIBRARY DESTINATION lib/navsim/plugins/planning
    ARCHIVE DESTINATION lib/navsim/plugins/planning
)
