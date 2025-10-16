# JPS Planner Plugin CMakeLists.txt

cmake_minimum_required(VERSION 3.10)

# Find Boost (required for heap data structures)
find_package(Boost REQUIRED)

# Plugin name
set(PLUGIN_NAME jps_planner_plugin)

# Source files
set(PLUGIN_SOURCES
    src/jps_planner_plugin.cpp
    src/jps_planner.cpp
    src/graph_search.cpp
    src/register.cpp
)

# Header files (for IDE)
set(PLUGIN_HEADERS
    include/jps_planner_plugin.hpp
    include/jps_planner.hpp
    include/graph_search.hpp
    include/jps_data_structures.hpp
)

# Create plugin library
add_library(${PLUGIN_NAME} SHARED
    ${PLUGIN_SOURCES}
    ${PLUGIN_HEADERS}
)

# Include directories
target_include_directories(${PLUGIN_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/plugins/perception/esdf_builder/include
)

# Link libraries
target_link_libraries(${PLUGIN_NAME}
    PUBLIC
        navsim_plugin_framework
        Eigen3::Eigen
        Boost::boost
    PRIVATE
        esdf_builder_plugin
)

# Compiler options
target_compile_features(${PLUGIN_NAME} PUBLIC cxx_std_17)
target_compile_options(${PLUGIN_NAME} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# Install plugin
install(TARGETS ${PLUGIN_NAME}
    LIBRARY DESTINATION lib/navsim/plugins/planning
    ARCHIVE DESTINATION lib/navsim/plugins/planning
)

# Install headers (optional, for development)
install(DIRECTORY include/
    DESTINATION include/navsim/plugins/planning/jps_planner
    FILES_MATCHING PATTERN "*.hpp"
)

# Print build info
message(STATUS "Configuring JPS Planner Plugin")
message(STATUS "  - Plugin name: ${PLUGIN_NAME}")
message(STATUS "  - Source files: ${PLUGIN_SOURCES}")
message(STATUS "  - Header files: ${PLUGIN_HEADERS}")

