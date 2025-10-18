# NavSimPluginSDK Package Configuration
#
# This file provides the NavSim Plugin SDK for external plugin development.

include(${CMAKE_CURRENT_LIST_DIR}/NavSimPluginSDK.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/FindNavSimCore.cmake)

set(NavSimPluginSDK_FOUND TRUE)
set(NavSimPluginSDK_VERSION "1.0.0")

# Set component variables
set(NavSimPluginSDK_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../include)
set(NavSimPluginSDK_LIBRARIES "")

# Mark as found
set(NavSimPluginSDK_FOUND TRUE)

message(STATUS "Found NavSim Plugin SDK ${NavSimPluginSDK_VERSION}")