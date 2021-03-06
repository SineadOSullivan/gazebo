cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#-----------------------------------------------
#                COMPILE SETTINGS
#-----------------------------------------------
set(CMAKE_BUILD_TYPE "Debug")
set(BUILD_FLAGS "-std=gnu++0x -Wreturn-type -Wno-enum-compare -Wall")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Boost REQUIRED COMPONENTS system)
include_directories(include)
include_directories(behavior_include)
#include_directories(metric_include)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Common behaviors and source files
file(GLOB BEHAVIOR_SRCS behavior_include/*.h behavior/*.cpp)
file(GLOB ARCHITECTURE_SRCS include/architecture.h src/architecture.cpp)

# Collision Plugin
#file(GLOB COLLISION_SRCS metric_include/collision_plugin.h metric/collision_plugin.cpp)
#add_library(collision_plugin SHARED ${COLLISION_SRCS})
#target_link_libraries(collision_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Subsumption
file(GLOB SUBSUMPTION_SRCS include/subsumption_arch.h src/subsumption_arch.cpp)
add_library(subsumption_arch SHARED ${SUBSUMPTION_SRCS} ${BEHAVIOR_SRCS} ${ARCHITECTURE_SRCS})
target_link_libraries(subsumption_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Motor Schema
file(GLOB MOTOR_SCHEMA_SRCS include/motor_schema_arch.h src/motor_schema_arch.cpp)
add_library(motor_schema_arch SHARED ${MOTOR_SCHEMA_SRCS} ${BEHAVIOR_SRCS} ${ARCHITECTURE_SRCS})
target_link_libraries(motor_schema_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# DAMN
file(GLOB DAMN_SRCS include/damn_arch.h src/damn_arch.cpp)
add_library(damn_arch SHARED ${DAMN_SRCS} ${BEHAVIOR_SRCS} ${ARCHITECTURE_SRCS})
target_link_libraries(damn_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
