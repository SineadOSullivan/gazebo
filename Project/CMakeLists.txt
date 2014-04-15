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
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Add Behavior Source and Header files
file(GLOB BEHAVIOR_SRCS behavior_include/*.h behavior/*.cpp)
# Add Common Architecture Source and Header files
file(GLOB ARCHITECTURE_SRCS include/architecture.h src/architecture.cpp)

# Add Subsumption Source and Header files
file(GLOB SUBSUMPTION_SRCS include/subsumption_arch.h src/subsumption_arch.cpp)
# Add Subsumption Architecture Library
#add_library(subsumption_arch SHARED ${SUBSUMPTION_SRCS} ${BEHAVIOR_SRCS} ${ARCHITECTURE_SRCS})
#target_link_libraries(subsumption_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Add Motor Schema Source and Header files
file(GLOB MOTOR_SCHEMA_SRCS include/motor_schema_arch.h src/motor_schema_arch.cpp)
# Add Motor Schema Architecture Library
add_library(motor_schema_arch SHARED ${MOTOR_SCHEMA_SRCS} ${BEHAVIOR_SRCS} ${ARCHITECTURE_SRCS})
target_link_libraries(motor_schema_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Add DAMN Source and Header files
file(GLOB DAMN_SRCS include/damn_arch.h src/damn_arch.cpp)
# Add DAMN Architecture Library
#add_library(damn_arch SHARED ${DAMN_SRCS} ${BEHAVIOR_SRCS} ${ARCHITECTURE_SRCS})
#target_link_libraries(damn_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
