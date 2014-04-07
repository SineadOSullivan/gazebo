cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(subsumption_arch SHARED subsumption.cc)
target_link_libraries(subsumption_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(motor_schema_arch SHARED motor_schema_arch.cc)
target_link_libraries(motor_schema_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(damn_arch SHARED damn_arch.cc)
target_link_libraries(damn_arch ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
