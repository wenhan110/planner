cmake_minimum_required(VERSION 2.8.3)
project(traj_generator)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -o3 -Wall")

include_directories(include/)
include_directories(usr/include/eigen3)
# include_directories(../costmap/include)
add_library(TrajGenerator_shared SHARED src/planner.cpp src/MotionModel.cpp src/dwa_traj.cpp src/a_star.cpp src/CostmapBase.cpp src/inflation_layer.cpp src/obstacle_layer.cpp src/PlannerMap.cpp src/static_layer.cpp src/pure_pursuit.cpp src/pid.cpp)

add_executable(TrajGenerator src/main.cpp)

target_link_libraries(TrajGenerator TrajGenerator_shared)