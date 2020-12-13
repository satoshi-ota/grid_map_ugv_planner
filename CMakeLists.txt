cmake_minimum_required(VERSION 3.0.2)
project(grid_map_ugv_planner)
add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
    roscpp
    grid_map_core
    grid_map_ros
    grid_map_filters
    grid_map_loader
    grid_map_msgs
    grid_map_octomap
    grid_map_rviz_plugin
    grid_map_visualization
    geometry_msgs
    nav_msgs
    visualization_msgs
    sensor_msgs
    octomap_msgs
    filters
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(octomap REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
    src/grid_map_ugv_planner.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(grid_map_ugv_planner_node src/grid_map_ugv_planner_node.cpp)
add_dependencies(grid_map_ugv_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_map_ugv_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
)
