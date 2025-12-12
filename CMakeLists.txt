cmake_minimum_required(VERSION 3.8)
project(robot_venvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(dualarm_custom_msgs REQUIRED)


add_executable(spawn_object src/spawn_object.cpp)
target_include_directories(spawn_object
PUBLIC include)
ament_target_dependencies(spawn_object ament_cmake 
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
geometry_msgs
moveit_msgs
shape_msgs
)

add_executable(spawn_meshobject src/spawn_meshobject.cpp)
target_include_directories(spawn_object
PUBLIC include)
ament_target_dependencies(spawn_meshobject ament_cmake 
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
geometry_msgs
moveit_msgs
shape_msgs
)

add_executable(spawn_pick_place_object src/spawn_pick_place_object.cpp)
target_include_directories(spawn_pick_place_object
PUBLIC include)
ament_target_dependencies(spawn_pick_place_object ament_cmake 
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
geometry_msgs
moveit_msgs
shape_msgs
)
add_executable(spawn_bottle_camPose src/spawn_bottle_camPose.cpp)
target_include_directories(spawn_bottle_camPose
PUBLIC include)
ament_target_dependencies(spawn_bottle_camPose ament_cmake 
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
geometry_msgs
moveit_msgs
shape_msgs
dualarm_custom_msgs
)

add_executable(spawn_bottle src/spawn_bottle.cpp)
target_include_directories(spawn_bottle
PUBLIC include)
ament_target_dependencies(spawn_bottle ament_cmake 
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
geometry_msgs
moveit_msgs
shape_msgs
)

add_executable(spawn_cylinder src/spawn_cylinder.cpp)
target_include_directories(spawn_cylinder
PUBLIC include)
ament_target_dependencies(spawn_cylinder ament_cmake 
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
geometry_msgs
moveit_msgs
shape_msgs
)

add_executable(spawn_table src/spawn_table.cpp)
target_include_directories(spawn_table
PUBLIC include)
ament_target_dependencies(spawn_table ament_cmake 
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
geometry_msgs
moveit_msgs
shape_msgs
)

install(TARGETS 
    spawn_object
    spawn_meshobject 
    spawn_pick_place_object
    spawn_bottle
    spawn_cylinder
    spawn_bottle_camPose
    spawn_table
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME})

ament_package()
