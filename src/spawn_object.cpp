#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spawn_object");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    /*  ==============================
            1. CUBE
        ==============================*/
    moveit_msgs::msg::CollisionObject collision_cube;
    collision_cube.header.frame_id = "torso";
    collision_cube.id = "cube";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.05, 0.15, 0.05};

    geometry_msgs::msg::Pose cube_pose;
    cube_pose.position.x = 0.75;
    cube_pose.position.y = -0.4;
    cube_pose.position.z = -0.29;
    cube_pose.orientation.x = 0.5;
    cube_pose.orientation.y = -0.5;
    cube_pose.orientation.z = 0.5;
    cube_pose.orientation.w = -0.5;

    collision_cube.primitives.push_back(primitive);
    collision_cube.primitive_poses.push_back(cube_pose);
    collision_cube.operation = collision_cube.ADD;
    planning_scene_interface.applyCollisionObject(collision_cube);

    /*  ==============================
            2. TABLE
        ==============================*/

    moveit_msgs::msg::CollisionObject collision_table;
    collision_table.header.frame_id = "torso";
    collision_table.id = "table";

    shape_msgs::msg::SolidPrimitive primitive_table;
    primitive_table.type = primitive.BOX;
    primitive_table.dimensions = {0.8, 1.6, 0.01};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.7;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.38;
    // table_pose.orientation.x = 0.5;
    // table_pose.orientation.y = -0.5;
    // table_pose.orientation.z = 0.5;
    // table_pose.orientation.w = -0.5;
    collision_table.primitives.push_back(primitive_table);
    collision_table.primitive_poses.push_back(table_pose);
    collision_table.operation = collision_table.ADD;
    planning_scene_interface.applyCollisionObject(collision_table);


    RCLCPP_INFO(node->get_logger(), "Object added to planning scene");

    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::shutdown();
    return 0;
}