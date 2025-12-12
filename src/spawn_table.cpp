#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_mesh_example");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    /*  ==============================
            1. TABLE
        ==============================*/

    moveit_msgs::msg::CollisionObject collision_table;
    collision_table.header.frame_id = "torso_hw";
    collision_table.id = "table";

    shape_msgs::msg::SolidPrimitive primitive_table;
    primitive_table.type = primitive_table.BOX;
    primitive_table.dimensions = {0.8, 1.6, 0.01};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.7;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.58;
    // table_pose.orientation.x = 0.5;
    // table_pose.orientation.y = -0.5;
    // table_pose.orientation.z = 0.5;
    // table_pose.orientation.w = -0.5;
    collision_table.primitives.push_back(primitive_table);
    collision_table.primitive_poses.push_back(table_pose);
    collision_table.operation = collision_table.ADD;
    planning_scene_interface.applyCollisionObject(collision_table);


    RCLCPP_INFO(node->get_logger(), "Object added to planning scene");

    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
