#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spawn_object");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    /*  ==============================
            1. CYLINDER
        ==============================*/
    moveit_msgs::msg::CollisionObject collision_cube;
    collision_cube.header.frame_id = "torso";
    collision_cube.id = "cylinder";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;

    double height = 0.30;   // 30 cm tall
    double radius = 0.04;   // 5 cm radius

    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;

    // Pose of mesh
    double roll, pitch, yaw;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    tf2::Quaternion q;
    q.setRPY(roll* M_PI / 180.0, pitch* M_PI / 180.0, yaw* M_PI / 180.0);
    q.normalize();
    geometry_msgs::msg::Pose cube_pose;
    cube_pose.position.x = 0.68;
    cube_pose.position.y = -0.4;
    cube_pose.position.z = -0.27;
    cube_pose.orientation.x = q.x();
    cube_pose.orientation.y = q.y();
    cube_pose.orientation.z = q.z();
    cube_pose.orientation.w = q.w();

    collision_cube.primitives.push_back(primitive);
    collision_cube.primitive_poses.push_back(cube_pose);
    collision_cube.operation = collision_cube.ADD;
    planning_scene_interface.applyCollisionObject(collision_cube);


    RCLCPP_INFO(node->get_logger(), "Object added to planning scene");

    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::shutdown();
    return 0;
}