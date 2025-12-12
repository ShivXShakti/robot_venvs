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
            1. launcher cone
        ==============================*/
    moveit_msgs::msg::CollisionObject mesh_cone;
    mesh_cone.id = "bottle";
    mesh_cone.header.frame_id = "torso";
    Eigen::Vector3d scale(0.0002, 0.0002, 0.0002);  // mm → m
    std::string mesh_path = "file:///home/cstar/Documents/rough_ws/src/robot_venvs/models/bottle/bottle.obj";
    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path, scale);
    if (!mesh){
        RCLCPP_ERROR(node->get_logger(), "Failed to load mesh from: %s", mesh_path.c_str());
        return 1;}
    shapes::ShapeMsg mesh_msg_tmp;
    shapes::constructMsgFromShape(mesh, mesh_msg_tmp);
    shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);
    mesh_cone.meshes.push_back(mesh_msg);
    
    // Pose of mesh
    double roll, pitch, yaw;
    roll = 90.0;
    pitch = 0.0;
    yaw = 0.0;
    tf2::Quaternion q;
    q.setRPY(roll* M_PI / 180.0, pitch* M_PI / 180.0, yaw* M_PI / 180.0);
    q.normalize();

    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.orientation.x = q.x();
    mesh_pose.orientation.y = q.y();
    mesh_pose.orientation.z = q.z();
    mesh_pose.orientation.w = q.w();
    mesh_pose.position.x = 0.68;
    mesh_pose.position.y = -0.4;
    mesh_pose.position.z = -0.27;

    mesh_cone.mesh_poses.push_back(mesh_pose);
    mesh_cone.operation = mesh_cone.ADD;
    planning_scene_interface.applyCollisionObject(mesh_cone);

    /*  ==============================
            2. TABLE
        ==============================*/

    moveit_msgs::msg::CollisionObject collision_table;
    collision_table.header.frame_id = "torso";
    collision_table.id = "table";

    shape_msgs::msg::SolidPrimitive primitive_table;
    primitive_table.type = primitive_table.BOX;
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

    delete mesh;


    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
