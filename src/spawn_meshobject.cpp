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
    mesh_cone.id = "custom_mesh";
    mesh_cone.header.frame_id = "torso";
    Eigen::Vector3d scale(0.0001, 0.0001, 0.0001);  // mm → m
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
    mesh_pose.position.x = 0.8;
    mesh_pose.position.y = 0.0;
    mesh_pose.position.z = 0.1;

    mesh_cone.mesh_poses.push_back(mesh_pose);
    mesh_cone.operation = mesh_cone.ADD;
    planning_scene_interface.applyCollisionObject(mesh_cone);

    /*  ==============================
            2. launcher cylinder
        ==============================*/
    moveit_msgs::msg::CollisionObject mesh_cyl;
    mesh_cyl.id = "cyl";
    mesh_cyl.header.frame_id = "torso";
    Eigen::Vector3d scale_cyl(0.001, 0.001, 0.001);  // mm → m
    std::string mesh_path_cyl = "file:///home/cstar/Documents/rough_ws/src/robot_venvs/models/cylinder_hole.STL";
    shapes::Mesh* mesh_ = shapes::createMeshFromResource(mesh_path_cyl, scale_cyl);
    if (!mesh_){
        RCLCPP_ERROR(node->get_logger(), "Failed to load mesh from: %s", mesh_path_cyl.c_str());
        return 1;}
    shapes::ShapeMsg mesh_msg_tmp_cyl;
    shapes::constructMsgFromShape(mesh_, mesh_msg_tmp_cyl);
    shape_msgs::msg::Mesh mesh_msg_cyl = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp_cyl);
    mesh_cyl.meshes.push_back(mesh_msg_cyl);
    
    // Pose of mesh
    double roll_cyl, pitch_cyl, yaw_cyl;
    roll_cyl = 0.0;
    pitch_cyl = 0.3;
    yaw_cyl = 0.0;
    tf2::Quaternion q_cyl;
    q_cyl.setRPY(roll_cyl* M_PI / 180.0, pitch_cyl* M_PI / 180.0, yaw_cyl* M_PI / 180.0);
    q_cyl.normalize();

    geometry_msgs::msg::Pose mesh_pose_cyl;
    mesh_pose_cyl.orientation.x = q_cyl.x();
    mesh_pose_cyl.orientation.y = q_cyl.y();
    mesh_pose_cyl.orientation.z = q_cyl.z();
    mesh_pose_cyl.orientation.w = q_cyl.w();
    mesh_pose_cyl.position.x = 0.8;
    mesh_pose_cyl.position.y = 0.4;
    mesh_pose_cyl.position.z = -0.2;

    mesh_cyl.mesh_poses.push_back(mesh_pose_cyl);
    mesh_cyl.operation = mesh_cyl.ADD;
    planning_scene_interface.applyCollisionObject(mesh_cyl);

    delete mesh;
    delete mesh_;


    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
