#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <sstream>
#include <cmath>
#include <iterator>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <dualarm_custom_msgs/msg/obj_pose_array.hpp>
#include <dualarm_custom_msgs/msg/obj_pose.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class  RobotVenvs:public rclcpp::Node{
public:
     RobotVenvs(): Node("robot_venvs"){
        sub = this->create_subscription<dualarm_custom_msgs::msg::ObjPoseArray>("object_pose_torso", 10,
        std::bind(&RobotVenvs::poseCallback, this, _1));
     }
     void run(){
        rclcpp::Rate wait_rate(10);
        while (rclcpp::ok() && !pose_received_) {
            wait_rate.sleep();
        }
        if (!rclcpp::ok()) return;
        std::string mesh_path= "file:///home/cstar/Documents/rough_ws/src/robot_venvs/models/bottle/bottle.obj";
        spawn_mesh(mesh_path);
     }

private:
     void spawn_mesh(const std::string &mesh_path){
        moveit_msgs::msg::CollisionObject mesh1;
        mesh1.id = "bottle";
        mesh1.header.frame_id = "torso_hw";
        Eigen::Vector3d scale(0.0002, 0.0002, 0.0002);
        shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path, scale);
        if (!mesh){
            RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from : %s", mesh_path.c_str());
            return;
        }
        shapes::ShapeMsg mesh_msg_tmp;
        shapes::constructMsgFromShape(mesh, mesh_msg_tmp);
        shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);
        mesh1.meshes.push_back(mesh_msg);
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
        mesh_pose.position.x = first_pose_.position.x + 0.16;
        mesh_pose.position.y = first_pose_.position.y;
        mesh_pose.position.z = first_pose_.position.z;


        mesh1.mesh_poses.push_back(mesh_pose);
        mesh1.operation = mesh1.ADD;
        planning_scene_interface.applyCollisionObject(mesh1);
        RCLCPP_INFO(this->get_logger(), "Object added to planning scene");
        delete mesh;
     }

     void poseCallback(const dualarm_custom_msgs::msg::ObjPoseArray::SharedPtr msg){
        if (pose_received_) return;
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty ObjPoseArray.");
            return;
        }
        const auto &obj = msg->data.front();
        const auto &pose = obj.pose_stamped.pose;

        first_pose_ = pose; 
        pose_received_ = true;
        RCLCPP_INFO(this->get_logger(),
            "🟢 Spawning Object at: %s | Position [x: %.3f, y: %.3f, z: %.3f]",
            obj.object_name.c_str(), pose.position.x, pose.position.y, pose.position.z);
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    rclcpp::Subscription<dualarm_custom_msgs::msg::ObjPoseArray>::SharedPtr sub;

    bool pose_received_;
    geometry_msgs::msg::Pose first_pose_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<RobotVenvs>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor](){executor.spin();}).detach();

    node->run();
    rclcpp::shutdown();
    return 0;
}

