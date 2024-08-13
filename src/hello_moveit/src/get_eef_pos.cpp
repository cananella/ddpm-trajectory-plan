#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>


using namespace std;

void quaternionToRPY(const geometry_msgs::msg::Quaternion& quaternion){
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
}

int main(int argc, char* argv[]) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Declare Node
  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("get_eef_pose",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
        true));

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, "dsr_m0609");

  // print current pose
  geometry_msgs::msg::Pose current_pose =
    move_group_interface.getCurrentPose().pose;

  // Print the current pose of the end effector
  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);
  

  tf2::Quaternion quat_tf;
  geometry_msgs::msg::Quaternion quat_msg = current_pose.orientation;
  quaternionToRPY(quat_msg);

  vector<double> end_rpy= move_group_interface.getCurrentRPY();

  RCLCPP_INFO(node->get_logger(), "Current rpy pose: %f %f %f",end_rpy[0],end_rpy[1],end_rpy[2]);


  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}