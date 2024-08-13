#include <iostream>
#include <vector>
#include <tuple>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "m0609_interfaces/srv/trajectory_plan_rq.hpp"

using namespace std::chrono_literals;
using namespace std;

geometry_msgs::msg::Pose target_pose;
geometry_msgs::msg::Pose current_pose;
float zero_x=0.334;
float zero_y=-0.113;
float zero_z=0.088;

bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan, rclcpp::Logger logger) {
    bool success = static_cast<bool>(move_group_interface.plan(plan));
    if (success) {
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Execution successful!");
        return true;
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
        return false;
    }
}

void quaternionToRPY(const geometry_msgs::msg::Quaternion& quaternion) {
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
}

auto addCollision(moveit::planning_interface::MoveGroupInterface& move_group_interface) {
    auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "floor";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1.0;
        primitive.dimensions[primitive.BOX_Y] = 1.0;
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -primitive.dimensions[primitive.BOX_Z] / 2-0.05;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);

        collision_object.operation = collision_object.ADD;
        return collision_object;
    }();
    return collision_object;
}

void printCurrentPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, rclcpp::Logger logger) {
    current_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f %f",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z,
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w);
}

void handleTrajectoryPlanRequest(const std::shared_ptr<m0609_interfaces::srv::TrajectoryPlanRQ::Request> request,
                                 std::shared_ptr<m0609_interfaces::srv::TrajectoryPlanRQ::Response> response,
                                 moveit::planning_interface::MoveGroupInterface& move_group_interface,
                                 rclcpp::Logger logger,
                                 moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    // Cartesian path plan
    std::vector<geometry_msgs::msg::Pose> waypoints;
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction =0;

    //pose ready 
    move_group_interface.setNamedTarget("ready");
    planAndExecute(move_group_interface, plan, logger);
    this_thread::sleep_for(5s);


    //pose start point +z 0.05
    current_pose = move_group_interface.getCurrentPose().pose;
    target_pose=current_pose;
    target_pose.position.x=request->points.poses[0].position.x/2000;
    target_pose.position.y=request->points.poses[0].position.y/2000;
    target_pose.position.z=zero_z + 0.05;
    move_group_interface.setPoseTarget(target_pose);
    planAndExecute(move_group_interface, plan, logger);
    this_thread::sleep_for(5s);
    

    //pose start point
    current_pose = move_group_interface.getCurrentPose().pose;
    target_pose=current_pose;
    target_pose.position.z=zero_z;
    waypoints.push_back(target_pose);
    fraction=move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction >= 0.9) { // If the plan is successful
        move_group_interface.execute(trajectory);
        RCLCPP_INFO(logger, "Trajectory execution successful!");
    } else {
        response->doneflag = false;
        RCLCPP_ERROR(logger, "Trajectory planning failed!");
        return;
    }
    waypoints.clear();
    this_thread::sleep_for(5s);

    // trajectory plan
    current_pose = move_group_interface.getCurrentPose().pose;
    waypoints.push_back(current_pose); // Add the current pose as the start point
    target_pose = current_pose;
    for (const auto& pose : request->points.poses) {
        target_pose.position.x=pose.position.x/2000;
        target_pose.position.y=pose.position.y/2000;
        waypoints.push_back(target_pose);
    }
    fraction=move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction >= 0.9) { // If the plan is successful
        move_group_interface.execute(trajectory);
        RCLCPP_INFO(logger, "Trajectory execution successful!");
    } else {
        response->doneflag = false;
        RCLCPP_ERROR(logger, "Trajectory planning failed!");
        return;
    }
    waypoints.clear();
    this_thread::sleep_for(5s);

    //pose end point z+0.05
    current_pose = move_group_interface.getCurrentPose().pose;
    target_pose=current_pose;
    target_pose.position.z+=0.05;
    waypoints.push_back(target_pose);
    fraction=move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction >= 0.9) { // If the plan is successful
        move_group_interface.execute(trajectory);
        RCLCPP_INFO(logger, "Trajectory execution successful!");
    } else {
        response->doneflag = false;
        RCLCPP_ERROR(logger, "Trajectory planning failed!");
        return;
    }
    this_thread::sleep_for(5s);
    waypoints.clear();



    //pose ready
    move_group_interface.setNamedTarget("ready");
    planAndExecute(move_group_interface, plan, logger);
    this_thread::sleep_for(5s);

    printCurrentPose(move_group_interface, logger);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = make_shared<rclcpp::Node>("pose_move_test_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("pose_move_test_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    thread spinner = thread([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    auto arm_move_group_interface = MoveGroupInterface(node, "dsr_m0609");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto collision_object = addCollision(arm_move_group_interface);
    planning_scene_interface.applyCollisionObject(collision_object);

    arm_move_group_interface.setNamedTarget("ready");
    planAndExecute(arm_move_group_interface, plan, logger);
    this_thread::sleep_for(2s);

    current_pose = arm_move_group_interface.getCurrentPose().pose;
    printCurrentPose(arm_move_group_interface, logger);

    this_thread::sleep_for(10s);

    // Create the service
    auto trajectory_service = node->create_service<m0609_interfaces::srv::TrajectoryPlanRQ>(
        "trajectory_plan_rq",
        [&arm_move_group_interface, &logger, &plan](const std::shared_ptr<m0609_interfaces::srv::TrajectoryPlanRQ::Request> request,
                                             std::shared_ptr<m0609_interfaces::srv::TrajectoryPlanRQ::Response> response) {
            handleTrajectoryPlanRequest(request, response, arm_move_group_interface, logger, plan);
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    spinner.join();

    return 0;
}
