#include <iostream>
#include <vector>
#include <tuple>
#include <string>
#include <algorithm>
#include <thread>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std::chrono_literals;
using namespace std;

geometry_msgs::msg::Pose target_pose;
geometry_msgs::msg::Pose current_pose;

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

void quaternionToRPY(const geometry_msgs::msg::Quaternion& quaternion){
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
}

auto addCollision(moveit::planning_interface::MoveGroupInterface& move_group_interface){
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

void printCurrentPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, rclcpp::Logger logger){
    current_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(logger, "Current position    : x= %f y= %f z= %f",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z);
    
    RCLCPP_INFO(logger, "Current orientation : x= %f y= %f z= %f w= %f",
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w);
}

char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}
void printsetting(rclcpp::Logger logger){
    RCLCPP_INFO(logger, "       move 10mm       |        move 1mm          ");
    RCLCPP_INFO(logger, "--------------------------------------------------");
    RCLCPP_INFO(logger, "     q     w     e     |     u     i     o        ");
    RCLCPP_INFO(logger, "     a     s     d     |     j     k     l        ");
    RCLCPP_INFO(logger, "                       |                          ");
    RCLCPP_INFO(logger, "    up    +y    down   |    up    +y    down      ");
    RCLCPP_INFO(logger, "    -x    -y     +x    |    -x    -y     +x       ");
    RCLCPP_INFO(logger, "--------------------------------------------------");
    RCLCPP_INFO(logger, " z = get current pose                             ");
    RCLCPP_INFO(logger, " m = end                                          ");
}

void moveBasedOnInput(moveit::planning_interface::MoveGroupInterface& move_group_interface, rclcpp::Logger logger) {
    char input;
    bool loop=false;
    if(rclcpp::ok()) loop=true;
    printsetting(logger);
    while (loop) {
        current_pose = move_group_interface.getCurrentPose().pose;
        target_pose=current_pose; 
        input = getch();
        switch (input) {
            case 'w':
                target_pose.position.y += 0.01;
                break;
            case 's':
                target_pose.position.y -= 0.01;
                break;
            case 'a':
                target_pose.position.x -= 0.01;
                break;
            case 'd':
                target_pose.position.x += 0.01;
                break;
            case 'q':
                target_pose.position.z += 0.01;
                break;
            case 'e':
                target_pose.position.z -= 0.01;
                break;
            case 'i':
                target_pose.position.y += 0.001;
                break;
            case 'k':
                target_pose.position.y -= 0.001;
                break;
            case 'j':
                target_pose.position.x -= 0.001;
                break;
            case 'l':
                target_pose.position.x += 0.001;
                break;
            case 'u':
                target_pose.position.z += 0.001;
                break;
            case 'o':
                target_pose.position.z -= 0.001;
                break;
            
            case 'z':
                printCurrentPose(move_group_interface,logger);
                printsetting(logger);
                break;
                
            case 'm':
                loop=false;
                break;
            default:
                continue;
        }

        std::vector<geometry_msgs::msg::Pose> waypoints;
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;

        waypoints.push_back(target_pose);
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.0) {
            move_group_interface.execute(trajectory);
            printCurrentPose(move_group_interface, logger);
        } else {
            RCLCPP_WARN(logger, "Failed to compute Cartesian path for the given input.");
        }

        std::this_thread::sleep_for(100ms); // Add a small delay to make movements smoother
    }
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
    target_pose = current_pose;

    printCurrentPose(arm_move_group_interface, logger);

    // Start the keyboard listener thread
    std::thread input_thread(moveBasedOnInput, std::ref(arm_move_group_interface), logger);
    input_thread.join();

    return 0;
}
