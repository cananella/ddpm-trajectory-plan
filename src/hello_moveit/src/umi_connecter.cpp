#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <mutex>
#include <algorithm>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std;




geometry_msgs::msg::Pose target_pose;
geometry_msgs::msg::Pose current_pose;
const int buffer_size = 1024;
char buffer[buffer_size];


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

struct Pose {
    double x, y, z, rx, ry, rz;
};

struct Position{
    double x,y,z,ox,oy,oz,ow;
};

Pose c_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
Pose actual_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
mutex pose_mutex;
queue<Position> waypoint_list;

vector<double> parse_pose(const string &pose_str) {
    vector<double> pose_values;
    
    // 1. 괄호 제거 (pose_str에서 양쪽 괄호 [] 제거)
    string cleaned_pose_str = pose_str;
    cleaned_pose_str.erase(remove(cleaned_pose_str.begin(), cleaned_pose_str.end(), '['), cleaned_pose_str.end());
    cleaned_pose_str.erase(remove(cleaned_pose_str.begin(), cleaned_pose_str.end(), ']'), cleaned_pose_str.end());

    // 2. 쉼표를 기준으로 값을 나누어 파싱
    stringstream ss(cleaned_pose_str);
    string item;
    while (getline(ss, item, ',')) {
        try {
            pose_values.push_back(stod(item));  // 문자열을 double로 변환
        } catch (const invalid_argument& e) {
            cerr << "Invalid number format in pose string: " << item << endl;
        }
    }

    return pose_values;
}

bool handle_client(int client_socket, moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan, rclcpp::Logger logger) {
    string command(buffer);
    cout << "Received command: " << command << endl;

    if (command.find("SERVO_L") == 0) {
        // ex: "SERVO_L [0.3,0.2,0.5,0,1.57,0]"
        stringstream ss(command);
        string cmd_type;
        getline(ss, cmd_type, ' '); 

        string pose_str;
        getline(ss, pose_str); 

        // cout << "Parsed command: " << cmd_type << ", pose_str: " << pose_str << endl;

        vector<double> pose_values = parse_pose(pose_str.substr(1, pose_str.length() - 2));
        // cout << "Parsed Pose Values: ";
        // for (double val : pose_values) {
        //     cout << val << " ";
        // }
        // cout << endl;

        {
            lock_guard<mutex> lock(pose_mutex);
            if (pose_values.size() == 6) {
                actual_pose.x = pose_values[0];
                actual_pose.y = pose_values[1];
                actual_pose.z = pose_values[2];
                actual_pose.rx = pose_values[3];
                actual_pose.ry = pose_values[4];
                actual_pose.rz = pose_values[5];
            }
        }
        bool update_pose = true;
        double tolerance = 2e-6;

        if (abs(actual_pose.x - c_pose.x) < tolerance &&
            abs(actual_pose.y - c_pose.y) < tolerance &&
            abs(actual_pose.z - c_pose.z) < tolerance &&
            abs(actual_pose.rx - c_pose.rx) < tolerance &&
            abs(actual_pose.ry - c_pose.ry) < tolerance &&
            abs(actual_pose.rz - c_pose.rz) < tolerance
            ) update_pose = false;

        // cout << "Current Pose (c_pose): \n";
        // cout << "  Position: [" << c_pose.x << ", " << c_pose.y << ", " << c_pose.z << "]\n";
        // cout << "  Rotation (RPY): [" << c_pose.rx << ", " << c_pose.ry << ", " << c_pose.rz << "]\n\n";

        // cout << "New Pose (actual_pose): \n";
        // cout << "  Position: [" << actual_pose.x << ", " << actual_pose.y << ", " << actual_pose.z << "]\n";
        // cout << "  Rotation (RPY): [" << actual_pose.rx << ", " << actual_pose.ry << ", " << actual_pose.rz << "]\n\n";
        // cout << "ServoL command processed. New pose: "
        //      << actual_pose.x << ", "
        //      << actual_pose.y << ", "
        //      << actual_pose.z << ", "
        //      << actual_pose.rx << ", "
        //      << actual_pose.ry << ", "
        //      << actual_pose.rz << endl;
        if(update_pose){
            Position new_waypoint;
            tf2::Quaternion q;
            q.setRPY(pose_values[3], pose_values[4], pose_values[5]);

            new_waypoint.x = pose_values[0];
            new_waypoint.y = pose_values[1];
            new_waypoint.z = pose_values[2];
            new_waypoint.ox = q.x();
            new_waypoint.oy = q.y();
            new_waypoint.oz = q.z();
            new_waypoint.ow = q.w();

            waypoint_list.push(new_waypoint);  // 큐에 목표 좌표 추가
        }

    }

    else if(command.find("SERVO_J") == 0){ 
        stringstream ss(command);
        string cmd_type;
        getline(ss, cmd_type, ' ');

        string pose_str;
        getline(ss, pose_str); 

        vector<double> joint_values = parse_pose(pose_str.substr(1, pose_str.length() - 2));

        move_group_interface.setJointValueTarget(joint_values); 
        planAndExecute(move_group_interface, plan, logger);

    } 

    else if (command.find("GET_ACTUAL_TCP_POSE") == 0) {
        current_pose = move_group_interface.getCurrentPose().pose;
        tf2::Quaternion q(current_pose.orientation.x,current_pose.orientation.y,
                            current_pose.orientation.z,current_pose.orientation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        string pose_data;
        {
            lock_guard<mutex> lock(pose_mutex);
            pose_data = to_string(current_pose.position.x) + "," +
                        to_string(current_pose.position.y) + "," +
                        to_string(current_pose.position.z) + "," +
                        to_string(roll) + "," +
                        to_string(pitch) + "," +
                        to_string(yaw);

        }
        c_pose.x = current_pose.position.x;
        c_pose.y = current_pose.position.y;
        c_pose.z = current_pose.position.z;
        c_pose.rx = roll;
        c_pose.ry = pitch;
        c_pose.rz = yaw;
            
        

        send(client_socket, pose_data.c_str(), pose_data.length(), 0);
        // cout << "Sent actual TCP pose: " << pose_data << endl;

    } 
    
    else if (command.find("STOP") == 0) {
        cout << "Stopping client connection." << endl;
        return false;
    } 
    
    else {
        cerr << "Unknown command received." << endl;
    }

    return true;
}

void processWaypoints(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan, rclcpp::Logger logger) {
    while (true) {
        Position next_waypoint;
        {
            lock_guard<mutex> lock(pose_mutex);
            if (!waypoint_list.empty()) {
                next_waypoint = waypoint_list.front();
                waypoint_list.pop();
            } else {
                continue;  // 큐가 비어있으면 다음 반복으로 넘어감
            }
        }

        target_pose.position.x = next_waypoint.x;
        target_pose.position.y = next_waypoint.y;
        target_pose.position.z = next_waypoint.z;
        target_pose.orientation.x = next_waypoint.ox;
        target_pose.orientation.y = next_waypoint.oy;
        target_pose.orientation.z = next_waypoint.oz;
        target_pose.orientation.w = next_waypoint.ow;

        move_group_interface.setPoseTarget(target_pose);
        planAndExecute(move_group_interface, plan, logger);
    }
}

void printCurrentPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, rclcpp::Logger logger) {
    current_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(logger, "Current pose: %lf %lf %lf %lf %lf %lf %lf",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z,
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w);
}

// 메인 소켓 서버 함수
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<rclcpp::Node>("umi_connetor", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger =rclcpp::get_logger("umi_connetor");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    thread spinner = thread([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    auto arm_move_group_interface = MoveGroupInterface(node, "dsr_m0609");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    arm_move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_move_group_interface.setPlannerId("PTP"); 
    arm_move_group_interface.setPlanningTime(20.0); 

    thread waypoint_thread(processWaypoints, ref(arm_move_group_interface), ref(plan), logger);


    int server_fd, client_socket;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        cerr << "Socket creation error." << endl;
        return -1;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;  
    server_addr.sin_port = htons(6006);        

    if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        cerr << "Socket bind failed." << endl;
        return -1;
    }

    if (listen(server_fd, 3) < 0) {
        cerr << "Listen failed." << endl;
        return -1;
    }
    cout << "Server listening on port 6006..." << endl;

    if ((client_socket = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_len)) < 0) {
        cerr << "Client accept failed." << endl;
        return -1;
    }
    cout << "Client connected." << endl;

    while (true) {
        memset(buffer, 0, buffer_size);
        int bytes_received = recv(client_socket, buffer, buffer_size, 0);
        if (bytes_received <= 0) {
            cerr << "Client disconnected or error occurred." << endl;
            break;
        }

        if (!handle_client(client_socket, arm_move_group_interface, plan, logger)) {
            break;
        }
    }

    close(server_fd);
    waypoint_thread.join();
    return 0;
}
