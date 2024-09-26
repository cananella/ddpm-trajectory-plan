#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cassert>
#include <vector>
#include <tuple>
#include <thread>  // std::thread
#include <mutex>   // std::mutex

#include "../../include/DRFLEx.h"
using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>

CDRFLEx Drfl;
bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
bool moving = FALSE;

bool bAlterFlag = FALSE;

mutex mtx; // 스레드 간 데이터 보호를 위한 mutex

void send_joint_positions(int sock) {
    while (true) {
        mtx.lock(); // 데이터를 보호하기 위해 mutex 잠금
        auto current_joint_pos = Drfl.get_current_posj();
        mtx.unlock(); // 데이터 접근이 끝나면 mutex 해제

        // 현재 관절 위치 데이터를 소켓을 통해 전송
        float joint_pos_data[6];
        for(int i = 0; i < 6; ++i) {
            joint_pos_data[i] = current_joint_pos->_fPosition[i];
        }

        ssize_t bytes_sent = send(sock, joint_pos_data, sizeof(joint_pos_data), 0);
        if (bytes_sent < 0) {
            std::cerr << "Error sending joint position data" << std::endl;
            break;
        } 
        // else {
        //     std::cout << "Sent current joint positions: ";
        //     for (auto &pos : joint_pos_data) {
        //         std::cout << pos << " ";
        //     }
        //     std::cout << std::endl;
        // }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 1초 대기 후 반복
    }

    close(sock); // 스레드 종료 시 소켓 닫기
}

int main() {
    while (!Drfl.open_connection("192.168.137.100")) {
        std::cerr << "Failed to connect. Retrying in 3 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }


    Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
    Drfl.SetRobotControl(CONTROL_SERVO_ON);

    SYSTEM_VERSION tSysVerion = { '\0' };
    Drfl.get_system_version(&tSysVerion);
    Drfl.setup_monitoring_version(1);
    Drfl.set_robot_control(CONTROL_SERVO_ON);
    Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);
    cout << "System version: " << tSysVerion._szController << endl;
    cout << "Library version: " << Drfl.get_library_version() << endl;

    assert(Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS));
    assert(Drfl.set_robot_system(ROBOT_SYSTEM_REAL));

    bool bLoop = TRUE;

    int sock = 0;
    struct sockaddr_in serv_addr;
    float data[6];
    uint32_t data_length = 0;


    bool connected = false;

    while (!connected) {
        for (int port = 8080; port <= 8082; ++port) {
            if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                std::cerr << "Socket creation error." << std::endl;
                continue;
            }

            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(port);

            if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
                std::cerr << "Invalid address / Address not supported." << std::endl;
                close(sock);
                continue;
            }

            if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                std::cerr << "Connection to port " << port << " failed." << std::endl;
                close(sock);
                continue;
            }

            std::cout << "Connected to port " << port << std::endl;
            connected = true;
            break;
        }

        if (!connected) {
            std::cerr << "Failed to connect to any port in the range 8080-8082. Retrying in 3 seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    auto beforestate=Drfl.GetRobotState();
    cout<<"current robot state is : "<<beforestate<<"\n";
    auto last_update_time = std::chrono::steady_clock::now();
    
    std::thread sender_thread(send_joint_positions, sock);


    while (bLoop) {
        g_mStat = false;
        g_Stop = false;
        auto robotstate = Drfl.GetRobotState();
        if (beforestate != robotstate) {
            cout << "robot state change  " << beforestate << " --> " << robotstate << "\n";
            beforestate = robotstate;
        }

        int valread = read(sock, &data_length, sizeof(data_length));
        int waypoint_size = data_length/3/6;
        cout<<"waypoint size : "<<waypoint_size<<endl;

        if (valread != sizeof(data_length)) {
            std::cerr << "read error" << std::endl;
            return -1;
        }   
        float point_data[waypoint_size][6];
        float vel_data[waypoint_size][6];
        float acc_data[waypoint_size][6];

        for (int i = 0; i < waypoint_size; i++) {
            float value;
            for(int j = 0; j < 6; j++){
                valread = read(sock, &value, sizeof(value));
                if (valread != sizeof(value)) {
                    std::cerr << "read error" << std::endl;
                    return -1;
                }
                point_data[i][j]=value;
            }

            for(int j = 0; j < 6; j++){
                valread = read(sock, &value, sizeof(value));
                if (valread != sizeof(value)) {
                    std::cerr << "read error" << std::endl;
                    return -1;
                }
                vel_data[i][j]=value;
            }

            for(int j = 0; j < 6; j++){
                valread = read(sock, &value, sizeof(value));
                if (valread != sizeof(value)) {
                    std::cerr << "read error" << std::endl;
                    return -1;
                }
                acc_data[i][j]=value;
            }
        }

        cout<<"--------------------------------------------\n";
        for (int i = 0; i < waypoint_size; i++){
            cout <<i<<" waypoint"<<endl;
            cout <<"joint pos : "<< point_data[i][0]<<" , "<< point_data[i][1]<<" , "<< point_data[i][2]<<" , "<< point_data[i][3]<<" , "<< point_data[i][4]<<" , "<< point_data[i][5]<<" , " << " \n";
            cout <<"joint val : "<< vel_data[i][0]<<" , "<< vel_data[i][1]<<" , "<< vel_data[i][2]<<" , "<< vel_data[i][3]<<" , "<< vel_data[i][4]<<" , "<< vel_data[i][5]<<" , " << " \n";
            cout <<"joint acc : "<< acc_data[i][0]<<" , "<< acc_data[i][1]<<" , "<< acc_data[i][2]<<" , "<< acc_data[i][3]<<" , "<< acc_data[i][4]<<" , "<< acc_data[i][5]<<" , " << " \n"; 
        }
        cout<<"--------------------------------------------\n";

        Drfl.amovesj(point_data,waypoint_size,10,10,0.1);
        
    }
    sender_thread.join(); // 스레드 종료를 대기

    close(sock);
    return 0;
}



//g++ -c trajectory_plan_client.cpp && g++ -o trajectory_plan_client trajectory_plan_client.o ../../library/Linux/64bits/22.04/libDRFL.a /usr/lib/x86_64-linux-gnu/libPocoFoundation.so /usr/lib/x86_64-linux-gnu/libPocoNet.so && rm trajectory_plan_client.o && cp trajectory_plan_client ~/
