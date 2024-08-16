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
    float befor_data[6];

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

    ssize_t bytes_received = recv(sock, data, sizeof(data), 0);
    if (bytes_received < 0) std::cerr << "recv error" << std::endl;
    else if (bytes_received == 0) std::cout << "Server closed connection" << std::endl;
    Drfl.movej(data, 40, 40);

    auto beforestate=Drfl.GetRobotState();
    cout<<beforestate<<"\n";
    auto last_update_time = std::chrono::steady_clock::now();

    while (bLoop) {
        g_mStat = false;
        g_Stop = false;
        auto robotstate = Drfl.GetRobotState();
        if (beforestate != robotstate) {
            std::cout << "robot state change  " << beforestate << " --> " << robotstate << "\n";
            beforestate = robotstate;
        }
        ssize_t bytes_received = recv(sock, data, sizeof(data), 0);
        if (bytes_received < 0) {
            std::cerr << "recv error" << std::endl;
            break;
        }
        else if (bytes_received == 0) {
            std::cout << "Server closed connection" << std::endl;
            break;
        }

        bool is_same = true;
        for (int idx = 0; idx < 6; idx++) {
            if (data[idx] != befor_data[idx]) {
                is_same = false;
                break;
            }
        }

        auto current_time = std::chrono::steady_clock::now();
        auto duration_since_last_update = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_update_time);

        if (is_same) {
            if (duration_since_last_update.count() >= 10) {
                Drfl.amovej(data, 80, 80);
                last_update_time = current_time;
                cout<<"joint is same \n";
            }
        } else {
            Drfl.amovej(data, 80, 80);
            last_update_time = current_time;
            for (int idx = 0; idx < 6; idx++) {
                befor_data[idx] = data[idx];
            }
        }
    }

    close(sock);
    return 0;
}



//g++ -c test.cpp && g++ -o test test.o ../../library/Linux/64bits/22.04/libDRFL.a /usr/lib/x86_64-linux-gnu/libPocoFoundation.so /usr/lib/x86_64-linux-gnu/libPocoNet.so && rm test.o && cp test ~/
