import socket
import struct
import rclpy
import math
import time
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
import threading

class JointState_Subscriber(Node):
    def __init__(self):
        super().__init__("dsr_controller")
        
        self.socket=self.connecting_socket()
        self.connection, self.client_address = self.socket.accept()
        
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            "display_planned_path",
            self.listener_callback,
            10
        )
        
        self.joint_subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_listener_callback,
            10
        )
        
        self.subscription
        self.joint_subscription
        self.arm_data = [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
        self.trajectory_send_data=None
        self.is_execute=False
        self.waypoint_size=None
        self.befor_joint_state=self.arm_data
        
        joint_recv_thread = threading.Thread(target=self.read_joint_state,daemon=True)
        joint_recv_thread.start()
        data_send_thread = threading.Thread(target=self.send_trajectory_plan,daemon=True)
        data_send_thread.start()
        
    def send_trajectory_plan(self):
        while True:
            if self.is_execute:
                self.send_trajectory()
                self.is_execute =False
                self.arm_data = None
                
        
    def joint_listener_callback(self, msg: JointState):
        data_list = []
        joint_name = msg.name
        joint1 = joint_name.index('joint_1')
        joint2 = joint_name.index('joint_2')
        joint3 = joint_name.index('joint_3')
        joint4 = joint_name.index('joint_4')
        joint5 = joint_name.index('joint_5')
        joint6 = joint_name.index('joint_6')

        for idx, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
                
        self.arm_data = [data_list[joint1], data_list[joint2], data_list[joint3], data_list[joint4], data_list[joint5], data_list[joint6]]
        for idx,data in enumerate(self.arm_data):
            if not data == self.befor_joint_state[idx]:
                self.is_execute=True
                break
            self.is_execute=False
        self.befor_joint_state=self.arm_data
        # print (self.arm_data)
        
    
    def read_joint_state(self):
        try:
            while True:
                # 데이터 수신 (joint positions)
                data = self.connection.recv(24)  # float 6개의 배열이므로 4 * 6 = 24 bytes
                if not data:
                    print("No data received. Closing connection.")
                    break

                floats = struct.unpack('f' * 6, data)
                self.robot_joint_state=np.deg2rad(floats)

        except Exception as e:
            print(f"An error occurred: {e}")
        
    def connecting_socket(self):
        ports = range(8080, 8083)  
        server_address = None
        sock = None
        
        for port in ports:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_address = ('127.0.0.1', port)
                sock.bind(server_address)
                sock.listen(1)
                print(f'Successfully bound to {server_address[0]} port {server_address[1]}')
                break
            
            except socket.error as e:
                print(f'Port {port} is in use or cannot be bound: {e}')
                sock.close()
                sock = None

        if sock is None:
            print('Failed to bind to any port in the specified range.')
            return

        print(f'Starting server on {server_address[0]} port {server_address[1]}')
        return sock
        
    def listener_callback(self, msg: DisplayTrajectory):
        print(" msg sub ")
        t:RobotTrajectory=msg.trajectory[0]
        points=t.joint_trajectory.points
        dataset=[]
        for point in points:
            point:JointTrajectoryPoint
            pos=np.round(np.rad2deg(point.positions),2)
            vel=np.round(np.rad2deg(point.velocities),2)
            acc=np.round(np.rad2deg(point.accelerations),2)
            dataset.append([pos,vel,acc])
                
        self.trajectory_send_data = [item for sublist in dataset for subsublist in sublist for item in subsublist]
        self.waypoint_size=len(self.trajectory_send_data)

        
            
    def send_trajectory(self):
        if not self.waypoint_size == None :
            if self.is_execute:
                try:
                    self.connection.sendall(struct.pack('<I', self.waypoint_size))
                    for value in self.trajectory_send_data:
                        self.connection.sendall(struct.pack('<f', value))
                except Exception as e:
                    print(f"Error sending data: {e}")
                self.is_execute=False
                self.waypoint_size=None
        


def main(args=None):
    rclpy.init(args=args)
    jointsub = JointState_Subscriber()
    rclpy.spin(jointsub)
    jointsub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
