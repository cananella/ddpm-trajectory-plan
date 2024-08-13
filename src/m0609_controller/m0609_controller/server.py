import socket
import struct
import rclpy
import math
import time
from sensor_msgs.msg import JointState
from rclpy.node import Node

class JointState_Subscriber(Node):
    def __init__(self):
        super().__init__("dsr_controller")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription
        self.arm_data = []
    
    def listener_callback(self, msg: JointState):
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

def main(args=None):
    rclpy.init(args=args)
    jointsub = JointState_Subscriber()
    
    # Server setup
    ports = range(8080, 8083)  # Ports 8080 to 8082
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
    
    while True:
        print('Waiting for a connection...')
        connection, client_address = sock.accept()
        
        try:
            print(f'Connection from {client_address}')
            last_sent_time = time.time()
            interval = 1.0 / 15  # Time interval for sending data (15 times per second)

            while True:
                rclpy.spin_once(jointsub)
                # print(jointsub.arm_data)
                current_time = time.time()
                if current_time - last_sent_time >= interval:
                    # Sending coordinates
                    if jointsub.arm_data:
                        data = struct.pack('<6f', *jointsub.arm_data)  # Little-endian 6 float values
                        connection.sendall(data)
                        last_sent_time = current_time  # Update the last sent time
                else:
                    # Sleep for a short duration to prevent busy-waiting
                    time.sleep(0.01)  # Sleep for 10 milliseconds

        except Exception as e:
            print(f'Error: {e}')
        finally:
            connection.close()
            jointsub.destroy_node()
            rclpy.shutdown()    

if __name__ == '__main__':
    main()
