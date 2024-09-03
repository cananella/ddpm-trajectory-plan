import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.joint_state = JointState()

        self.joint_state.name = ['joint_1', 'joint_2', 'joint_4', 'joint_5' 'joint_3' , 'joint_6' , 'tool0-hand']
        self.joint_state.position = [0.0, 0.0, 0.0, 1.0 , 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0]


        self.angle = 0.0

    def timer_callback(self):
        self.joint_state.header = Header()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # 각 관절의 위치 업데이트
        self.joint_state.position[0] = math.sin(self.angle)
        self.joint_state.position[1] = math.sin(self.angle + math.pi / 2)
        self.joint_state.position[2] = math.sin(self.angle + math.pi)

        self.publisher_.publish(self.joint_state)
        
        self.angle += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
