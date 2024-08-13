import rclpy
from rclpy.node import Node
from m0609_interfaces.srv import TrajectoryPlanRQ
from geometry_msgs.msg import PoseArray, Pose

class TrajectoryPlanClient(Node):
    def __init__(self):
        super().__init__('trajectory_plan_client')
        self.client = self.create_client(TrajectoryPlanRQ, 'trajectory_plan')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again ...')
            
        self.request = TrajectoryPlanRQ.Request()
    
    def send_request(self, points=None):
        if points is None:
            print("empty point")
            return 0
        else:
            pose_array = PoseArray()
            for point in points:
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose_array.poses.append(pose)
            self.request.points = pose_array
        
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            print(response)
            if response.doneflag:
                print("trajectory plan and execute done")
            else:
                self.get_logger().info('Empty response received')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    client = TrajectoryPlanClient()
    client.send_request(points=[[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0], [0.0, 0.0]])

    # Keep the node alive until the response is received
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
