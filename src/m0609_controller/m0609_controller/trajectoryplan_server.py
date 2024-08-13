import rclpy
from rclpy.node import Node
from m0609_interfaces.srv import TrajectoryPlanRQ
from geometry_msgs.msg import PoseArray


class TrajectroyPlanService(Node):
    def __init__(self):
        super().__init__('trajectory_plan_service')
        self.srv = self.create_service(TrajectoryPlanRQ, 'trajectory_plan',self.trajectory_callback)
        self.data_arrat=[]
        
        
    def trajectory_callback(self, request:TrajectoryPlanRQ.Request, response:TrajectoryPlanRQ.Response):
        if request.points:
            response.doneflag = True
            print(request.points)
            print("Servise Request")
        return response
    


def main(args=None):
    rclpy.init(args=args)
    service = TrajectroyPlanService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()