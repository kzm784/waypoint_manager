import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

import yaml

class WaypointCreater(Node):
    def __init__(self):
        super().__init__('waypoint_creater')
        
        # Parameters
        self.declare_parameter('waypoints_path', '')
        self.waypoints_path = self.get_parameter('waypoints_path').value

        # Subscription
        self.pose_sub_ =  self.create_subscription(PoseWithCovarianceStamped, 'pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCreater()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()