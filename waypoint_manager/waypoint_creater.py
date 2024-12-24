import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

import math
import threading
from pynput import keyboard
import csv
import os
from datetime import datetime

class WaypointCreater(Node):
    def __init__(self):
        super().__init__('waypoint_creater')
        
        # Parameters
        self.declare_parameter('waypoints_file_name', '')
        self.waypoints_file_name = self.get_parameter('waypoints_file_name').value

        # Subscription
        self.pose_sub_ = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint/markers', 10)
        self.arrow_marker_pub_ = self.create_publisher(MarkerArray, 'waypoint/arrow_markers', 10)
        self.text_marker_pub = self.create_publisher(MarkerArray, 'waypoint/text_markers', 10)
        self.line_marker_pub = self.create_publisher(MarkerArray, 'waypoint/line_markers', 10)

        # Keyboard 
        self.get_logger().info('Press "s" to save to csv.')
        self.keyboard_thread = threading.Thread(target=self.keyboard_listerner)
        self.keyboard_thread.start()

        # Variables
        self.poses = []
        self.pose = None
        self.pose_id = 0
        self.skip_flag = 1
        self.event_flag = 0

        # Marker-related variables
        self.markers = MarkerArray()
        self.arrow_markers = MarkerArray()
        self.text_markers = MarkerArray()
        self.line_markers = MarkerArray()

    def pose_callback(self, msg):
        self.pose = msg.pose
        if self.pose is not None:
            # Save pose data for logging or processing
            pose_data = [
                str(self.pose_id),
                str(self.pose.position.x),
                str(self.pose.position.y),
                str(self.pose.position.z),
                str(self.pose.orientation.x),
                str(self.pose.orientation.y),
                str(self.pose.orientation.z),
                str(self.pose.orientation.w),
                int(self.skip_flag),
                int(self.event_flag)
            ]
            self.poses.append(pose_data)
            self.get_logger().info(f'Saved! ID: {self.pose_id}')

            # Create a PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = self.pose

            # Create and publish markers
            self.create_and_publish_markers(pose_stamped)

            self.pose_id += 1

        else:
            self.get_logger().warn('Warning: No pose received yet.')
    
    def create_and_publish_markers(self, pose_stamped):
        # Create a sphere marker
        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.header.stamp = pose_stamped.header.stamp
        marker.ns = "waypoints"
        marker.id = self.pose_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.markers.markers.append(marker)

        # Create an arrow marker
        arrow_marker = Marker()
        arrow_marker.header.frame_id = pose_stamped.header.frame_id
        arrow_marker.header.stamp = pose_stamped.header.stamp
        arrow_marker.ns = "waypoints_arrows"
        arrow_marker.id = self.pose_id + 1000
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        # Calculate start and end points for the arrow
        start_point = Point(
            x=pose_stamped.pose.position.x,
            y=pose_stamped.pose.position.y,
            z=pose_stamped.pose.position.z
        )
        end_point = Point()
        arrow_length = 0.3
        orientation = pose_stamped.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        )
        end_point.x = start_point.x + arrow_length * math.cos(yaw)
        end_point.y = start_point.y + arrow_length * math.sin(yaw)
        end_point.z = start_point.z

        arrow_marker.points.append(start_point)
        arrow_marker.points.append(end_point)

        # Set arrow scale and color
        arrow_marker.scale.x = 0.05
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.a = 1.0
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        self.arrow_markers.markers.append(arrow_marker)

        # Create a text marker for the waypoint ID
        text_marker = Marker()
        text_marker.header.frame_id = pose_stamped.header.frame_id
        text_marker.header.stamp = pose_stamped.header.stamp
        text_marker.ns = "waypoints_text"
        text_marker.id = self.pose_id + 2000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = pose_stamped.pose.position.x + 0.3
        text_marker.pose.position.y = pose_stamped.pose.position.y - 0.3
        text_marker.pose.position.z = pose_stamped.pose.position.z + 0.3
        text_marker.scale.z = 0.5
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.text = str(self.pose_id)
        self.text_markers.markers.append(text_marker)

        # Create a line marker to connect the previous waypoint
        if len(self.markers.markers) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = pose_stamped.header.frame_id
            line_marker.header.stamp = pose_stamped.header.stamp
            line_marker.ns = "waypoints_lines"
            line_marker.id = self.pose_id + 3000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02
            line_marker.color.a = 1.0
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0

            # Set the start and end points for the line
            prev_marker = self.markers.markers[-2]
            line_marker.points.append(prev_marker.pose.position)
            line_marker.points.append(pose_stamped.pose.position)

            self.line_markers.markers.append(line_marker)

        # Publish all markers
        self.marker_pub.publish(self.markers)
        self.arrow_marker_pub_.publish(self.arrow_markers)
        self.text_marker_pub.publish(self.text_markers)
        self.line_marker_pub.publish(self.line_markers)
    
    def keyboard_listerner(self):
        with keyboard.Listener(on_press=self.on_key_press) as listerner:
            listerner.join()
    
    def on_key_press(self, key):
        try:
            if key.char == 's':
                self.save_waypoints_to_csv()
        except AttributeError:
            pass
    
    def save_waypoints_to_csv(self):
        if self.waypoints_file_name:
            filename = self.waypoints_file_name
        else:
            filename = datetime.now().strftime('%Y%m%d%H%M%S') + '_waypoints.csv'
        
        path = os.path.join(os.path.curdir, 'src', 'waypoint_manager', 'waypoints', filename)  

        os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['id', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w', 'skip_flag', 'event_flag'])
            writer.writerows(self.poses)

        self.get_logger().info(f'Saved {len(self.poses)} waypoints to {path}.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCreater()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
