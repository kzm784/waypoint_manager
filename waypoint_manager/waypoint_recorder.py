import os
import csv
import time
from math import sqrt, pow
from pynput import keyboard
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('wayopint_recorder')

        # Parameters
        self.declare_parameter('saving_waypoints_file_name', '')
        self.declare_parameter('use_keyboard', False)
        self.declare_parameter('use_joy', False)
        self.declare_parameter('save_button', 0)
        self.declare_parameter('quit_button', 1)
        self.declare_parameter('erace_button', 2)
        self.declare_parameter('auto_record', False)
        self.declare_parameter('waypoint_interval', 1.0)
        self.declare_parameter('save_interval', 0.5)
        self.saving_waypoints_file_name = self.get_parameter('saving_waypoints_file_name').value
        self.use_keyboard = self.get_parameter('use_keyboard').value
        self.use_joy = self.get_parameter('use_joy').value
        self.save_button = self.get_parameter('save_button').value
        self.quit_button = self.get_parameter('quit_button').value
        self.erace_button = self.get_parameter('erace_button').value
        self.auto_record = self.get_parameter('auto_record').value
        self.waypoint_interval = self.get_parameter('waypoint_interval').value
        self.save_interval = self.get_parameter('save_interval').value

        # Subscription
        self.create_subscription(PoseWithCovarianceStamped, 'current_pose', self.pose_callback, 10)        

        # Publisher
        self.pose_publisher = self.create_publisher(PoseStamped, '/navigation_manager/waypoint_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint/markers', 10)
        self.text_marker_pub = self.create_publisher(MarkerArray, 'waypoint/text_markers', 10)
        self.line_marker_pub = self.create_publisher(MarkerArray, 'waypoint/line_markers', 10)

        # Keyboard save mode
        if self.use_keyboard:
            self.get_logger().info('Press "s" to save the current pose, "q" to quit and save to csv.')
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.start()
        
        # Joy save mode
        if self.use_joy:
            self.get_logger().info(f'Press "{self.save_button}" to save the current pose, "{self.quit_button}" to quit and save to csv.')
            self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Variables
        self.poses = []
        self.current_pose = None
        self.pose_id = 0
        self.skip_flag = 1
        self.prev_joy_buttons = [0] * 12
        self.last_pose = PoseStamped()
        self.last_save_time = time.time()
        self.last_erase_time = time.time()

        # Marker-related variables
        self.markers = MarkerArray()
        self.text_markers = MarkerArray()
        self.line_markers = MarkerArray()

    def keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_key_press) as listener:
            listener.join()

    def on_key_press(self, key):
        try:
            if key.char == 's':
                self.try_save_pose()
            elif key.char == 'q':
                self.save_poses_to_csv()
                self.destroy_node()
                rclpy.shutdown()
        except AttributeError:
            pass

    def joy_callback(self, msg):
        # Check if save_button is pressed
        if msg.buttons[self.save_button] == 1 and self.prev_joy_buttons[self.save_button] == 0:
            self.try_save_pose()

        # Check if quit_button is pressed
        if msg.buttons[self.quit_button] == 1 and self.prev_joy_buttons[self.quit_button] == 0:
            self.save_poses_to_csv()
            self.destroy_node()
            rclpy.shutdown()

        # Check if erase_button is pressed
        if msg.buttons[self.erace_button] == 1 and self.prev_joy_buttons[self.erace_button] == 0:
            self.try_erase_pose()

        # Update the previous button states
        self.prev_joy_buttons = list(msg.buttons)

    def try_save_pose(self):
        current_time = time.time()
        if current_time - self.last_save_time >= self.save_interval:
            self.save_pose()
            self.last_save_time = current_time
        else:
            self.get_logger().info(f'Ignored save request. Wait at least {self.save_interval} seconds between saves.')

    def try_erase_pose(self):
        current_time = time.time()
        if current_time - self.last_erase_time >= self.save_interval:
            self.erase_last_pose()
            self.last_erase_time = current_time
        else:
            self.get_logger().info(f'Ignored erase request. Wait at least {self.save_interval} seconds between erases.')

    def save_pose(self):
        if self.current_pose is not None:
            pose_data = [
                str(self.pose_id),
                str(self.current_pose.position.x),
                str(self.current_pose.position.y),
                str(self.current_pose.position.z),
                str(self.current_pose.orientation.x),
                str(self.current_pose.orientation.y),
                str(self.current_pose.orientation.z),
                str(self.current_pose.orientation.w),
                int(self.skip_flag)
            ]
            self.poses.append(pose_data)
            self.get_logger().info(f'saved! ID: {self.pose_id}')

            # Publish the saved pose as PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = self.current_pose
            self.pose_publisher.publish(pose_stamped)

            # Create and publish markers
            self.create_and_publish_markers(pose_stamped)

            # Increment pose ID after publishing the markers
            self.pose_id += 1

            # Memorize the last recorded pose
            self.last_pose.pose = self.current_pose

        else:
            self.get_logger().warn('Warning: No pose received yet.')

    def create_and_publish_markers(self, pose_stamped):
        # Create a marker for the waypoint
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

        # Create a text marker to display the waypoint ID
        text_marker = Marker()
        text_marker.header.frame_id = pose_stamped.header.frame_id
        text_marker.header.stamp = pose_stamped.header.stamp
        text_marker.ns = "waypoints_text"
        text_marker.id = self.pose_id + 1000
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

        # Create a line marker connecting to the previous waypoint, if applicable
        if len(self.markers.markers) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = pose_stamped.header.frame_id
            line_marker.header.stamp = pose_stamped.header.stamp
            line_marker.ns = "waypoints_lines"
            line_marker.id = self.pose_id + 2000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02
            line_marker.color.a = 1.0
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0

            # Set the points for the line
            line_marker.points.append(self.markers.markers[-2].pose.position)
            line_marker.points.append(pose_stamped.pose.position)

            self.line_markers.markers.append(line_marker)

        # Publish all markers
        self.marker_pub.publish(self.markers)
        self.text_marker_pub.publish(self.text_markers)
        self.line_marker_pub.publish(self.line_markers)

    def erase_last_pose(self):
        if self.poses:
            # Remove the last saved pose
            last_pose_id = self.pose_id - 1
            self.poses.pop()
            self.pose_id -= 1
            self.get_logger().info(f'erased! ID: {last_pose_id}')

            # Remove the corresponding marker
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = "waypoints"
            delete_marker.id = last_pose_id
            delete_marker.action = Marker.DELETE
            self.markers.markers = [marker for marker in self.markers.markers if marker.id != last_pose_id]
            self.markers.markers.append(delete_marker)

            # Remove the corresponding text marker
            delete_text_marker = Marker()
            delete_text_marker.header.frame_id = "map"
            delete_text_marker.header.stamp = self.get_clock().now().to_msg()
            delete_text_marker.ns = "waypoints_text"
            delete_text_marker.id = last_pose_id + 1000
            delete_text_marker.action = Marker.DELETE
            self.text_markers.markers = [text_marker for text_marker in self.text_markers.markers if text_marker.id != last_pose_id + 1000]
            self.text_markers.markers.append(delete_text_marker)

            # Remove the corresponding line marker
            delete_line_marker = Marker()
            delete_line_marker.header.frame_id = "map"
            delete_line_marker.header.stamp = self.get_clock().now().to_msg()
            delete_line_marker.ns = "waypoints_lines"
            delete_line_marker.id = last_pose_id + 2000
            delete_line_marker.action = Marker.DELETE
            self.line_markers.markers = [line_marker for line_marker in self.line_markers.markers if line_marker.id != last_pose_id + 2000]
            self.line_markers.markers.append(delete_line_marker)

            # Publish updated markers
            self.marker_pub.publish(self.markers)
            self.text_marker_pub.publish(self.text_markers)
            self.line_marker_pub.publish(self.line_markers)
        else:
            self.get_logger().warn('No poses to erase.')

    def save_poses_to_csv(self):
        if self.saving_waypoints_file_name:
            filename = self.saving_waypoints_file_name
        else:
            filename = datetime.now().strftime('%Y%m%d%H%M%S') + '_waypoints.csv'
        
        path = os.path.join(os.path.curdir, 'src', 'waypoint_manager', 'waypoints', filename)  

        os.makedirs(os.path.dirname(path), exist_ok=True)

        with open(path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['id', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w', 'skip_flag'])
            writer.writerows(self.poses)

        self.get_logger().info(f'Saved {len(self.poses)} poses to {path}.')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        dist_between_waypoints = None
        if self.auto_record:
            if self.last_pose.pose is None:
                self.last_pose.pose = self.current_pose
            else:
                dist_between_waypoints = sqrt(pow(self.current_pose.position.x - self.last_pose.pose.position.x, 2) + 
                                            pow(self.current_pose.position.y - self.last_pose.pose.position.y, 2) + 
                                            pow(self.current_pose.position.z - self.last_pose.pose.position.z, 2))
                if dist_between_waypoints >= self.waypoint_interval:
                    self.save_pose()
                    self.last_pose.pose = self.current_pose


def main(args=None):
    rclpy.init(args=args)
    pose_recorder = WaypointRecorder()

    try:
        rclpy.spin(pose_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
