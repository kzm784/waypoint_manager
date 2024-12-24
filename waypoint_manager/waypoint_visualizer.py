import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
import csv
import math
from datetime import datetime

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')

        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('map_frame', 'map')

        self.markers = MarkerArray()
        self.text_markers = MarkerArray()
        self.line_markers = MarkerArray()
        self.arrow_markers = MarkerArray()

        filename = self.get_parameter('waypoints_csv').get_parameter_value().string_value
        map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        if filename == '':
            self.get_logger().error("Waypoint file not specified")
            return

        self.load_waypoints_from_csv(filename, map_frame)

        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint/markers', 10)
        self.text_marker_pub = self.create_publisher(MarkerArray, 'waypoint/text_markers', 10)
        self.line_marker_pub = self.create_publisher(MarkerArray, 'waypoint/line_markers', 10)
        self.arrow_marker_pub = self.create_publisher(MarkerArray, 'waypoint/arrow_markers', 10)

        self.waypoint_id_sub = self.create_subscription(Int32, 'next_waypoint_id', self.waypoint_id_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_waypoint_id = None

    def load_waypoints_from_csv(self, filename, map_frame):
        try:
            with open(filename, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)  # Skip the header row

                for index, row in enumerate(reader):
                    if len(row) == 10:  # Ensure the row has 10 columns
                        pose = PoseStamped()
                        pose.pose.position.x = float(row[1])
                        pose.pose.position.y = float(row[2])
                        pose.pose.position.z = float(row[3])

                        pose.pose.orientation.x = float(row[4])
                        pose.pose.orientation.y = float(row[5])
                        pose.pose.orientation.z = float(row[6])
                        pose.pose.orientation.w = float(row[7])

                        pose.header.frame_id = map_frame
                        pose.header.stamp = self.get_current_time()

                        waypoint_id = int(row[0])

                        # Create a marker for each waypoint
                        marker = Marker()
                        marker.header.frame_id = map_frame
                        marker.header.stamp = self.get_current_time()
                        marker.ns = "waypoints"
                        marker.id = waypoint_id
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose = pose.pose
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
                        text_marker.header.frame_id = map_frame
                        text_marker.header.stamp = self.get_current_time()
                        text_marker.ns = "waypoints_text"
                        text_marker.id = waypoint_id + 1000
                        text_marker.type = Marker.TEXT_VIEW_FACING
                        text_marker.action = Marker.ADD
                        text_marker.pose.position.x = pose.pose.position.x + 0.2
                        text_marker.pose.position.y = pose.pose.position.y - 0.2
                        text_marker.pose.position.z = pose.pose.position.z + 0.3
                        text_marker.scale.z = 0.5
                        text_marker.color.a = 1.0
                        text_marker.color.r = 0.0
                        text_marker.color.g = 0.0
                        text_marker.color.b = 0.0
                        text_marker.text = str(waypoint_id)
                        self.text_markers.markers.append(text_marker)

                        # Create a arrow_marker to displey the wayopint orientation
                        arrow_marker = Marker()
                        arrow_marker.header.frame_id = map_frame
                        arrow_marker.header.stamp = self.get_current_time()
                        arrow_marker.ns = "waypoints_arrows"
                        arrow_marker.id = waypoint_id + 2000 
                        arrow_marker.type = Marker.ARROW
                        arrow_marker.action = Marker.ADD

                        start_point = Point(
                            x = pose.pose.position.x,
                            y = pose.pose.position.y,
                            z = pose.pose.position.z
                        )
                        end_point = Point()
                        arrow_length = 0.3
                        orientation = pose.pose.orientation
                        yaw = math.atan2(
                            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                            1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
                        )
                        end_point.x = start_point.x + arrow_length * math.cos(yaw)
                        end_point.y = start_point.y + arrow_length * math.sin(yaw)
                        end_point.z = start_point.z

                        arrow_marker.points.append(start_point)
                        arrow_marker.points.append(end_point)

                        arrow_marker.scale.x = 0.05
                        arrow_marker.scale.y = 0.1
                        arrow_marker.scale.z = 0.1
                        arrow_marker.color.a = 1.0
                        arrow_marker.color.r = 1.0
                        arrow_marker.color.g = 0.0
                        arrow_marker.color.b = 0.0
                        self.arrow_markers.markers.append(arrow_marker)

                        # Create line marker (skip the first waypoint)
                        if index > 0:
                            line_marker = Marker()
                            line_marker.header.frame_id = map_frame
                            line_marker.header.stamp = self.get_current_time()
                            line_marker.ns = "waypoints_lines"
                            line_marker.id = waypoint_id + 3000
                            line_marker.type = Marker.LINE_STRIP
                            line_marker.action = Marker.ADD
                            line_marker.scale.x = 0.02
                            line_marker.color.a = 1.0
                            line_marker.color.r = 0.0
                            line_marker.color.g = 1.0
                            line_marker.color.b = 0.0

                            # Set the points for the line
                            line_marker.points.append(self.markers.markers[index - 1].pose.position)
                            line_marker.points.append(pose.pose.position)

                            self.line_markers.markers.append(line_marker)

                    else:
                        self.get_logger().error(f"Unexpected row length: {len(row)}. Row content: {row}")

        except FileNotFoundError:
            self.get_logger().error(f"Error! File {filename} cannot be opened")

    def get_current_time(self):
        now = datetime.now()
        return rclpy.time.Time(seconds=now.timestamp()).to_msg()

    def waypoint_id_callback(self, msg):
        new_waypoint_id = msg.data
        self.get_logger().info(f"Received waypoint ID: {new_waypoint_id}")

        # If there's a previous waypoint ID, reset its color
        if self.current_waypoint_id is not None:
            for marker in self.markers.markers:
                if marker.id == self.current_waypoint_id:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0

            for text_marker in self.text_markers.markers:
                if text_marker.id == self.current_waypoint_id + 1000:
                    text_marker.color.r = 0.0
                    text_marker.color.g = 0.0
                    text_marker.color.b = 0.0

            # Reset line color
            for line_marker in self.line_markers.markers:
                if line_marker.id == self.current_waypoint_id + 2000:
                    line_marker.color.r = 0.0
                    line_marker.color.g = 1.0
                    line_marker.color.b = 0.0

        # Update the color of the new waypoint ID to red
        self.current_waypoint_id = new_waypoint_id
        for marker in self.markers.markers:
            if marker.id == new_waypoint_id:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

        for text_marker in self.text_markers.markers:
            if text_marker.id == new_waypoint_id + 1000:
                text_marker.color.r = 1.0
                text_marker.color.g = 0.0
                text_marker.color.b = 0.0

        # Change the color of the line between the previous and current waypoint to red
        if self.current_waypoint_id is not None:
            for line_marker in self.line_markers.markers:
                if line_marker.id == new_waypoint_id + 2000:
                    line_marker.color.r = 1.0
                    line_marker.color.g = 0.0
                    line_marker.color.b = 0.0

    def timer_callback(self):
        self.marker_pub.publish(self.markers)
        self.text_marker_pub.publish(self.text_markers)
        self.line_marker_pub.publish(self.line_markers)
        self.arrow_marker_pub.publish(self.arrow_markers)

def main(args=None):
    rclpy.init(args=args)

    node = WaypointVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
