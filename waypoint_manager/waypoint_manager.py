import csv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, CancelResponse
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # Parameters
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('loop_enable', False)
        self.declare_parameter('loop_count', 0)
        waypoints_csv = self.get_parameter('waypoints_csv').value
        self.loop_enable = self.get_parameter('loop_enable').value
        self.loop_count = self.get_parameter('loop_count').value

        # Subscribers
        self.skip_flag_sub = self.create_subscription(Bool, 'skip_flag', self.skip_flag_callback, 10)
        self.event_flag_sub = self.create_subscription(Int32, 'event_flag', self.event_flag_callback, 10)

        # Publishers
        self.next_waypoint_id_pub = self.create_publisher(Int32, 'next_waypoint_id', 10)
        self.reached_waypoint_id_pub = self.create_publisher(Int32, 'reached_waypoint_id', 10)

        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Load waypoints from CSV file
        self.waypoints_data = self.load_waypoints_from_csv(waypoints_csv)
        if not self.waypoints_data:
            self.get_logger().error("No waypoints loaded. Please check the CSV file.")
            return

        # Variables to manage waypoints and navigation
        self.current_waypoint_index = 0
        self.current_loop_count = 0
        self.skip_flag = False
        self.waiting_for_event = False
        self._last_feedback_time = self.get_clock().now()
        self.retry_count = 0
        self.goal_handle = None

    def load_waypoints_from_csv(self, filename):
        waypoints_data = []
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader)

            for row in reader:
                pose_stamped_msg = PoseStamped()
                pose_stamped_msg.header.frame_id = 'map'
                pose_stamped_msg.pose.position.x = float(row[1])
                pose_stamped_msg.pose.position.y = float(row[2])
                pose_stamped_msg.pose.position.z = float(row[3])
                pose_stamped_msg.pose.orientation.x = float(row[4])
                pose_stamped_msg.pose.orientation.y = float(row[5])
                pose_stamped_msg.pose.orientation.z = float(row[6])
                pose_stamped_msg.pose.orientation.w = float(row[7])

                waypoint_data = {
                    "pose": pose_stamped_msg,
                    "skip_flag": int(row[8]),
                    "event_flag": int(row[9])
                }
                
                waypoints_data.append(waypoint_data)

        return waypoints_data

    def skip_flag_callback(self, msg):
        # Handle skip flag messages
        if msg.data and self.current_waypoint_index < len(self.waypoints_data):
            if self.waypoints_data[self.current_waypoint_index]["skip_flag"] == 1:
                self.skip_flag = True
                if self.goal_handle is not None:
                    # Cancel the current goal due to skip request
                    self.get_logger().info(f'Cancelling goal for waypoint {self.current_waypoint_index} due to skip.')
                    cancel_future = self.goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self.cancel_done_callback)

    def event_flag_callback(self, msg):
        # Handle event flag messages
        if self.waiting_for_event and msg.data == 1:
            self.get_logger().info('Event flag received. Proceeding to next waypoint.')
            self.waiting_for_event = False
            self.current_waypoint_index += 1
            self.advance_to_next_waypoint()

    def send_goal(self, waypoint_data):
        # Send a navigation goal to the action server
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint_data["pose"]

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available, waiting...')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the action server
        current_time = self.get_clock().now()
        if (current_time - self._last_feedback_time).nanoseconds >= 3e9:
            # Optionally log feedback (commented out)
            # self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.distance_remaining))
            self._last_feedback_time = current_time

    def goal_response_callback(self, future):
        # Handle the response from the action server when a goal is sent
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            self.goal_handle = None
            return
        
        self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        
        # Publish the next waypoint ID
        if self.current_waypoint_index < len(self.waypoints_data):
            next_waypoint_id = self.current_waypoint_index
            self.next_waypoint_id_pub.publish(Int32(data=next_waypoint_id))

    def goal_result_callback(self, future):
        # Handle the result of the navigation goal
        status = future.result().status
        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached successfully.')
            self.skip_flag = False
            self.retry_count = 0
            
            # Publish the reached waypoint ID
            reached_waypoint_id = self.current_waypoint_index
            self.reached_waypoint_id_pub.publish(Int32(data=reached_waypoint_id))

            current_waypoint_data = self.waypoints_data[self.current_waypoint_index]

            if current_waypoint_data["event_flag"] == 1:
                # Wait for an event before proceeding to next waypoint
                self.waiting_for_event = True
                self.get_logger().info(f'Waiting for event to proceed from waypoint {self.current_waypoint_index}.')
            else:
                # Proceed to the next waypoint
                self.current_waypoint_index += 1
                self.advance_to_next_waypoint()

        elif status == GoalStatus.STATUS_ABORTED:
            if self.retry_count < 1:
                # Retry once if navigation was aborted
                self.retry_count += 1
                self.get_logger().warn(f'Navigation to waypoint {self.current_waypoint_index} aborted. Retrying ({self.retry_count}/1)...')
                if self.current_waypoint_index < len(self.waypoints_data):
                    self.send_goal(self.waypoints_data[self.current_waypoint_index])
                else:
                    self.get_logger().warn('No more waypoints to navigate to.')
            else:
                # Move to next waypoint after retrying
                self.get_logger().warn(f'Navigation to waypoint {self.current_waypoint_index} aborted after retry. Moving to next waypoint.')
                self.retry_count = 0
                self.skip_flag = False
                self.current_waypoint_index += 1
                self.advance_to_next_waypoint()
                
        elif status == GoalStatus.STATUS_CANCELED:
            # Handle goal cancellation
            self.get_logger().info(f'Navigation to waypoint {self.current_waypoint_index} canceled.')
            self.skip_flag = False
            self.current_waypoint_index += 1
            self.advance_to_next_waypoint()
            
        else:
            # Handle other statuses
            self.get_logger().warn(f'Goal failed with status code: {status}. Not advancing to next waypoint.')

    def cancel_done_callback(self, future):
        # Handle the result of a goal cancellation
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f'Goal cancel request accepted for waypoint {self.current_waypoint_index}.')
        else:
            self.get_logger().warn(f'Goal cancel request rejected for waypoint {self.current_waypoint_index}.')

    def advance_to_next_waypoint(self):
        # Proceed to the next waypoint
        if self.current_waypoint_index < len(self.waypoints_data):
            next_waypoint_data = self.waypoints_data[self.current_waypoint_index]
            self.retry_count = 0
            next_waypoint_data["pose"].header.stamp = self.get_clock().now().to_msg()
            self.send_goal(next_waypoint_data)
        else:
            # Handle looping or completion
            if self.loop_enable and self.current_loop_count < self.loop_count - 1:
                self.current_waypoint_index = 0
                self.current_loop_count += 1
                self.get_logger().info(f'Starting loop {self.current_loop_count + 1} / {self.loop_count}.')
                self.advance_to_next_waypoint()
            else:
                self.get_logger().info('Arrived at the last waypoint. Navigation complete.')
        self.skip_flag = False

    def run(self):
        # Start the navigation process
        if self.waypoints_data and self.current_waypoint_index < len(self.waypoints_data):
            self.send_goal(self.waypoints_data[self.current_waypoint_index])
        else:
            self.get_logger().info('No waypoints to navigate to.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Received KeyboardInterrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
