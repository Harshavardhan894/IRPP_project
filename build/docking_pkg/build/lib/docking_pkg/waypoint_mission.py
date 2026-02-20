import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus


class WaypointMission(Node):

    def __init__(self):
        super().__init__('waypoint_mission')

        self.waypoints = [
            (0.5, 0.5),
            (1.5, 0.0),
            (0.0, -1.0)
        ]

        self.current_index = 0

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(2.0, self.start_mission)

    def start_mission(self):
        self.timer.cancel()
        self.send_next_goal()

    def send_next_goal(self):

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("Mission Complete")
            return

        x, y = self.waypoints[self.current_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()

        self.get_logger().info(f"Sending Goal {self.current_index + 1}")

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def result_callback(self, future):

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached")
        else:
            self.get_logger().info("Goal failed")

        self.current_index += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
