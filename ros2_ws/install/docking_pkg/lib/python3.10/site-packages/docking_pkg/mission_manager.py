import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.state = "MISSION"
        self.battery_level = 100.0
        self.low_battery_threshold = 30.0

        self.waypoints = [
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (-2.0, 2.0),
            (-2.0, 0.0),
            (0.0, -2.0),
            (2.0, -2.0)
        ]

        self.current_index = 0

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.battery_sub = self.create_subscription(
            Float32,
            '/battery_status',
            self.battery_callback,
            10
        )

        self.charge_pub = self.create_publisher(
            Bool,
            '/charging_mode',
            10
        )

        self.action_client.wait_for_server()
        self.send_next_waypoint()

    def battery_callback(self, msg):
        self.battery_level = msg.data

        if self.state == "MISSION" and self.battery_level <= self.low_battery_threshold:
            self.get_logger().warn("Battery Low → Returning to Dock")
            self.state = "RETURN_TO_DOCK"
            self.send_dock_goal()

        elif self.state == "CHARGING" and self.battery_level >= 100.0:
            self.get_logger().info("Battery Full → Resuming Mission")
            self.publish_charging(False)
            self.state = "MISSION"
            self.send_next_waypoint()

    def send_next_waypoint(self):

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("Mission Complete")
            return

        x, y = self.waypoints[self.current_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(x, y)

        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_dock_goal(self):

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(0.0, 0.0)

        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        if self.state == "MISSION":
            self.current_index += 1
            self.send_next_waypoint()

        elif self.state == "RETURN_TO_DOCK":
            self.get_logger().info("Dock Reached → Charging")
            self.state = "CHARGING"
            self.publish_charging(True)

    def publish_charging(self, mode):
        msg = Bool()
        msg.data = mode
        self.charge_pub.publish(msg)

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
