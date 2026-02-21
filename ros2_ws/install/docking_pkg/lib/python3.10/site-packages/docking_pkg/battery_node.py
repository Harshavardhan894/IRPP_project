import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        self.battery_level = 40.0
        self.is_moving = False
        self.is_charging = False

        self.publisher = self.create_publisher(
            Float32,
            '/battery_status',
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.charge_sub = self.create_subscription(
            Bool,
            '/charging_mode',
            self.charge_callback,
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.update_battery
        )

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.is_moving = True
        else:
            self.is_moving = False

    def charge_callback(self, msg):
        self.is_charging = msg.data

    def update_battery(self):

        if self.is_charging:
            if self.battery_level < 100.0:
                self.battery_level += 2.0
        else:
            if self.is_moving and self.battery_level > 0.0:
                self.battery_level -= 1.0

        msg = Float32()
        msg.data = self.battery_level
        self.publisher.publish(msg)

        if self.battery_level <= 30.0:
            self.get_logger().warn('LOW BATTERY!')

    def main_loop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

