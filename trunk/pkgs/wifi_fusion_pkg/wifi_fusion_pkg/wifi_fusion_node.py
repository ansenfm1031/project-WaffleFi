import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from wifi_interface.msg import WifiRssi, WifiFused


class WifiFusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.sub_wifi = self.create_subscription(
            WifiRssi,
            '/wifi/raw',
            self.on_wifi,
            10
        )

        self.sub_pose = self.create_subscription(
            Point,
            '/grid_pose',
            self.on_pose,
            10
        )

        self.pub_fused = self.create_publisher(
            WifiFused,
            '/wifi/fused',
            10
        )

        self.last_wifi = None
        self.last_pos = None

        self.get_logger().info('fusion_node started')

    def on_wifi(self, msg: WifiRssi):
        self.last_wifi = msg

    def on_pose(self, msg: Point):
        if self.last_wifi is None:
            return

        x = msg.x
        y = msg.y

        if self.last_pos is not None:
            if self.last_pos[0] == x and self.last_pos[1] == y:
                return

        self.last_pos = (x, y)

        fused = WifiFused()
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.header.frame_id = 'map'
        fused.x = x
        fused.y = y
        fused.ssid = self.last_wifi.ssid
        fused.rssi = self.last_wifi.rssi

        self.pub_fused.publish(fused)

        self.get_logger().info(
            f'FUSED (x={x:.2f}, y={y:.2f}) SSID={fused.ssid} RSSI={fused.rssi}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WifiFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
