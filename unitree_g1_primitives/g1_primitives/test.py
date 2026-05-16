import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np


class G1Node(Node):
    def __init__(self):
        super().__init__('g1_node')

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/arm_cmd',
            10
        )

        self.timer = self.create_timer(0.02, self.loop)  # 50Hz

    def loop(self):
        msg = Float32MultiArray()
        msg.data = np.zeros(14).tolist()
        self.pub.publish(msg)

        # 👇 加这个，方便你看到在运行
        self.get_logger().info("Publishing arm command")


# 👇 必须有这个！
def main():
    rclpy.init()
    node = G1Node()
    rclpy.spin(node)


if __name__ == '__main__':
    main()