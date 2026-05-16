import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # 订阅图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10
        )

        self.bridge = CvBridge()

        # 加载模型（改成你的）
        self.model = YOLO("yolo26x-seg.pt")  
        # self.model = YOLO("best.pt")
        """
        x: carrot 0.6
        s: hot dog 0.7
        m: carrot 0.3
        l: carrot 0.6
        """
        self.get_logger().info("YOLO node started")
        self.last_log_time = self.get_clock().now()
    def listener_callback(self, msg):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO 推理
        results = self.model(frame)

        # 可视化
        annotated_frame = results[0].plot()

        # 显示
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1)

        # ⭐ 限制打印频率（比如 1 秒一次）
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1e9:
            result = results[0]
            num = len(result.masks.data) if result.masks else 0
            self.get_logger().info(f"Detected {num} masks")
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    """
    yolo测试代码
    """