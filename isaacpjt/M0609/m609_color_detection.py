"""손목 카메라 이미지에서 큐브 색을 HSV로 감지해 숫자로 퍼블리시하는 노드.

구독:  image_topic  (sensor_msgs/Image)   기본 /rgb
발행:  result_topic (std_msgs/Int32)      기본 /color_id
       파랑 = 1, 초록 = 2  (감지 안 되면 발행하지 않음)
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# HSV 범위 (OpenCV: H 0~179)
# Isaac Sim 하늘색 배경(H 100~103, S 77~127)이 잡히지 않도록 S 하한을 높게 잡음
BLUE_LOWER, BLUE_UPPER = (110, 150, 50), (130, 255, 255)
GREEN_LOWER, GREEN_UPPER = (40, 120, 50), (85, 255, 255)


class ColorDetectorNode(Node):

    def __init__(self):
        super().__init__('m609_color_detector')
        self.declare_parameter('image_topic', '/rgb')
        self.declare_parameter('result_topic', '/color_id')
        self.declare_parameter('min_area', 500)  # 픽셀 수가 이보다 적으면 무시

        image_topic = self.get_parameter('image_topic').value
        result_topic = self.get_parameter('result_topic').value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Int32, result_topic, 10)
        # Isaac Sim 카메라 퍼블리셔가 BEST_EFFORT이므로 sensor_data QoS 사용
        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'구독: {image_topic} -> 발행: {result_topic} (blue=1, green=2)')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 범위 안에 들어오는 픽셀 수 세기 (마스크)
        blue = cv2.countNonZero(cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER))
        green = cv2.countNonZero(cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER))

        if max(blue, green) < self.get_parameter('min_area').value:
            return

        color_id = 1 if blue > green else 2
        self.pub.publish(Int32(data=color_id))
        self.get_logger().info(
            f'감지: {color_id} (blue={blue}, green={green})', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
