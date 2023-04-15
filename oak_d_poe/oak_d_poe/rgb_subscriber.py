# import dependencies
import rclpy
from rclpy.node import Node
import depthai as dai
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):

    def __init__(self):

        super().__init__('rgb_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'detection_image',
            self.listener_callback,
            10)
        self.subscription

        self.br = CvBridge()

        self.get_logger().info('Publishing video frames from detection_image')

    def listener_callback(self, data):


        current_frame = self.br.imgmsg_to_cv2(data)

        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)

def main(args=None):

    rclpy.init(args=args)

    rgb_subscriber = ImageSubscriber()

    rclpy.spin(rgb_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()