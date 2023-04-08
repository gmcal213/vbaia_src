# import dependencies
import rclpy
from rclpy.node import Node
import depthai as dai
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path

# create oak pipeline function with rgb camera
def createPipeline():
    # create Pipeline
    pipeline = dai.Pipeline()

    # create color camera
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(416, 416)
    cam_rgb.setInterleaved(False)


    # transport color camera data from camera side to host side via XLink
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")

    # link color cam and nn output to xlink input
    cam_rgb.preview.link(xout_rgb.input)
    return pipeline

# create Image publisher
class ImagePublisher(Node):

    def __init__(self, rgbQ):

        super().__init__('rgb_publisher')

        self.publisher_ = self.create_publisher(Image, 'video_frames', 1)

        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = rgbQ

        self.br = CvBridge()


        

    def timer_callback(self):
        
        frame = None

        in_rgb = self.cap.tryGet()
       
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()

        if frame is not None:
            self.get_logger().info('Publishing: video frame')
            cv2.imshow("camera", frame)
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            cv2.waitKey(1)

        



# main function loop
def main(args=None):
    # intialize rclpy
    rclpy.init(args=args)

    # create oak pipeline
    pipeline = createPipeline()

    # instantiate the device
    with dai.Device(pipeline) as device:
        
        # from now on, the Device will be in "running mode" and send data via XLink

        # to consume device results, we define output queue
        q_rgb = device.getOutputQueue("rgb")

        # instantiate color camera publisher
        rgbPublisher = ImagePublisher(q_rgb)

        # start callback on color camera node
        rclpy.spin(rgbPublisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()