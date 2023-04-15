# import dependencies
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import depthai as dai
import cv2
from cv_bridge import CvBridge
from pathlib import Path
import json
import numpy as np

# setup oak pipeline function with rgb camera and the implement detection network
def createPipeline(nnConfig, nnPath):
    
    # parse input shape
    if "input_size" in nnConfig:
        W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

    # extract metadata
    metadata = nnConfig.get("NN_specific_metadata", {})
    classes = metadata.get("classes", {})
    coordinates = metadata.get("coordinates", {})
    anchors = metadata.get("anchors", {})
    anchorMasks = metadata.get("anchor_masks", {})
    iouThreshold = metadata.get("iou_threshold", {})
    confidenceThreshold = metadata.get("confidence_threshold", {})

    # sync outputs
    syncNN = True
    
    # create Pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    nnOut = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")
    nnOut.setStreamName("nn")

    # Properties
    camRgb.setPreviewSize(W, H)

    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(40)

    # Network specific settings
    detectionNetwork.setConfidenceThreshold(confidenceThreshold)
    detectionNetwork.setNumClasses(classes)
    detectionNetwork.setCoordinateSize(coordinates)
    detectionNetwork.setAnchors(anchors)
    detectionNetwork.setAnchorMasks(anchorMasks)
    detectionNetwork.setIouThreshold(iouThreshold)
    detectionNetwork.setBlobPath(nnPath)
    detectionNetwork.setNumInferenceThreads(2)
    detectionNetwork.input.setBlocking(False)

    # Linking
    camRgb.preview.link(detectionNetwork.input)
    detectionNetwork.passthrough.link(xoutRgb.input)
    detectionNetwork.out.link(nnOut.input)
    return pipeline


# create detection result publisher
class DetectionPublisher(Node):

    # constuctor function
    def __init__(self):

        super().__init__('detection_publisher')

        # create publisher for image
        self.publisher1_ = self.create_publisher(Image, 'detection_image', 10)
        self.publisher2_ = self.create_publisher(String, 'detection_label', 10)

        # parse config
        configPath = Path(str(Path.cwd()) + '/src/oak_d_poe/resource/best1.json')

        with configPath.open() as f:
            config = json.load(f)
        nnConfig = config.get("nn_config", {})

        # get model path
        nnPath = str(Path.cwd()) + '/src/oak_d_poe/resource/best1_openvino_2022.1_6shave.blob'

        # parse labels
        nnMappings = config.get("mappings", {})
        self.labels = nnMappings.get("labels", {})

        # create device object 
        self.device = dai.Device(createPipeline(nnConfig, nnPath))

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        # CV bridge to ros
        self.br = CvBridge()

        # define timer, this will act as a while loop checking if the queue's have received a new frame
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Oak-D publishing images to detection_image')
        self.get_logger().info('Oak-D publishing detections to detection_label')

          

    # processing image functions
    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(self, frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    # display the frame to the screen
    def displayFrame(self, name, frame, detections):
        color = (255, 0, 0)
        for detection in detections:
            bbox = self.frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            cv2.putText(frame, self.labels[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        return frame

    # timer callback function
    def timer_callback(self):
        frame = None
        detections = []
        
        inRgb = self.qRgb.get()
        inDet = self.qDet.get()

        msg = String()

        if inRgb is not None:
            frame = inRgb.getCvFrame()

        if inDet is not None:
            detections = inDet.detections

        if frame is not None:
            frame = self.displayFrame("rgb", frame, detections)
            self.publisher1_.publish(self.br.cv2_to_imgmsg(frame))
            for detection in detections:
                if detection.confidence > .8:
                    msg.data = str(self.labels[detection.label]) + " with " + str(detections[0].confidence * 100) + " percent confidence"
                    self.publisher2_.publish(msg)
                else:
                    msg.data = "No implement detected"
                    self.publisher2_.publish(msg)
                break
            cv2.waitKey(1)

        


def main(args=None):
    rclpy.init(args=args)

    detection_publisher = DetectionPublisher()

    rclpy.spin(detection_publisher)

    detection_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()