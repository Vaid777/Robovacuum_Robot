#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load YOLOv5 model (pre-trained on COCO)
        self.get_logger().info('Loading YOLOv5 model...')
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.45  # Confidence threshold
        self.model.iou = 0.45   # IoU threshold for NMS
        
        # Classes relevant for vacuum robot
        self.relevant_classes = [
            'person', 'cat', 'dog',  # Living beings to avoid
            'chair', 'couch', 'bed',  # Furniture
            'backpack', 'handbag', 'suitcase',  # Objects on floor
            'bottle', 'cup',  # Spillable items
            'book', 'cell phone', 'laptop',  # Valuable items
            'shoe'  # Common floor obstacles
        ]
        
        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )
        
        self.annotated_pub = self.create_publisher(
            Image,
            '/yolo/image_annotated',
            10
        )
        
        # Performance tracking
        self.frame_count = 0
        self.process_every_n = 3  # Process every 3rd frame for performance
        
        self.get_logger().info('YOLO Detector initialized!')

    def image_callback(self, msg):
        # Skip frames for performance
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLOv5 inference
            results = self.model(cv_image)
            
            # Parse results
            detections = results.pandas().xyxy[0]  # Get detections as pandas DataFrame
            
            # Create detection message
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # Filter for relevant objects
            for idx, detection in detections.iterrows():
                if detection['name'] in self.relevant_classes:
                    det_msg = Detection2D()
                    det_msg.header = msg.header
                    
                    # Bounding box
                    bbox = BoundingBox2D()
                    bbox.center = Pose2D()
                    bbox.center.x = (detection['xmin'] + detection['xmax']) / 2.0
                    bbox.center.y = (detection['ymin'] + detection['ymax']) / 2.0
                    bbox.size_x = detection['xmax'] - detection['xmin']
                    bbox.size_y = detection['ymax'] - detection['ymin']
                    det_msg.bbox = bbox
                    
                    # Add class and confidence
                    det_msg.results = []
                    # Note: vision_msgs might need ObjectHypothesisWithPose
                    
                    detection_array.detections.append(det_msg)
                    
                    # Log detection
                    self.get_logger().info(
                        f"Detected {detection['name']} at "
                        f"({bbox.center.x:.0f}, {bbox.center.y:.0f}) "
                        f"with confidence {detection['confidence']:.2f}"
                    )
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Publish annotated image
            annotated = results.render()[0]
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in YOLO detection: {str(e)}')

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detector = YOLODetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()