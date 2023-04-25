#https://pyimagesearch.com/2021/08/02/pytorch-object-detection-with-pre-trained-networks/
#https://pytorch.org/hub/ultralytics_yolov5/
#https://github.com/ultralytics/yolov5/discussions/5872

import torch
import rclpy
import os
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from threading import Lock
from ament_index_python.packages import get_package_share_directory
import cv2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('yolov5_node')

        # Get package directory
        self.__package_path = get_package_share_directory('francor_yolo_hazmat')
        self.get_logger().info("Package path: %s" % self.__package_path)

        # get ros2 param image subscriber
        self.declare_parameter('yolo_path', '/opt/yolov5')
        self.declare_parameter('image_subs_topic', '/image_raw')
        self.declare_parameter('image_pub_topic', '/yolov5/detections')
        self.declare_parameter('qos_sensor_data_on', 0)
        self.declare_parameter('yolo_weights_file', '2023-04-25-02-yolo5s-128e.pt')
        self.declare_parameter('yolo_update_rate_hz', 10.0)
        self.declare_parameter('object_mode', 0)
        self.declare_parameter('yolo_confidence', 0.85)
        self.declare_parameter('obj_max_size', 0.5)
        self.declare_parameter('obj_min_ratio', 0.6)
        self.declare_parameter('obj_max_ratio', 1.4)

        # Get parameters
        self.__yolo_path = self.get_parameter('yolo_path').value
        self.__image_subs_topic = self.get_parameter('image_subs_topic').value
        self.__image_pub_topic = self.get_parameter('image_pub_topic').value
        self.__qos_sensor_data = bool(self.get_parameter('qos_sensor_data_on').value)
        self.__yolo_weights_file = self.get_parameter('yolo_weights_file').value
        self.__yolo_confidence = self.get_parameter('yolo_confidence').value
        self.__yolo_update_rate_hz = self.get_parameter('yolo_update_rate_hz').value
        self.__object_mode = bool(self.get_parameter('object_mode').value)
        self.__obj_max_size = self.get_parameter('obj_max_size').value
        self.__obj_min_ratio = self.get_parameter('obj_min_ratio').value
        self.__obj_max_ratio = self.get_parameter('obj_max_ratio').value

        # Set paths
        self.__model_path = self.__package_path + '/data/' + self.__yolo_weights_file

        # Print info
        self.get_logger().info("Yolo-Path: %s" % (self.__yolo_path))
        self.get_logger().info("Model-Path: %s" % (self.__model_path))
        self.get_logger().info("Object Mode: %s" % str(self.__object_mode))

        # Create subscriber
        if self.__qos_sensor_data == True:
            self.__img_subs = self.create_subscription(Image, self.__image_subs_topic, self.listener_callback, qos_profile=qos_profile_sensor_data)
        else:
            self.__img_subs = self.create_subscription(Image, self.__image_subs_topic, self.listener_callback, 1)

        # Create publishers
        self.__img_pub = self.create_publisher(CompressedImage, self.__image_pub_topic + '/compressed', qos_profile=qos_profile_sensor_data)

        # Create 10 hz timer
        self.__timer = self.create_timer((1.0 / self.__yolo_update_rate_hz), self.timer_callback)
        
        # declare tools
        self.__cv_br = CvBridge()
        self.__model = torch.hub.load(self.__yolo_path, 'custom', path=self.__model_path, source='local') 

        # Image container
        self.__cv_image_header = None
        self.__cv_image = None
        self.__cv_image_lock = Lock()


        self.get_logger().info("Node Initialized")
	
    def timer_callback(self):
        # Process image with model
        detection_lst = []

        with self.__cv_image_lock:
            if self.__cv_image is None:
                return
                          
            # Run model
            results = self.__model(self.__cv_image)

            # Loop through all detection as tupples
            for detection in results.pandas().xyxy[0].itertuples():
                confidence = detection.confidence

                if confidence < self.__yolo_confidence:
                    continue
                
                class_name = detection.name
                class_type = detection.name[0:3]

                # Check mode
                if self.__object_mode == False:
                    if class_type != 'SQR' and class_type != 'RHO':
                        continue
                else:
                    if class_type != 'OBJ':
                        continue

                p1 = (int(detection.xmin), int(detection.ymin))
                p2 = (int(detection.xmax), int(detection.ymax))

                # Calculate size
                x_size = p2[0] - p1[0]
                y_size = p2[1] - p1[1]
                size = x_size * y_size

                # Calculate ratio
                ratio = x_size / y_size

                # Calculate image size
                image_size = self.__cv_image.shape[0] * self.__cv_image.shape[1]

                # Only if size is smaller than 50% of image size continue
                if size > image_size * self.__obj_max_size:
                    self.get_logger().warn("[%s]: Invalid size: %.3f" % (class_name, image_size))
                    continue

                if ratio > self.__obj_max_ratio or ratio < self.__obj_min_ratio:
                    self.get_logger().warn("[%s]: Invalid ratio: %.3f" % (class_name, ratio))
                    continue
                # Append to detection list
                detection_lst.append((class_name, p1, p2))

                # Draw rectangle
                cv2.rectangle(self.__cv_image, p1, p2, (0, 0, 255), 2)

                # Draw text
                cv2.putText(self.__cv_image, class_name[4:], (p1[0], p1[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)

            # Sort list alphabetically
            detection_lst.sort(key=lambda tup: tup[0])

            # Loop through detection list
            for detection in detection_lst:
                # Draw fonts on left corner of image
                cv2.putText(self.__cv_image, detection[0][4:], (10, 40 + 30 * detection_lst.index(detection)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

            # Change color format
            self.__cv_image = cv2.cvtColor(self.__cv_image, cv2.COLOR_RGB2BGR)

            msg = CompressedImage()
            msg.header = self.__cv_image_header
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpeg', self.__cv_image)[1]).tostring()

            # Publish image
            self.__img_pub.publish(msg)


    def listener_callback(self, data):
        # Copy image to self.__cv_image
        with self.__cv_image_lock:
            self.__cv_image_header = data.header
            self.__cv_image = self.__cv_br.imgmsg_to_cv2(data, desired_encoding='rgb8').copy()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
