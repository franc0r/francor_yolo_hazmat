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

        # Ros2 get package path
        self.__package_path = '/home/martin/Projekte/francor/ros2_ws/src/francor_yolo_hazmat/'
        self.__yolo_path = self.__package_path + 'yolov5'
        self.__model_path = self.__package_path + 'yolov5/2023-04-25-02-yolo5s-128e.pt'

        self.get_logger().info("Package path: " + self.__package_path)

        # declare pubsub routes
        self.__img_subs = self.create_subscription(Image, '/image_raw', self.listener_callback, qos_profile=qos_profile_sensor_data)
        self.__img_pub = self.create_publisher(CompressedImage, '/yolov5/image/compressed', qos_profile=qos_profile_sensor_data)

        # Create bool service callback
        self.__bool_srv = self.create_service(SetBool, '/hazmat_yolo/enable', self.bool_srv_callback)

        # Create 10 hz timer
        self.__timer = self.create_timer(0.1, self.timer_callback)
        
        # declare tools
        self.__cv_br = CvBridge()
        self.__model = torch.hub.load(self.__yolo_path, 'custom', path=self.__model_path, source='local') 

        # Image container
        self.__cv_image_header = None
        self.__cv_image = None
        self.__cv_image_lock = Lock()
        self.__detection_enabled = False

        # Last detections
        self.__last_detection_lst = None

        self.get_logger().info("Node Initialized")
	
    def bool_srv_callback(self, request, response):
        self.__detection_enabled = request.data
        response.success = True
        response.message = "Service call successful"
        return response

    def timer_callback(self):
        # Process image with model
        detection_lst = []

        with self.__cv_image_lock:
            if self.__cv_image is None:
                return
                
            
            # Only if detection is enabled
            if self.__detection_enabled:              
                # Run model
                results = self.__model(self.__cv_image)

                # Loop through all detection as tupples
                for detection in results.pandas().xyxy[0].itertuples():
                    confidence = detection.confidence

                    if confidence < 0.5:
                        continue
                    
                    class_name = detection.name
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
                    if size > image_size * 0.5:
                        continue

                    #if ratio > 1.7 or ratio < 0.3:
                    #    self.get_logger().error("Invalid ratio: %.3f" % ratio)
                    #    continue
                    # Append to detection list
                    detection_lst.append((class_name, p1, p2))

                    # Draw rectangle
                    cv2.rectangle(self.__cv_image, p1, p2, (0, 0, 255), 2)

                    # Draw text
                    cv2.putText(self.__cv_image, class_name, (p1[0], p1[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)

            # Loop through detection list
            for detection in detection_lst:
                # Draw fonts on left corner of image
                cv2.putText(self.__cv_image, detection[0], (10, 20 + 10 * detection_lst.index(detection)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

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
