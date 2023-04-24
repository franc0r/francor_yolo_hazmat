#https://pyimagesearch.com/2021/08/02/pytorch-object-detection-with-pre-trained-networks/
#https://pytorch.org/hub/ultralytics_yolov5/
#https://github.com/ultralytics/yolov5/discussions/5872

import torch
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from threading import Lock
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('yolov5_node')

        # declare pubsub routes
        self.__img_subs = self.create_subscription(Image, '/image_raw', self.listener_callback, 1)
        self.__img_pub = self.create_publisher(Image, '/yolov5/image', 10)

        # Create 10 hz timer
        self.__timer = self.create_timer(0.2, self.timer_callback)
        
        # declare tools
        self.__cv_br = CvBridge()
        self.__model = torch.hub.load('/home/martin/Projekte/francor/HazmatWS/yolov5', 'custom', path='/home/martin/Projekte/francor/HazmatWS/yolov5/hazmat.pt', source='local') 

        # Image container
        self.__cv_image = None
        self.__cv_image_lock = Lock()

        # Last detections
        self.__last_detection_lst = None

        self.get_logger().info("Node Initialized")
	
    def timer_callback(self):
        # Process image with model
        with self.__cv_image_lock:
            if self.__cv_image is None:
                return

            # Run model
            results = self.__model(self.__cv_image)

            # Detection list
            detection_lst = []
            
            # Loop through all detection as tupples
            for detection in results.pandas().xyxy[0].itertuples():
                confidence = detection.confidence

                if confidence < 0.7:
                    continue
                
                class_name = detection.name
                p1 = (int(detection.xmin), int(detection.ymin))
                p2 = (int(detection.xmax), int(detection.ymax))

                # Calculate size
                size = (p2[0] - p1[0]) * (p2[1] - p1[1])

                # Calculate image size
                image_size = self.__cv_image.shape[0] * self.__cv_image.shape[1]

                # Only if size is smaller than 40% of image size continue
                if size > image_size * 0.3:
                    continue

                # Draw rectangle
                cv2.rectangle(self.__cv_image, p1, p2, (0, 255, 0), 2)

                # Draw text
                cv2.putText(self.__cv_image, class_name, p1, cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)


            # Publish image
            self.__img_pub.publish(self.__cv_br.cv2_to_imgmsg(self.__cv_image, encoding="rgb8"))


    def listener_callback(self, data):
        # Convert to greyscale
        cv_image = self.__cv_br.imgmsg_to_cv2(data)

        # Conver to rgb8
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)

        # Copy image to self.__cv_image
        with self.__cv_image_lock:
            self.__cv_image = cv_image.copy()


        #processed_image = self.model(current_frame)
        #img = processed_image.ims[0]

        # convert to rgb8 ros msg
        #detections = self.br.cv2_to_imgmsg(img, encoding="rgb8")

        # pub it back
        #self.image_publisher.publish(result)

        #results = self.model(cv_image)
        #self.get_logger().info(str(results.pandas().xyxy[0])) 


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
