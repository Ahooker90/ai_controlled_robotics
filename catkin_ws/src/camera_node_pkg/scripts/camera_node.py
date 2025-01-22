#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import numpy as np
import numpy.random as random

class Nodo(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("imagetimer111", anonymous=True)
        
        # Initialize YOLO model
        self.model = YOLO("yolov8m.pt")  # Ensure this model path is correct and downloaded
        #self.model = YOLO("yolov8s.pt")
        print("updated!")
        # Parameters
        self.image = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1)  # Node cycle rate in Hz
        
        # Publishers
        self.image_pub = rospy.Publisher('/ultralytics/detection/image', Image, queue_size=10)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_to_get_image_raw)
        #rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_to_get_image_raw)

    def callback_to_get_image_raw(self, msg):
        try:
            # Convert the incoming ROS Image message to OpenCV format
            self.image = self.br.imgmsg_to_cv2(msg, "bgr8")
            #rospy.loginfo("Image received...")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def start(self):
        rospy.loginfo("Starting YOLO detection and image publishing...")
        while not rospy.is_shutdown():
            if self.image is not None:
                # Run YOLO detection on the image
                results = self.model(self.image)
                
                # Annotate the image with detections
                annotated_image = results[0].plot(show=False)
                
                # Display the image
                cv2.imshow("YOLO Detection", annotated_image)
                cv2.waitKey(1)  # Display update

                # Extract and print detection information
                detections = results[0].boxes  # Get the Boxes object
                if detections is not None and len(detections) > 0:
                    for box in detections:
                        # Extract bounding box coordinates
                        coords = box.xyxy[0].cpu().numpy()  # [xmin, ymin, xmax, ymax]
                        xmin, ymin, xmax, ymax = coords
                        
                        # Extract class index and name
                        class_idx = int(box.cls[0])
                        class_name = self.model.names[class_idx]
                        
                        # Extract confidence score
                        confidence = box.conf[0].cpu().numpy()
                        
                        # Print detection information
                        print(f"Class: {class_name}, Coordinates: ({xmin}, {ymin}, {xmax}, {ymax}), Confidence: {confidence}")

                else:
                    rospy.loginfo("No detections.")

                # Publish the annotated image
                try:
                    self.image_pub.publish(self.br.cv2_to_imgmsg(annotated_image, "bgr8"))
                    rospy.loginfo("Annotated image published")
                except CvBridgeError as e:
                    rospy.logerr("Failed to publish image: {0}".format(e))

                # Create and publish a random Twist command
                cmd = Twist()
                cmd.linear.x = random.rand() - 0.5
                cmd.angular.z = random.rand() - 0.5
                self.twist_pub.publish(cmd)
            else:
                rospy.logwarn("No image received yet. Waiting for /usb_cam/image_raw topic...")

            # Sleep to maintain the loop rate
            self.loop_rate.sleep()

if __name__ == '__main__':
    try:
        my_node = Nodo()
        my_node.start()
    except rospy.ROSInterruptException:
        pass
