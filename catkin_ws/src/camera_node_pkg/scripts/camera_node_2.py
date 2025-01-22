#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import math
import cv2
import tf.transformations
import tf2_ros


class Nodo(object):
    def __init__(self):
        # Initialize YOLO model
        self.model = YOLO("yolov8m.pt")  # Ensure the YOLO model file is available
        print("YOLOv8 Model Loaded")

        # Params
        self.image = None
        self.image_width = None
        self.image_height = None
        self.last_detection_list = []

        self.br = CvBridge()
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_to_get_image_raw)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback_to_get_tag_detections)

    def callback_to_get_tag_detections(self, msg):
        self.last_detection_list = msg.detections

    def callback_to_get_image_raw(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            self.image = self.br.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = self.image.shape[:2]
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def calculate_yolo_centroid(self, box):
        """Calculates the centroid of a YOLO bounding box."""
        xmin, ymin, xmax, ymax = box
        centroid_x = (xmin + xmax) / 2
        centroid_y = (ymin + ymax) / 2
        return centroid_x, centroid_y

    def calculate_apriltag_centroid(self, tag):
        """Calculates the centroid of an AprilTag detection."""
        if not self.image_width or not self.image_height:
            return None, None

        tag_center = tag.pose.pose.pose.position
        image_x = int((tag_center.x + 1) * (self.image_width / 2))  # Assuming normalized tag center
        image_y = int((1 - tag_center.y) * (self.image_height / 2))  # Flipping y-axis for image coordinates
        return image_x, image_y

    def start(self):
        rospy.loginfo("Starting node with YOLOv8 and AprilTag detection")

        while not rospy.is_shutdown():
            if self.image is not None:
                # Run YOLOv8 detection
                results = self.model(self.image)
                detections = results[0].boxes  # Get the detection boxes

                # Annotate the image
                annotated_image = self.image.copy()
                yolo_centroid = None

                # Process YOLO detections
                if detections is not None and len(detections) > 0:
                    for box in detections:
                        coords = box.xyxy[0].cpu().numpy()  # [xmin, ymin, xmax, ymax]
                        xmin, ymin, xmax, ymax = map(int, coords)
                        yolo_centroid = self.calculate_yolo_centroid((xmin, ymin, xmax, ymax))
                        cv2.rectangle(annotated_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                        cv2.circle(annotated_image, (int(yolo_centroid[0]), int(yolo_centroid[1])), 5, (255, 0, 0), -1)

                # Process AprilTag detections
                apriltag_centroid = None
                if len(self.last_detection_list) > 0:
                    tag = self.last_detection_list[0]  # Assuming first detected AprilTag
                    apriltag_centroid = self.calculate_apriltag_centroid(tag)
                    if apriltag_centroid[0] is not None and apriltag_centroid[1] is not None:
                        cv2.circle(annotated_image, (apriltag_centroid[0], apriltag_centroid[1]), 5, (0, 0, 255), -1)

                # Compare YOLO and AprilTag centroids
                if yolo_centroid and apriltag_centroid:
                    x_diff = abs(yolo_centroid[0] - apriltag_centroid[0])
                    y_diff = abs(yolo_centroid[1] - apriltag_centroid[1])
                    rospy.loginfo(f"Centroid Comparison - X Diff: {x_diff}, Y Diff: {y_diff}")

                # Display the annotated image
                cv2.imshow("Camera View", annotated_image)
                cv2.waitKey(1)

                # Continue traveling in a circle
                cmd = Twist()
                cmd.linear.x = 0.2
                cmd.angular.z = 0.5
                self.twist_pub.publish(cmd)
            else:
                rospy.logwarn("No image received. Waiting...")

            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("yolo_apriltag_controller", anonymous=True)
    my_node = Nodo()
    my_node.start()
