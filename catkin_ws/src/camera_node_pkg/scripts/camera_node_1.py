#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

from cv_bridge import CvBridge

import math
import cv2
import os
import numpy as np
import numpy.random as random
import tf.transformations
import tf2_ros

'''
Resources:
- Writing a TF2 listener: http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
'''
def quaternion_to_euler(quaternion):
    """Converts a quaternion to roll, pitch, and yaw angles."""
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return roll, pitch, yaw
    
class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.bounding_boxes = None
        self.past_dist_to_tag = 0
        self.image_width = None
        self.image_height = None
        self.horizontal_fov = 1.047  # Approx 60 degrees in radians
        self.latest_bounding_boxes = []
                
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.image_pub = rospy.Publisher('imagetimer', Image, queue_size=10)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.setpoint_data_pub = rospy.Publisher('/setpoint/data', Float64, queue_size=10)
        self.state_data_pub = rospy.Publisher('/state/data', Float64, queue_size=10)
        # Create a buffer to hold the transforms for the listener
        self.transform_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.transform_buffer)

        rospy.Subscriber("/control_effort/data", Float64, self.callback_to_get_control_effort_x)
        
        # Subscribers
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_to_get_image_raw)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_to_get_bounding_boxes)
        
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback_to_get_tag_detections)
        self.last_detection_list = []

    def callback_to_get_control_effort_x(self, msg):
        print("ok")
        
    def callback_to_get_tag_detections(self, msg):
        self.detection_list = msg

        for detection in self.detection_list.detections:
            id = detection.id[0]
            r,p,y = quaternion_to_euler(detection.pose.pose.pose.orientation)
            #print("r: %f, p: %f, y: %f" % (r, p, y))
            if id == 0:
               self.last_detection_list.append(detection)
               #print("x:", detection.pose.pose.pose.position.x)
               #print("y:", detection.pose.pose.pose.position.y)
               #print("z:", detection.pose.pose.pose.position.z)
        
    def callback_to_get_image_raw(self, msg):
        #rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_height, self.image_width = self.image.shape[:2]

    def callback_to_get_bounding_boxes(self, msg):
        #rospy.loginfo('Bounding boxes received...')
        self.latest_bounding_boxes = []
        for box in msg.bounding_boxes:
            # Store all bounding boxes
            self.latest_bounding_boxes.append(box)
            
    # Consider "start()" as your main()
    def start(self):
        rospy.loginfo("Starting node")

        while not rospy.is_shutdown():
            #rospy.loginfo('publishing image')

            t_camera_link_to_taggy_one = None
            try:
                # Updated to use rospy.Time(0)
                t_camera_link_to_taggy_one = self.transform_buffer.lookup_transform('camera_link', 'name_0', rospy.Time(0))
                #print("Transform received\n", t_camera_link_to_taggy_one)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            t_base_link_to_taggy_one = None
            try:
                # Updated to use rospy.Time(0)
                t_base_link_to_taggy_one = self.transform_buffer.lookup_transform('base_link', 'name_0', rospy.Time(0))
                #print("Transform received\n", t_base_link_to_taggy_one)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
                
            cmd = Twist()
                
            # If we have the transform between the camera and the camera_base_link (may have to add condition on the tag being seen)
            if (t_base_link_to_taggy_one is not None) and (len(self.last_detection_list) > 0):
                
                     
                #print("DIFF in x between base link and camera link", t_base_link_to_taggy_one.transform.translation.x - t_camera_link_to_taggy_one.transform.translation.x) 
                # We have the target in sight!!
                x_dist_to_tag = t_base_link_to_taggy_one.transform.translation.x
                y_dist_to_tag = t_base_link_to_taggy_one.transform.translation.y
                    
                # Get the angle to the target
                angle_to_tag = math.atan2(y_dist_to_tag, x_dist_to_tag)

                 
                dist_to_tag = x_dist_to_tag - 2 #math.sqrt(x_dist_to_tag ** 2 + y_dist_to_tag ** 2) 
                print("dist_to_tag: ", dist_to_tag)
                # Get the distance to the target
                delta_dist_to_tag = (dist_to_tag - self.past_dist_to_tag) / 0.1

                self.past_dist_to_tag = dist_to_tag
                # Reduce the distance to the target by rotating towards it                    
                k_p_az = 2                  
                cmd.angular.z = k_p_az * angle_to_tag

                # Reduce the distance to the target by moving to it
                k_p_lx = 0.1 #0.5
                k_d_lx = 0.05
                cmd.linear.x = k_p_lx * dist_to_tag + k_d_lx * delta_dist_to_tag
                self.last_detection_list.clear()
            else:
                # Wander around
                cmd.linear.x = 0.2  # Forward speed
                cmd.angular.z = 0.5  # Angular speed for turning
                pass
                #print(self.msg)
            
            # Process YOLO bounding boxes and draw them on the image
            if self.image is not None:
                display_image = self.image.copy()
                if self.latest_bounding_boxes:
                    for box in self.latest_bounding_boxes:
                        # Draw bounding box
                        cv2.rectangle(display_image, (box.xmin, box.ymin), (box.xmax, box.ymax), (0,255,0), 2)
                        # Put class label
                        label = f"{box.Class}: {box.probability:.2f}"
                        cv2.putText(display_image, label, (box.xmin, box.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                        
                        # Calculate centroid and angle
                        x_centroid = (box.xmin + box.xmax) / 2.0
                        y_centroid = (box.ymin + box.ymax) / 2.0
                        angle_to_object = (x_centroid - self.image_width / 2.0) * self.horizontal_fov / self.image_width
                        print(f"Angle to object '{box.Class}' from bounding box: {angle_to_object}")
                        # Compare with angle_to_tag if available
                        if (t_base_link_to_taggy_one is not None) and (len(self.last_detection_list) > 0):
                            angle_diff = angle_to_object - angle_to_tag
                            print("Angle difference between YOLO and April Tag: ", angle_diff)
                        else:
                            print("No April Tag detected to compare with.")
                else:
                    print("No bounding boxes to display.")
                
                # Display the image with bounding boxes
                cv2.imshow("Camera Feed with Bounding Boxes", display_image)
                cv2.waitKey(1)  # Needed to display the image
                
                # Publish the image with bounding boxes if needed
                # self.image_pub.publish(self.br.cv2_to_imgmsg(display_image, "bgr8"))
            else:
                print("No image available to display.")
            
            self.twist_pub.publish(cmd)
            self.loop_rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()
    my_node.start()
