#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def camera_publisher():
    rospy.init_node('camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # Open the default camera
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Camera could not be opened. Please check your camera connection.")
        return

    # Set the publishing rate
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret:
            try:
                # Convert OpenCV image to ROS Image message in "rgb8" encoding
                image_msg = bridge.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), "rgb8")
                # Publish the image
                image_pub.publish(image_msg)
                rospy.loginfo("Image published on /usb_cam/image_raw")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            rospy.logwarn("Failed to capture image from camera.")

        # Sleep to maintain the rate
        rate.sleep()

    # Release the camera when done
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
