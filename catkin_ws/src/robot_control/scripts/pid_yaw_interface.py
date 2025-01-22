#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import tf.transformations as tft
import math


class PIDYawInterface:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pid_yaw_interface_node', anonymous=True)

        # PID Parameters (default values)
        self.kp = rospy.get_param('~kp', 5.0)
        self.ki = rospy.get_param('~ki', 0.0)
        self.kd = rospy.get_param('~kd', 0.00)

        # Robot state variables
        self.robot_yaw = None        # Current yaw angle (in radians)
        self.target_yaw = None       # Target yaw angle (in radians)

        # ROS publishers
        self.state_pub = rospy.Publisher('/state', Float64, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/setpoint', Float64, queue_size=10)
        self.pid_params_pub = rospy.Publisher('/pid_params', Vector3, queue_size=10)

        # ROS subscribers
        rospy.Subscriber('/robot_pose', Vector3, self.robot_pose_callback)  # Unity robot pose
        rospy.Subscriber('/target_pose', Vector3, self.target_pose_callback)  # Unity target pose
        rospy.Subscriber('/control_effort', Float64, self.control_effort_callback)

        # Publishing rate
        self.rate = rospy.Rate(10)  # 10 Hz

        rospy.loginfo("PIDYawInterface node initialized")

    def robot_pose_callback(self, msg):
        """
        Callback to receive the robot's pose and extract the yaw angle.
        """
        try:
            # Check if the input has a full quaternion or only partial components
            if hasattr(msg, 'w'):
                # Full quaternion provided (x, y, z, w)
                quaternion = (msg.x, msg.y, msg.z, msg.w)
            else:
                # Partial quaternion provided (x, y, z)
                # Assume w = 1.0 as a placeholder
                quaternion = (msg.x, msg.y, msg.z, 1.0)

            # Normalize the quaternion to ensure validity
            quaternion = self.normalize_quaternion(quaternion)

            # Convert quaternion to Euler angles and extract yaw
            euler = tft.euler_from_quaternion(quaternion, axes='sxyz')
            self.robot_yaw = euler[2]  # Yaw is the third value

            rospy.loginfo(f"Updated robot yaw: {self.robot_yaw}")
        except Exception as e:
            rospy.logerr(f"Error in robot_pose_callback: {e}")

    def target_pose_callback(self, msg):
        """
        Callback to receive the target object's pose and compute the desired yaw angle.
        """
        if self.robot_yaw is not None:
            try:
                # Compute target yaw based on position difference
                dx = msg.x
                dy = msg.y
                self.target_yaw = math.atan2(dy, dx)
                rospy.loginfo(f"Calculated target yaw: {self.target_yaw}")
            except Exception as e:
                rospy.logerr(f"Error in target_pose_callback: {e}")

    def control_effort_callback(self, msg):
        """
        Callback to receive control effort from the ROS PID controller.
        """
        try:
            control_effort = msg.data
            rospy.loginfo(f"Received control effort: {control_effort}")
        except Exception as e:
            rospy.logerr(f"Error in control_effort_callback: {e}")

    def publish_state(self):
        """
        Publish the robot's current yaw state to the /state topic.
        """
        if self.robot_yaw is not None:
            try:
                self.state_pub.publish(Float64(self.robot_yaw))
                rospy.loginfo(f"Published current yaw: {self.robot_yaw}")
            except Exception as e:
                rospy.logerr(f"Error in publish_state: {e}")

    def publish_setpoint(self):
        """
        Publish the target yaw setpoint to the /setpoint topic.
        """
        if self.target_yaw is not None:
            try:
                self.setpoint_pub.publish(Float64(self.target_yaw))
                rospy.loginfo(f"Published target yaw: {self.target_yaw}")
            except Exception as e:
                rospy.logerr(f"Error in publish_setpoint: {e}")

    def publish_pid_params(self):
        """
        Publish PID parameters to the /pid_params topic.
        """
        try:
            pid_params = Vector3(self.kp, self.ki, self.kd)
            self.pid_params_pub.publish(pid_params)
            rospy.loginfo(f"Published PID parameters: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        except Exception as e:
            rospy.logerr(f"Error in publish_pid_params: {e}")

    def normalize_quaternion(self, q):
        """
        Normalize a quaternion to ensure its validity.
        """
        norm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
        return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)

    def run(self):
        """
        Main loop to publish state, setpoint, and PID parameters.
        """
        while not rospy.is_shutdown():
            try:
                self.publish_state()
                self.publish_setpoint()
                self.publish_pid_params()
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("ROS Interrupt received. Shutting down.")
                break
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")

if __name__ == '__main__':
    try:
        node = PIDYawInterface()
        node.run()
    except rospy.ROSInterruptException:
        pass

