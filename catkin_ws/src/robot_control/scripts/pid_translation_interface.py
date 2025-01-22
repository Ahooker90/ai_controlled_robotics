#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Transform, Vector3
import tf.transformations as tft
import math
from pid_controller import PIDController


class PIDYawInterface:
    def __init__(self):
        rospy.init_node('pid_yaw_interface_node')

        # Default PID parameters
        self.kp = rospy.get_param('~kp', 1.0)
        self.ki = rospy.get_param('~ki', 0.1)
        self.kd = rospy.get_param('~kd', 0.05)

        # Sampling time (in seconds)
        self.min_sample_time = rospy.get_param('~min_sample_time', 0.01)

        # Initialize PID controller for yaw
        self.pid_yaw = PIDController(self.kp, self.ki, self.kd)

        # Variables to store positions and orientations
        self.robot_position = None
        self.robot_yaw = None
        self.target_position = None

        # ROS publishers and subscribers
        self.control_pub = rospy.Publisher('/yaw_output', Float64, queue_size=10)
        rospy.Subscriber('/robot_pose', Transform, self.robot_pose_callback)
        rospy.Subscriber('/transform_topic', Transform, self.target_transform_callback)
        rospy.Subscriber('/pid_params', Vector3, self.update_pid_params)

        # Last time control was computed
        self.last_control_time = None

    def update_pid_params(self, msg):
        self.kp, self.ki, self.kd = msg.x, msg.y, msg.z
        self.pid_yaw.kp = self.kp
        self.pid_yaw.ki = self.ki
        self.pid_yaw.kd = self.kd
        rospy.loginfo(f"Updated PID Parameters: kp={self.kp}, ki={self.ki}, kd={self.kd}")

    def robot_pose_callback(self, msg):
        # Extract robot's position in Unity's left-hand system (x, z)
        self.robot_position = (msg.translation.x, msg.translation.z)

        # Convert quaternion to Euler angles for yaw (in Unity's coordinate system)
        quaternion_unity = (msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)
        euler = tft.euler_from_quaternion(quaternion_unity, axes='sxyz')
        self.robot_yaw = euler[1]  # Unity's "yaw" corresponds to rotation about the Y-axis

        rospy.loginfo(f"Robot Position: {self.robot_position}, Robot Yaw: {self.robot_yaw}")

    def target_transform_callback(self, msg):
        # Extract target's position in Unity's left-hand system (x, z)
        self.target_position = (msg.translation.x, msg.translation.z)

        if self.robot_position and self.robot_yaw:
            current_time = rospy.get_time()
            if self.last_control_time is None or (current_time - self.last_control_time) >= self.min_sample_time:
                self.compute_and_publish_control(current_time)

    def compute_and_publish_control(self, current_time):
        # Calculate the direction to the target relative to Unity's positive Z-axis
        dx = self.target_position[0] - self.robot_position[0]
        dz = self.target_position[1] - self.robot_position[1]

        # Desired yaw in Unity's positive Z-axis
        desired_yaw = math.atan2(dx, dz)

        # Compute and normalize yaw error to [-pi, pi]
        yaw_error = math.atan2(math.sin(desired_yaw - self.robot_yaw), math.cos(desired_yaw - self.robot_yaw))
        rospy.loginfo(f"Desired Yaw: {desired_yaw}, Yaw Error: {yaw_error}")

        # Update PID controller
        control_yaw = self.pid_yaw.update(yaw_error, current_time)

        # Publish control output
        self.control_pub.publish(Float64(control_yaw))
        rospy.loginfo(f"Published Yaw Control Output: {control_yaw}")

        # Update last control time
        self.last_control_time = current_time


if __name__ == '__main__':
    try:
        pid_interface = PIDYawInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
