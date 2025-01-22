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
        rospy.Subscriber('/pid_params', Vector3, self.update_pid_params)  # Changed to Vector3

    def update_pid_params(self, msg):
        # Update PID parameters dynamically
        self.kp = msg.x
        self.ki = msg.y
        self.kd = msg.z
        self.pid_yaw.kp = self.kp
        self.pid_yaw.ki = self.ki
        self.pid_yaw.kd = self.kd
        rospy.loginfo(f"Updated PID Parameters: kp={self.kp}, ki={self.ki}, kd={self.kd}")

    def robot_pose_callback_old(self, msg):
        # Extract robot's position
        self.robot_position = (msg.translation.x, msg.translation.y)

        # Adjust the quaternion to match ROS coordinate frame
        # Unity to ROS conversion: (x, y, z, w) -> (-x, -z, -y, w)
        quaternion_unity = (
            msg.rotation.x,
            msg.rotation.y,
            msg.rotation.z,
            msg.rotation.w
        )

        # Adjust the quaternion
        quaternion_ros = (
            -quaternion_unity[0],
            -quaternion_unity[2],
            -quaternion_unity[1],
            quaternion_unity[3]
        )

        # Convert adjusted quaternion to Euler angles
        # Specify the axes sequence according to ROS conventions
        euler = tft.euler_from_quaternion(quaternion_ros, axes='sxyz')
        self.robot_yaw = euler[2]  # Extract yaw angle

        # Debug statement
        rospy.loginfo(f"Robot Position: {self.robot_position}, Robot Yaw: {self.robot_yaw}")

    def robot_pose_callback(self, msg):
        # Extract robot's position
        self.robot_position = (msg.translation.x, msg.translation.z)

        # Get the quaternion from the messageibe<Float64Msg>("/yaw_output"
        quaternion_unity = (
            msg.rotation.x,
            msg.rotation.y,
            msg.rotation.z,
            msg.rotation.w
        )

        euler = tft.euler_from_quaternion(quaternion_unity, axes='sxyz')
        self.robot_yaw = euler[2]  

        # Debug statement
        rospy.loginfo(f"Robot Position: {self.robot_position}, Robot Yaw: {self.robot_yaw}")


    def target_transform_callback(self, msg):
        # Extract target's position
        self.target_position = (msg.translation.x, msg.translation.y)

        if self.robot_position is not None and self.robot_yaw is not None:
            self.compute_and_publish_control()

    def compute_and_publish_control(self):
        # Compute desired yaw angle to face the target

        dx = self.target_position[0] - self.robot_position[0]
        dy = self.target_position[1] - self.robot_position[1]
        desired_yaw = math.atan2(dy, dx)

        # Compute yaw error
        yaw_error = desired_yaw - self.robot_yaw
        # Normalize yaw_error to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # Debug statements
        rospy.loginfo(f"Desired Yaw: {desired_yaw}, Yaw Error: {yaw_error}")

        # Update PID controller for yaw
        current_time = rospy.get_time()
        control_yaw = self.pid_yaw.update(yaw_error, current_time)

        # Publish yaw control output
        self.control_pub.publish(Float64(control_yaw))
        rospy.loginfo(f"Published Yaw Control Output: {control_yaw}")

if __name__ == '__main__':
    try:
        pid_interface = PIDYawInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
