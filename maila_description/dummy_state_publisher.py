#! /usr/bin/env python
from math import sin, cos, pi, atan2
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(
            JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)

        # robot state
        steering_angle = 0.
        wheel_angle = 0.

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        joint_state = JointState()

        try:
            t = 0
            while rclpy.ok():
                t += 1./30.
                self.get_logger().info(str(t))
                rclpy.spin_once(self)

                # Set angle
                rotation_names = ['front_left_wheel_joint', 'front_right_wheel_joint',
                                  'rear_right_wheel_joint', 'rear_left_wheel_joint']
                wheel_angle += 2*pi * t/50
                wheel_angle = atan2(sin(wheel_angle), cos(wheel_angle))
                rotation_position = [wheel_angle,
                                     wheel_angle, wheel_angle, wheel_angle]

                # Set steering
                steering_angle = pi/3.*sin(t*2*pi/4.)
                steering_names = ['front_left_steering_joint',
                                  'front_right_steering_joint']
                steering_position = [steering_angle, steering_angle]

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = rotation_names + steering_names
                joint_state.position = rotation_position + steering_position

                # update transform
                # (moving in a circle with radius=2)
                angle = t*2*pi/10.
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle)*2
                odom_trans.transform.translation.y = sin(angle)*2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, angle + pi/2)  # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
        cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
        sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
        sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
        sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    node = StatePublisher()


if __name__ == '__main__':
    main()
