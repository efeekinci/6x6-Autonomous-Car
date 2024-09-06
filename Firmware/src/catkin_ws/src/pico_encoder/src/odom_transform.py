#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf
from tf.transformations import quaternion_from_euler
import math

# Global variables for state
x = 0.0
y = 0.0
th = 0.0
vx = 0.0
vy = 0.0
vth = 0.0
pan_angle = 10.0

# Callback function for velocity updates
def chatter_callback(msg):
    global vx, vy, vth, vel
    vel = msg.linear.x
    vth = msg.angular.z
    vx = vel
    vy = 0.0

# Callback function for pan angle updates
def pan_callback(msg):
    global pan_angle
    pan_angle = msg.angular.z

if __name__ == '__main__':
    rospy.init_node('deneme')

    # Publishers and Subscribers
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    rospy.Subscriber('pan_angle', Twist, pan_callback)
    rospy.Subscriber('encoder', Twist, chatter_callback)

    # Transform broadcaster for TF
    odom_broadcaster = tf.TransformBroadcaster()

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rate = rospy.Rate(50)  # 50 Hz loop rate

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Calculate time difference
        dt = (current_time - last_time).to_sec()
        delta_x = vx * dt * math.cos(th)
        delta_y = vx * dt * math.sin(th)
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        # Create quaternion from yaw
        odom_quat = quaternion_from_euler(0, 0, th)

        # Publish the transform over tf
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        odom_broadcaster.sendTransform(
            (odom_trans.transform.translation.x, odom_trans.transform.translation.y, odom_trans.transform.translation.z),
            odom_quat,
            current_time,
            odom_trans.child_frame_id,
            odom_trans.header.frame_id
        )

        # Publish joint state
        joint_state = JointState()
        joint_state.header.stamp = current_time
        joint_state.name = ["base_link_to_pan"]
        joint_state.position = [pan_angle]
        joint_pub.publish(joint_state)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        odom_pub.publish(odom)

        last_time = current_time
        rate.sleep()
