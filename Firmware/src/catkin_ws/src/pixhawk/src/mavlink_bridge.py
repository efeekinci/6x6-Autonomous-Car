#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import pymavlink.mavutil as mavutil
import tf

# Initialize ROS node
rospy.init_node('imu_publisher', anonymous=True)
imu_pub = rospy.Publisher('/pixhawk/imu', Imu, queue_size=10)

# Initialize TF broadcaster
tf_broadcaster = tf.TransformBroadcaster()

# Establish connection to Pixhawk
def connect_pixhawk():
    connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    connection.wait_heartbeat()
    rospy.loginfo("Pixhawk connected successfully!")
    return connection

def publish_imu_data(mavlink_connection):
    while not rospy.is_shutdown():
        # Receive the IMU data from Pixhawk using pymavlink
        msg = mavlink_connection.recv_match(type='SCALED_IMU2', blocking=True)
        if msg:
            # Create an IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "base_link"
            
            # Convert pymavlink data to ROS IMU message format
            imu_msg.linear_acceleration.x = msg.xacc / 1000.0
            imu_msg.linear_acceleration.y = msg.yacc / 1000.0
            imu_msg.linear_acceleration.z = msg.zacc / 1000.0
            imu_msg.angular_velocity.x = msg.xgyro / 1000.0
            imu_msg.angular_velocity.y = msg.ygyro / 1000.0
            imu_msg.angular_velocity.z = msg.zgyro / 1000.0

            # Add orientation data (setting to default until real data is provided)
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0

            # Set covariance matrices (these values should be tuned based on sensor specs)
            imu_msg.orientation_covariance = [0.01, 0, 0,
                                              0, 0.01, 0,
                                              0, 0, 0.01]
            imu_msg.angular_velocity_covariance = [0.01, 0, 0,
                                                   0, 0.01, 0,
                                                   0, 0, 0.01]
            imu_msg.linear_acceleration_covariance = [0.1, 0, 0,
                                                      0, 0.1, 0,
                                                      0, 0, 0.1]

            # Publish the IMU message
            imu_pub.publish(imu_msg)

            # Broadcast the IMU transform
            tf_broadcaster.sendTransform(
                (0.0, 0.0, 0.0),
                (imu_msg.orientation.x,
                 imu_msg.orientation.y,
                 imu_msg.orientation.z,
                 imu_msg.orientation.w),
                rospy.Time.now(),
                "base_link",
                "imu_link"
            )

if __name__ == '__main__':
    pixhawk_connection = connect_pixhawk()
    try:
        publish_imu_data(pixhawk_connection)
    except rospy.ROSInterruptException:
        pass
