#!/usr/bin/env python

# MIT License

# Copyright (c) 2017 Victoria Robotics

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#
# @author Griswald Brooks

# @file victoria_base.py Module for receiving and republishing messages from 
#                        victoria_platform.

import rospy
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from victoria_nav_msgs.msg import Odom2DRaw
from victoria_sensor_msgs.msg import IMURaw, BNO055CalibStatus
import tf
import math
import numpy as np

# Publishers
imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
first_odom = Odom2DRaw()
have_first = False
gyro_readings = list()
MAX_GYRO_READINGS = 20
gyro_bias = float('nan')
imu_calibrated = False

def normalize(v):
    norm = np.linalg.norm(v)
    if norm==0:
        return v
    return v/norm

def callbackImuCalib(msg):
    global imu_calibrated
    if msg.sys > 0:
        imu_calibrated = True

def callbackImu(msg):
    # Grab accelerometer and gyro data.
    imu_msg = Imu()
    imu_msg.header = msg.header
    imu_msg.header.frame_id = 'imu_link'

    q = np.array([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
    q = normalize(q)
    euler = tf.transformations.euler_from_quaternion(q)
    q = tf.transformations.quaternion_from_euler(-(euler[0] + math.pi / 2), euler[1], euler[2] - math.pi)
    imu_msg.orientation.x = q[0]
    imu_msg.orientation.y = q[1]
    imu_msg.orientation.z = q[2]
    imu_msg.orientation.w = q[3]
    euler_new = tf.transformations.euler_from_quaternion(q)
    rospy.loginfo("euler[0] = %s, euler[1] = %s, euler = [2] = %s",
            str(euler_new[0]), str(euler_new[1]), str(euler_new[2]))

    imu_msg.orientation_covariance = [0.9,    0,    0, \
                                         0, 0.9,    0, \
                                         0,    0, 0.9]

    imu_msg.angular_velocity = msg.gyro
    imu_msg.angular_velocity_covariance = [0.9,   0,   0, \
                                             0, 0.9,   0, \
                                             0,   0, 0.9]

    # Estimate gyro bias.
    global gyro_readings
    global MAX_GYRO_READINGS
    global gyro_bias
    if len(gyro_readings) < MAX_GYRO_READINGS:
        gyro_readings.append(imu_msg.angular_velocity.z)
    elif math.isnan(gyro_bias):
        gyro_bias = sum(gyro_readings)/MAX_GYRO_READINGS

    if not math.isnan(gyro_bias):
        imu_msg.angular_velocity.z -= gyro_bias

    imu_msg.linear_acceleration = msg.accelerometer
    imu_msg.linear_acceleration_covariance = [0.90,    0,    0, \
                                                 0, 0.90,    0, \
                                                 0,    0, 0.90]

    # Grab magnetometer data.
    mag_msg = MagneticField()
    mag_msg.header = msg.header
    mag_msg.magnetic_field = msg.magnetometer
    # TODO(gbrooks): Add covariance.

    # Publish sensor data.
    imu_pub.publish(imu_msg)
    mag_pub.publish(mag_msg)

def callbackOdom(msg):
        global have_first
        global first_odom
        if not have_first:
            first_odom = msg
            have_first = True

        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = msg.child_frame_id
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = msg.pose.x - first_odom.pose.x
        odom_msg.pose.pose.position.y = msg.pose.y - first_odom.pose.y
        # Wheel radius.
        # TODO(gbrooks): Parameterize.
        odom_msg.pose.pose.position.z = 0.127

        q = tf.transformations.quaternion_from_euler(0, 0, msg.pose.theta - first_odom.pose.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.pose.covariance = [8, 0, 0,   0,   0,   0, \
                                    0, 8, 0,   0,   0,   0, \
                                    0, 0, 8,   0,   0,   0, \
                                    0, 0, 0, 1,   0,   0, \
                                    0, 0, 0,   0, 1,   0, \
                                    0, 0, 0,   0,   0, 1]

        odom_msg.twist.twist.linear.x = msg.twist.vx
        odom_msg.twist.twist.linear.y = msg.twist.vy
        odom_msg.twist.twist.linear.z = 0

        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = msg.twist.vtheta
        odom_msg.twist.covariance = [0.5,   0,   0,    0,    0,    0, \
                                       0, 0.5,   0,    0,    0,    0, \
                                       0,   0, 0.5,    0,    0,    0, \
                                       0,   0,   0, 1,    0,    0, \
                                       0,   0,   0,    0, 1,    0, \
                                       0,   0,   0,    0,    0, 1]


        odom_pub.publish(odom_msg)

        odom_br = tf.TransformBroadcaster()
        odom_br.sendTransform((odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0),
                              tf.transformations.quaternion_from_euler(0, 0, msg.pose.theta - first_odom.pose.theta),
                              rospy.Time.now(),
                              "base_link_odom_wheel",
                              "odom")
def main():
    rospy.init_node('victoria_base', anonymous=True)

    # Subscribe to the Teensy topics.
    rospy.Subscriber('/imu_raw', IMURaw, callbackImu)
    rospy.Subscriber('/odom_2d_raw', Odom2DRaw, callbackOdom)
    rospy.Subscriber('/imu_calibration', BNO055CalibStatus, callbackImuCalib)

    rospy.spin()

if __name__ == '__main__':
    print('victoria_base')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
