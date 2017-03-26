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
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from victoria_nav_msgs.msg import Odom2DRaw
from victoria_sensor_msgs.msg import IMURaw
import tf

# Publish the new stuff.
imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

def callbackImu(msg):
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.orientation = quaternion thing
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration

        imu_pub.publish(imu_msg)

def callbackOdom(msg):
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.child_frame_id = msg.child_frame_id
        odom_msg.pose.pose.position.x = msg.pose.x
        odom_msg.pose.pose.position.y = msg.pose.y
        odom_msg.pose.pose.position.z = 0

        odom_msg.twist.twist.linear.x = 0
        odom_msg.twist.twist.linear.y = 1
        odom_msg.twist.twist.linear.z = 2

        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 1
        odom_msg.twist.twist.angular.z = 2

        odom_pub.publish(odom_msg)

        odom_br = tf.TransformBroadcaster()
        odom_br.sendTransform((odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "/base_link",
                              "/odom")


def main():
    rospy.init_node('victoria_base', anonymous=True)

    # Subscribe to the Teensy topics.
    rospy.Subscriber('/odom_2d_raw', Odom2DRaw, callbackOdom)
    rospy.Subscriber('/imu_raw', IMURaw, callbackImu)

    rospy.spin()

if __name__ == '__main__':
    print('victoria_base')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
