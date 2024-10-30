#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    # 打印LiDAR的第一个距离值作为示例
    rospy.loginfo("Received LiDAR data:")
    rospy.loginfo("Ranges: %s", data.ranges)
    #rospy.loginfo(1111)
    # 您可以根据需要打印任意角度的距离值，例如data.ranges[0]、data.ranges[90]等

def listener():
    # 初始化节点
    rospy.init_node('lidar_listener', anonymous=True)
    # 订阅 /scan topic
    rospy.Subscriber("/scan", LaserScan, callback)
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    listener()
