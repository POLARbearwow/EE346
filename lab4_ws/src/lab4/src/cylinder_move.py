#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class RobotControl:
    def __init__(self):
        rospy.init_node('robot_control')

        # 发布速度指令
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 订阅激光雷达数据
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # 旋转的速度和阈值
        self.turn_speed = 0.5  # 转向速度
        self.distance_threshold = 0.3  # 30cm的突变阈值
        self.stop_distance = 0.15  # 15cm时停止
        self.rotation_angle = 0.1  # 每次旋转0.1度

        # 状态变量
        self.last_distance = None
        self.rotation_in_progress = False  # 标记是否正在旋转

    def laser_callback(self, msg):
        # 获取激光数据
        ranges = msg.ranges  # 激光扫描点的距离数组
        angle_min = msg.angle_min  # 激光扫描的起始角度
        angle_increment = msg.angle_increment  # 每个扫描点的角度增量

        # 获取激光雷达正前方的距离，假设是第360度的激光数据
        front_distance = ranges[len(ranges) // 2]  # 获取正前方的距离（假设激光雷达正前方是0度）

        # 检查是否有距离突变
        if self.last_distance is not None:
            distance_change = abs(front_distance - self.last_distance)

            if distance_change > self.distance_threshold:
                rospy.loginfo(f"Detected a distance change greater than {self.distance_threshold}m. Stopping rotation and moving forward.")
                self.stop_rotation_and_move()

        # 更新上次的前方距离
        self.last_distance = front_distance

        # 如果前方距离小于停止距离，停止
        if front_distance <= self.stop_distance:
            rospy.loginfo(f"Reached target distance {front_distance}m. Stopping.")
            self.stop_robot()
        else:
            # 如果不是在旋转状态中，开始旋转
            if not self.rotation_in_progress:
                self.rotate_robot()

    def rotate_robot(self):
        """进行原地旋转"""
        rospy.loginfo("Starting rotation...")
        self.rotation_in_progress = True  # 设置旋转状态为进行中

        move_cmd = Twist()
        move_cmd.angular.z = self.turn_speed  # 设置旋转速度
        self.cmd_pub.publish(move_cmd)  # 发布旋转指令

        # 旋转0.1度
        rospy.sleep(0.1)  # 假设旋转0.1度需要一定的时间，可以根据实际调整时间

        self.cmd_pub.publish(Twist())  # 停止旋转

        rospy.loginfo("Completed a 0.1 degree rotation.")
        # 等待下一次旋转
        self.rotation_in_progress = False

    def stop_rotation_and_move(self):
        """停止旋转并直行"""
        rospy.loginfo("Stopping rotation and starting to move forward.")
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # 设置前进速度
        self.cmd_pub.publish(move_cmd)  # 发布前进指令

        # 一直前进直到距离目标位置15cm以内
        while not rospy.is_shutdown():
            # 获取激光雷达数据，检查正前方的距离
            front_distance = rospy.wait_for_message('/scan', LaserScan).ranges[len(rospy.wait_for_message('/scan', LaserScan).ranges) // 2]
            if front_distance <= self.stop_distance:
                rospy.loginfo(f"Reached target distance {front_distance}m. Stopping.")
                self.stop_robot()
                break
            rospy.sleep(0.1)  # 控制前进的速度

    def stop_robot(self):
        """停止机器人运动"""
        move_cmd = Twist()
        self.cmd_pub.publish(move_cmd)
        rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    try:
        robot_control = RobotControl()
        rospy.loginfo("Robot Control Node Started")

        # 保持节点运行，等待激光数据并处理
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception: Shutting down")
