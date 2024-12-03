#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarTester:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('lidar_tester', anonymous=True)

        # 订阅激光雷达数据
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback)

        # 发布运动命令
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # 初始化一些参数
        self.angle_increment = None  # 激光扫描增量
        self.prev_distance = None  # 上次测量的距离
        self.distance_threshold = 0.30  # 距离跳变阈值（单位：米），超过此值认为发生了突变
        self.turn_speed = 0.3  # 原地旋转的速度
        self.move_speed = 0.2  # 向前移动的速度
        self.stop_distance = 0.15  # 停止前进的距离（单位：米），小于此值则停止
        self.is_moving_forward = False  # 标记小车是否在移动
        self.is_first_scan = True  # 是否第一次扫描

    def callback(self, data):
        # 获取激光雷达数据
        angle_min = data.angle_min  # 激光雷达起始角度
        self.angle_increment = data.angle_increment  # 激光扫描的角度增量
        ranges = data.ranges  # 激光雷达数据（距离值）

        # 计算180°方向对应的索引
        index_180 = int((180.0 - angle_min) / self.angle_increment)

        # 如果是第一次扫描，初始化prev_distance
        if self.is_first_scan:
            self.prev_distance = ranges[index_180]  # 初始位置的180°方向的距离
            self.is_first_scan = False  # 设置为False，后续不再是第一次扫描

        # 获取当前180°方向的距离
        current_distance = ranges[index_180]
        
        # 输出当前角度与上次角度之间的差异
        distance_diff = abs(current_distance - self.prev_distance)
        rospy.loginfo(f"180° Direction - Previous Distance: {self.prev_distance}, Current Distance: {current_distance}, Diff: {distance_diff}")

        # 如果距离差值超过阈值，则认为发生了突变
        if distance_diff > self.distance_threshold:
            rospy.loginfo(f"Significant distance change detected at 180° direction. Moving forward!")
            self.move_forward()
        
        # 更新上次的距离
        self.prev_distance = current_distance

        # 如果小车没有在前进，则继续旋转
        if not self.is_moving_forward:
            self.rotate()
        else:
            self.monitor_distance()

    def rotate(self):
        # 发布原地旋转命令
        move_cmd = Twist()
        move_cmd.angular.z = self.turn_speed  # 设置旋转速度
        self.cmd_pub.publish(move_cmd)

    def move_forward(self):
        # 发布前进命令
        move_cmd = Twist()
        move_cmd.linear.x = self.move_speed  # 设置前进速度
        self.cmd_pub.publish(move_cmd)

        # 设置小车正在前进的标记
        self.is_moving_forward = True

    def monitor_distance(self):
        # 获取激光雷达数据的前方距离（通常是扫描数据的中心部分）
        front_distance = rospy.wait_for_message("/scan", LaserScan).ranges[len(rospy.wait_for_message("/scan", LaserScan).ranges) // 2]
        rospy.loginfo(f"Front distance: {front_distance}")

        # 如果距离小于停止阈值，则停止前进
        if front_distance <= self.stop_distance:
            rospy.loginfo("Distance is less than 15cm. Stopping forward movement.")
            self.stop_moving()
        else:
            # 继续前进
            self.move_forward()

    def stop_moving(self):
        # 发布停止命令
        move_cmd = Twist()
        self.cmd_pub.publish(move_cmd)

        # 设置小车停止移动的标记
        self.is_moving_forward = False

    def run(self):
        # 保持ROS节点运行
        rospy.spin()

if __name__ == '__main__':
    try:
        tester = LidarTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass
