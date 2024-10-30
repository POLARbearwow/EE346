#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarTester:
    def __init__(self):
        # 初始化节点
        rospy.init_node('lidar_tester', anonymous=True)
        
        # 订阅 /scan topic
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback)
        
        # 存储距离数据，用于分析最小和最大测距范围、精度和分辨率
        self.distance_data = []
        self.precision_data = []
        self.desired_angle = 0.02  # 想要测量的角度，假设为0度（正前方）

    def callback(self, data):
        # 计算指定角度的索引
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        index = int((self.desired_angle - angle_min) / angle_increment)
        
        # 确保索引在范围内
        if index < 0 or index >= len(data.ranges):
            rospy.logwarn("Index out of range for desired angle")
            return
        
        # 获取指定方向的距离值
        distance = data.ranges[index]
        
        # 记录当前测量数据
        rospy.logwarn("Distance at %d degrees: %f", self.desired_angle, distance)
        self.distance_data.append(distance)
        
        # 根据实验条件，执行不同的测试
        self.test_range(distance)
        self.test_precision(distance)
       # self.test_resolution(distance)

    def test_range(self, distance):
        # 测试最小和最大测距范围
        if distance > 0:  # 忽略无效测量
            min_range = min(self.distance_data)
            max_range = max(self.distance_data)
            rospy.logerr("Current Min Range: %f, Max Range: %f", min_range, max_range)

    def test_precision(self, distance):
        # 测试距离精度：在已知距离位置多次测量距离
        target_distance =0.3  # 假设目标实际距离为1米
        
        if len(self.precision_data) < 10:  # 收集10次测量
            self.precision_data.append(distance)
        else:
            # 计算均值和标准差
            mean_distance = np.mean(self.precision_data)
            std_dev = np.std(self.precision_data)
            rospy.loginfo("Precision Test - Mean: %f, Standard Deviation: %f", mean_distance, std_dev)
            rospy.loginfo("Deviation from target: %f", abs(mean_distance - target_distance))
            self.precision_data = []  # 清空数据以便下次测试

    def test_resolution(self, distance):
        # 测试分辨率
        resolution_threshold = 0.05  # 分辨率阈值（5cm）
        
        if len(self.distance_data) > 1:
            # 比较最近两次距离数据
            if abs(self.distance_data[-1] - self.distance_data[-2]) >= resolution_threshold:
                rospy.loginfo("Resolution Test - Distinguishable difference detected")
            else:
                rospy.loginfo("Resolution Test - No distinguishable difference")

    def run(self):
        # 保持节点运行
        rospy.spin()

if __name__ == '__main__':
    tester = LidarTester()
    tester.run()
