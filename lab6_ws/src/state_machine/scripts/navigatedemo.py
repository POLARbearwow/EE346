#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
import math
import time
import tf
from sensor_msgs.msg import LaserScan
#import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN

class ScanAndMove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_reached', 'no_goal'])
        self.data_queue = []  # 只保留最新的数据
        self.angles = []
        self.x = []
        self.y = []
        self.ok = False
        self.stop_distance = 0.20 # 修改停止距离为 15 厘米
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_reached = False

        #rospy.init_node('scan_listener', anonymous=True)  # 初始化 ROS 节点
        rospy.Subscriber("scan", LaserScan, self.callback)  # 订阅 scan 话题

    def callback(self, data):
        #print("enter laserscan callback")
        self.data_queue = data.ranges
        # 将 data.ranges 转换为 numpy 数组
        ranges_array = np.array(data.ranges)
        self.angles = np.linspace(data.angle_min, data.angle_max, len(ranges_array))

        # 创建掩码，筛选出非零的点
        non_zero_mask = ranges_array > 0

        # 仅保留非零点的 x 和 y
        self.x = ranges_array[non_zero_mask] * np.cos(self.angles[non_zero_mask])
        self.y = ranges_array[non_zero_mask] * np.sin(self.angles[non_zero_mask])

        self.ok = True  # 更新ok标志位

    def move_to_goal(self, goal_x, goal_y):
        """
        控制 TurtleBot3 移动到目标点并在指定距离停下。
        """
        twist = Twist()
        rate = rospy.Rate(10)  # 10 Hz

        distance_to_goal = np.sqrt(goal_x**2 + goal_y**2)

        rospy.loginfo(f"Current distance to goal: {distance_to_goal:.2f} meters")

        if distance_to_goal <= self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            rospy.loginfo(f"TurtleBot stopped within {self.stop_distance} meters of the goal.")
            return True  # 目标达成

        # 计算角度调整和线速度
        angle_to_goal = np.arctan2(goal_y, goal_x)
        rospy.loginfo(f"Moving to goal: distance={distance_to_goal:.2f}, angle={angle_to_goal:.2f}")    
        twist.linear.x = min(0.2, distance_to_goal)  # 最大速度0.2米/秒
        twist.angular.z = angle_to_goal * 3.0  # 调整角速度比例增益
        self.velocity_publisher.publish(twist)
        rate.sleep()
        return False  # 尚未达到目标

    def execute(self, userdata):
        #if self.data_queue and self.ok:  # 检查队列中是否有数据
            # 聚类部分
        max_points = 8  # 默认最大点数
        thred = 0.1  # 默认最大距离
        while not self.goal_reached:    
            # 聚类
            points = np.vstack((self.x, self.y)).T  # 创建用于聚类的点
            dbscan = DBSCAN(eps=0.3, min_samples=2)  # 定义DBSCAN聚类
            labels = dbscan.fit_predict(points)  # 进行聚类

            # 统计每个聚类的点数
            unique_labels = set(labels)  # 获取唯一标签
            cluster_sizes = {label: np.sum(labels == label) for label in unique_labels}

            # 按点数排序找到最少和次少的聚类
            sorted_clusters = sorted([(label, size) for label, size in cluster_sizes.items() if label != -1], 
                                     key=lambda x: x[1])
            selected_cluster = None
            selected_center = None

            for cluster in sorted_clusters:  # 遍历从点数最少到最多的聚类
                label, size = cluster
                if size >= max_points: 
                    continue
                center = np.mean(points[labels == label], axis=0)
                distance_to_origin = np.linalg.norm(center)  # 计算质心到原点的距离
                if distance_to_origin <= 1.4:  # 找到第一个满足条件的聚类
                    selected_cluster = label
                    selected_center = center
                    break

            if selected_cluster is not None:
                # 调用移动函数
                self.goal_reached = self.move_to_goal(selected_center[0], selected_center[1])
                if self.goal_reached:
                    time.sleep(3)
                    return 'goal_reached'
            else:
                print("No cluster within 1.3 meters from the origin was found.")
        
        # print(self.data_queue)
        # print(self.ok)
        # #print(self.data_queue)
        # return 'no_goal'  # 如果没有找到目标或数据不符合条件



class NavigateToPoint(smach.State):
    def __init__(self, x, y, frame="map"):
            smach.State.__init__(self, outcomes=['succeeded', 'failed'])
            self.x = x
            self.y = y
            self.frame = frame
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.client.wait_for_server()
            self.listener = tf.TransformListener()

    def get_current_position(self):
        """
        获取机器人的当前位置
        """
        try:
            # 获取机器人当前位置的变换（从 base_link 到 map）
            (trans, rot) = self.listener.lookupTransform(self.frame, 'base_link', rospy.Time(0))
            return trans  # 返回 (x, y) 坐标
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get current position.")
            return None

    def execute(self, userdata):
        rospy.loginfo(f"Navigating to point ({self.x}, {self.y})")

        # 创建目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = 1.0  # 姿态朝向目标

        # 发送目标并等待结果
        self.client.send_goal(goal)

        # 等待结果直到目标到达
        self.client.wait_for_result()

        # 获取当前机器人的位置
        current_position = self.get_current_position()

        if current_position:
            distance = math.sqrt((current_position[0] - self.x) ** 2 + (current_position[1] - self.y) ** 2)

            if distance < 0.2:
                rospy.loginfo(f"Successfully reached point ({self.x}, {self.y}), distance: {distance:.2f}m")
                return 'succeeded'
            else:
                rospy.logwarn(f"Failed to reach point ({self.x}, {self.y}), distance: {distance:.2f}m")
                return 'failed'
        else:
            rospy.logwarn("Unable to get current position.")
            return 'failed'


def set_initial_pose():
# 设置机器人的初始位姿
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    #rospy.init_node('set_initial_pose_node', anonymous=True)
    rate = rospy.Rate(1)

    # 设置机器人的初始位姿  通过 rviz/initialpose 读出来的
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = -0.565  # 设置 x 坐标
    pose.pose.pose.position.y = 0.45  # 设置 y 坐标
    pose.pose.pose.orientation.z = -0.1387  # 设置姿态
    pose.pose.pose.orientation.w = 0.9903  # 设置姿态

    #covariance要考虑吗
    start_time = time.time()
    
    # 每 0.1 秒发布一次 initialpose 消息，持续 2 秒钟
    while time.time() - start_time < 2.0:  # 在两秒钟内发布消息
        pub.publish(pose)
        rospy.loginfo("Publishing initial pose...")
        time.sleep(0.1)  # 每 0.1 秒发布一次

def main():
    rospy.init_node('state_machine_navigation')

    set_initial_pose()

    sm = smach.StateMachine(outcomes=['success', 'failure'])

    with sm:
        smach.StateMachine.add('NavigateToA', NavigateToPoint(1.536, 0.831), transitions={'succeeded': 'ScanAndMoveA', 'failed': 'failure'})
        smach.StateMachine.add('ScanAndMoveA', ScanAndMove(), transitions={'goal_reached': 'NavigateToB', 'no_goal': 'failure'})
        smach.StateMachine.add('NavigateToB', NavigateToPoint(1.296, 3.649), transitions={'succeeded': 'ScanAndMoveB', 'failed': 'failure'})
        smach.StateMachine.add('ScanAndMoveB', ScanAndMove(), transitions={'goal_reached': 'NavigateToC', 'no_goal': 'failure'})
        smach.StateMachine.add('NavigateToC', NavigateToPoint(3.606, 3.706), transitions={'succeeded': 'ScanAndMoveC', 'failed': 'failure'})
        smach.StateMachine.add('ScanAndMoveC', ScanAndMove(), transitions={'goal_reached': 'success', 'no_goal': 'failure'})


    outcome = sm.execute()

    if outcome == 'success':
        rospy.loginfo("Navigation to all points succeeded!")
    else:
        rospy.loginfo("Navigation failed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass