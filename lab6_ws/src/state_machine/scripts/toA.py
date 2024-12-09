#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal
import actionlib
import time

def set_initial_pose():
    # 设置机器人的初始位姿
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('set_initial_pose_node', anonymous=True)
    rate = rospy.Rate(1)

    # 设置机器人的初始位姿
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
    # 发布初始位姿  不用循环initialpose可以接收到 但是rviz里无变化
    # while not rospy.is_shutdown():
    #     pub.publish(pose)
    #     rate.sleep()

def move_to_goal(x, y):
    # 使用 MoveBase 发送目标点
    rospy.loginfo("start navigating!")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server to start...")
    client.wait_for_server()

    rospy.loginfo("Sending goal to move_base...")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标点坐标
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # 假设目标点朝向为 0

    client.send_goal(goal)


    # goal_time=time.time()
    # while time.time()-goal_time<2.0:
    #     rospy.loginfo("Sending goal to move_base...")
    #     client.send_goal(goal)
    #     time.sleep(0.1)

    # 等待直到目标到达
    client.wait_for_result()

    rospy.loginfo("Goal reached!")

def send_goal():
    # 初始化节点
    #rospy.init_node('send_goal_node', anonymous=True)

    # 创建发布器
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    # 构造目标消息
    goal = MoveBaseActionGoal()
    
    # 填充目标数据
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    
    # 设置目标位姿
    goal.goal.target_pose.pose.position.x = 0.095
    goal.goal.target_pose.pose.position.y = 0.85
    goal.goal.target_pose.pose.position.z = 0.0
    
    goal.goal.target_pose.pose.orientation.x = 0.0
    goal.goal.target_pose.pose.orientation.y = 0.0
    goal.goal.target_pose.pose.orientation.z = 0.14080793448251405
    goal.goal.target_pose.pose.orientation.w = 0.9900369314256756

    
    # 发布目标消息
    rospy.loginfo("Sending goal to /move_base/goal")
    pub.publish(goal)

    # 等待一段时间让消息发布完成
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        # 1. 设置机器人的初始位置
        set_initial_pose()
        #rospy.init_node('robot_navigator', anonymous=True)
        # 2. 设定目标位置 (例如，目标点坐标为 (2, 2))
        #move_to_goal(1.475, 0.93)
        #send_goal()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
