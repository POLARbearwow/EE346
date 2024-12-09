#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time
import tf


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

    # 设置初始位姿
    set_initial_pose()

    # 创建 SMACH 状态机
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # 添加状态到状态机
    with sm:
        smach.StateMachine.add('NavigateToA', NavigateToPoint(1.536, 0.831), transitions={'succeeded': 'NavigateToB', 'failed': 'failure'})
        smach.StateMachine.add('NavigateToB', NavigateToPoint(1.296, 3.649), transitions={'succeeded': 'NavigateToC', 'failed': 'failure'})
        smach.StateMachine.add('NavigateToC', NavigateToPoint(1.536, 0.831), transitions={'succeeded': 'success', 'failed': 'failure'})

    # 执行状态机
    outcome = sm.execute()

    # 输出状态机的最终结果
    if outcome == 'success':
        rospy.loginfo("Navigation to all points succeeded!")
    else:
        rospy.loginfo("Navigation failed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
