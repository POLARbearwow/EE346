#!/usr/bin/env python3

#import actionlib.simple_action_client
import rospy
import smach
import smach_ros
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from lab6_ws.src.state_machine.scripts.ApproachCylinder import ApproachCylinder  # 导入靠近圆柱的状态

# 导航到目标点的状态
# 导航状态

#point_name, goal_x, goal_y, distance_threshold
#cylinder处使用的cancel goal以后回调执行停止两秒？
#初始位姿怎么给定 initialpose是怎么发布的？
class NavigateToPoint(smach.State):
    def __init__(self, point_name, goal_x, goal_y, distance_threshold):
        self.point_name = point_name
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.distance_threshold = distance_threshold
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_robot_position)

    def update_robot_position(self, msg):
        #为什么没有提示
        self.robot_x = msg.pose.
        self.robot_y = msg.
        rospy.loginfo(f"{self.robot_x},{self.robot_y}")  #f

        

    
























# class NavigateToPoint(smach.State):
#     def __init__(self, point_name, goal_x, goal_y, distance_threshold=0.5):
#         smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
#         self.point_name = point_name
#         self.goal_x = goal_x
#         self.goal_y = goal_y
#         self.distance_threshold = distance_threshold
#         self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

#         # 当前机器人位置
#         self.robot_x = 0.0
#         self.robot_y = 0.0
#         rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_robot_position)

#     def update_robot_position(self, msg):
#         """
#         更新机器人当前位置（从 AMCL 获取）。
#         """
#         self.robot_x = msg.pose.pose.position.x
#         self.robot_y = msg.pose.pose.position.y

#     def execute(self, userdata):
#         rospy.loginfo(f"Navigating to {self.point_name} at ({self.goal_x}, {self.goal_y})...")

#         # 等待 move_base 服务可用
#         rospy.loginfo("Waiting for move_base action server...")
#         if not self.client.wait_for_server(rospy.Duration(5.0)):
#             rospy.loginfo("move_base action server not available. Aborting...")
#             return 'aborted'

#         # 设置导航目标
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.header.stamp = rospy.Time.now()
#         goal.target_pose.pose.position.x = self.goal_x
#         goal.target_pose.pose.position.y = self.goal_y
#         goal.target_pose.pose.orientation.w = 1.0

#         # 发送目标
#         self.client.send_goal(goal)
#         rospy.loginfo(f"Sent goal to move_base: ({self.goal_x}, {self.goal_y})")

#         rate = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             distance = math.sqrt((self.robot_x - self.goal_x)**2 + (self.robot_y - self.goal_y)**2)
#             rospy.loginfo(f"Distance to Point {self.point_name}: {distance:.2f}")
#             if distance < self.distance_threshold:
#                 rospy.loginfo(f"Reached {self.point_name} within {self.distance_threshold} meters.")
#                 self.client.cancel_goal()
#                 return 'succeeded'
#             rate.sleep()

#         return 'aborted'


# 主程序
def main():
    rospy.init_node('smach_navigation_with_move_base')

    # 创建状态机
    sm = smach.StateMachine(outcomes=['all_tasks_completed', 'aborted'])

    with sm:
        # 导航到 A 点
        smach.StateMachine.add('NAVIGATE_TO_POINT_A',
                               NavigateToPoint('Point A', 1.0, 2.0),
                               transitions={'succeeded': 'APPROACH_CYLINDER_AT_A',
                                            'aborted': 'aborted'})

        # 靠近 A 点的圆柱
        smach.StateMachine.add('APPROACH_CYLINDER_AT_A',
                               ApproachCylinder(),
                               transitions={'succeeded': 'NAVIGATE_TO_POINT_B',
                                            'aborted': 'aborted'})

        # 导航到 B 点
        smach.StateMachine.add('NAVIGATE_TO_POINT_B',
                               NavigateToPoint('Point B', 2.0, 3.0),
                               transitions={'succeeded': 'APPROACH_CYLINDER_AT_B',
                                            'aborted': 'aborted'})

        # 靠近 B 点的圆柱
        smach.StateMachine.add('APPROACH_CYLINDER_AT_B',
                               ApproachCylinder(),
                               transitions={'succeeded': 'NAVIGATE_TO_POINT_C',
                                            'aborted': 'aborted'})

        # 导航到 C 点
        smach.StateMachine.add('NAVIGATE_TO_POINT_C',
                               NavigateToPoint('Point C', 3.0, 4.0),
                               transitions={'succeeded': 'APPROACH_CYLINDER_AT_C',
                                            'aborted': 'aborted'})

        # 靠近 C 点的圆柱
        smach.StateMachine.add('APPROACH_CYLINDER_AT_C',
                               ApproachCylinder(),
                               transitions={'succeeded': 'all_tasks_completed',
                                            'aborted': 'aborted'})

    # 执行状态机
    outcome = sm.execute()
    rospy.loginfo(f"State Machine finished with outcome: {outcome}")


if __name__ == "__main__":
    main()
