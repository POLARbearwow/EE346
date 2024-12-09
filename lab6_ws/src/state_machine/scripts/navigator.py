import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

class Navigator:
    def __init__(self, point_name, goal_x, goal_y, distance_threshold):
        self.point_name = point_name
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.robot_x = None  # 初始化实例属性
        self.robot_y = None
        self.distance_threshold = distance_threshold
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_robot_position)  #从amcl中获取位置

    def update_robot_position(self, msg):  #msg是 PoseWithCovarianceStamped
        #为什么没有提示 msg Pose pose position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        rospy.loginfo(f"{self.robot_x},{self.robot_y}")  #f
        return self.robot_x , self.robot_y
    
    def get_robot_position(self):
        return self.robot_x, self.robot_y

    def send_goal(self, goal_x, goal_y):
        """发送导航目标"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.w = 1.0  # 默认方向（不对朝向进行约束）
        rospy.loginfo(f"Sending goal to: x={goal_x}, y={goal_y}")
        self.client.send_goal(goal)

    def move_to_goal(self):
        """等待导航完成"""
        rospy.loginfo("Navigating...")
        self.client.wait_for_result()
        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation succeeded!")
            return True
        elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.LOST]:
            rospy.logerr(f"Navigation failed with state: {state}")
            return False
        else:
            rospy.logwarn(f"Unexpected navigation state: {state}")
            return False

    def cancel_goal(self):
        """取消导航任务"""
        if self.client.gh:
            rospy.loginfo("Cancelling current navigation task...")
            self.client.cancel_goal()
        else:
            rospy.logwarn("No active navigation task to cancel.")

    def is_goal_reached(self, robot_x, robot_y, goal_x, goal_y, distance_threshold):
        """检查目标是否到达"""
        distance = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)
        rospy.loginfo(f"Distance to goal: {distance}")
        return distance <= distance_threshold
