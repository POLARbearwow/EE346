#import actionlib.simple_action_client
import rospy
import actionlib
import math
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

# 导航到目标点的状态
# 导航状态

#point_name, goal_x, goal_y, distance_threshold
#cylinder处使用的cancel goal以后回调执行停止两秒？


#初始位姿怎么给定 initialpose是怎么发布的？
#这个方法使用了 actionlib 来与 move_base 进行交互 不用/move_base_simple/goal & /initialpose

class NavigateToPoint(smach.State):
    def __init__(self, point_name, goal_x, goal_y, distance_threshold):
        smach.State.__init__(self, outcomes=['success', 'failure'])  #smach的class必须要有？ 为了验证是否对应？
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
    
    #怎么和actionlib结合  sefl.client?
    #只设置了位置 没有设置朝向
    def send_goal(self, x, y, z):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now( )
        goal.goal.target_pose.pose.position.x = self.goal_x
        goal.goal.target_pose.pose.position.y = self.goal_y  #不对z进行约束
        goal.target_pose.pose.orientation.w = 1.0  #默认方向 不对朝向进行约束

    def move_to_goal(self):
        rospy.loginfo("navigating......")
        self.client.wait_for_result()  #阻塞执行
        # result = self.client.get_result() # return Null 可以跳转查看
        state = self.client.get_state()#move_base_msgs/MoveBaseActionResult 也可以扩展出duration

        #阻塞完后才会执行
        if state == actionlib.GoalStatus.SUCCEEDED:  # move_base也是基于actionlib 所以GoalStatus在更基础的actionlib里面
            rospy.loginfo("Navigation succeeded!")
            return True
        elif state == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Navigation is actively being executed.")
        elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.LOST]:
            rospy.logerr(f"Navigation failed with state: {state}")
            #return 'failure'


    def is_goal_reached(self):
        # if self.robot_x is None or self.robot_y is None:
        #     return  False
        distance = math.sqrt((self.goal_x-self.robot_x)**2+(self.goal_y-self.robot_y))  #变量带有self 则是类的实例属性 类的方法都可以访问
        rospy.loginfo(f"距离目标点：{distance}")
        return distance<=self.distance_threshold
    
    def cancel_goal(self):
        """取消导航任务"""
        if self.client.gh:
            rospy.loginfo("取消当前导航任务...")
            self.client.cancel_goal()
        else:
            rospy.logwarn("没有正在进行的导航任务，无法取消。")

     #smach会自动调用execute()     outcome = sm.execute()
     #return的结果（字符串）必须和transitions 的 key 匹配
    def execute(self, userdata):
        """状态机主逻辑"""
        rospy.loginfo(f"开始导航到: {self.point_name}")

        # 等待 action server 就绪
        rospy.loginfo("等待 move_base 动作服务器...")
        self.client.wait_for_server()

        # 发送导航目标  哪里执行发送的？
        if not self.send_goal():
            rospy.logerr("发送目标失败。")
            return 'failure'

        # 等待导航完成或失败   要加一个取消导航点吗？
        result = self.move_to_goal()
        if result:
            if self.is_goal_reached():
                rospy.loginfo("成功到达目标点！")
                self.cancel_goal()
                return 'success'
            else:
                rospy.logerr("未能到达目标点的距离阈值范围内。")
                return 'failure'
        else:
            rospy.logerr("导航任务失败。")
            return 'failure'
        


# region NavigateToPoint AI demo
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

# endregion

# region /initialpose & /move_base_simple/goal
# class GoalPublisher:
#     def __init__(self):
#         rospy.init_node('goal_publisher', anonymous=True)
#         self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
#         self.rate = rospy.Rate(10)  # 10hz

#     def publish_goal(self, x, y, z, qx, qy, qz, qw):
#         goal = PoseStamped()
#         goal.header.frame_id = "map"
#         goal.pose.position.x = x
#         goal.pose.position.y = y
#         goal.pose.position.z = z
#         goal.pose.orientation.x = qx
#         goal.pose.orientation.y = qy
#         goal.pose.orientation.z = qz
#         goal.pose.orientation.w = qw

#         goal.header.stamp = rospy.Time.now()
#         self.pub.publish(goal)
#         self.rate.sleep()

# if __name__ == '__main__':
#     try:
#         goal_publisher = GoalPublisher()
#         # 在需要时调用 publish_goal 方法
#         goal_publisher.publish_goal(2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
#     except rospy.ROSInterruptException:
#         pass

# class InitialPosePublisher:
# def __init__(self):
#     rospy.init_node('initial_pose_publisher', anonymous=True)
#     self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
#     self.rate = rospy.Rate(10)  # 10hz

# def publish_initial_pose(self, x, y, z, qx, qy, qz, qw):
#     initial_pose = PoseWithCovarianceStamped()
#     initial_pose.header.frame_id = "map"
#     initial_pose.pose.pose.position.x = x
#     initial_pose.pose.pose.position.y = y
#     initial_pose.pose.pose.position.z = z
#     initial_pose.pose.pose.orientation.x = qx
#     initial_pose.pose.pose.orientation.y = qy
#     initial_pose.pose.pose.orientation.z = qz
#     initial_pose.pose.pose.orientation.w = qw
#     initial_pose.pose.covariance = [0.0] * 36

#     initial_pose.header.stamp = rospy.Time.now()
#     self.pub.publish(initial_pose)
#     self.rate.sleep()

# if __name__ == '__main__':
#     try:
#         initial_pose_publisher = InitialPosePublisher()
#         # 在需要时调用 publish_initial_pose 方法
#         initial_pose_publisher.publish_initial_pose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
#     except rospy.ROSInterruptException:
#         pass

# endregion
