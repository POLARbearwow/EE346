#!/usr/bin/env python
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def get_closest_cylinder():
    """
    获取最近圆柱的距离和方向。
    :return: 最近圆柱的距离（单位：米）和角度（单位：弧度）
    """
    # 模拟从激光雷达数据中提取圆柱位置，实际需替换为用户定义逻辑
    closest_distance = float('inf')  # 假设初始距离无限远
    closest_angle = 0.0  # 最近圆柱方向
    # 实现激光雷达扫描处理逻辑（可订阅 /scan 数据并解析）
    rospy.loginfo("Detecting cylinder...")
    return closest_distance, closest_angle


# 轨迹控制逻辑
def move_toward_cylinder(closest_distance, closest_angle, target_distance):
    """
    控制机器人向圆柱靠近。

    :param closest_distance: 最近圆柱的距离（单位：米）。
    :param closest_angle: 最近圆柱的方向（单位：弧度）。
    :param target_distance: 目标距离（单位：米）。
    :return: True 表示已成功接近目标距离，False 表示尚未达到目标距离。
    """
    # 定义速度控制发布器
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    # 如果未达到目标距离
    if closest_distance > target_distance:
        # 控制机器人朝向圆柱方向移动
        twist.linear.x = 0.2  # 前进速度（单位：米/秒，可调）
        twist.angular.z = -0.5 * closest_angle  # 根据方向调整角速度（单位：弧度/秒）
        velocity_publisher.publish(twist)
        rospy.loginfo(f"Moving toward cylinder. Distance: {closest_distance}, Angle: {closest_angle}")
        return False
    else:
        # 停止机器人
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        velocity_publisher.publish(twist)
        rospy.loginfo("Reached target distance from cylinder.")
        return True


class TaskWithCylinderDetection(smach.State):
    def __init__(self, task_name):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.task_name = task_name

    def execute(self, userdata):
        rospy.loginfo(f"Starting task: {self.task_name}")
        rospy.loginfo("Switching to cylinder detection mode...")
        rospy.sleep(1)  # 模拟模式切换时间

        target_distance = 0.1  # 目标距离：10cm
        timeout_duration = 15  # 超时时间：15秒
        start_time = rospy.Time.now()

        # 开始检测并接近圆柱
        while True:
            # 获取最近圆柱的距离和方向
            closest_distance, closest_angle = get_closest_cylinder()

            # 检查是否超时
            if rospy.Time.now() - start_time > rospy.Duration(timeout_duration):
                rospy.logerr("Failed to approach cylinder: timeout reached.")
                return 'failed'

            # 调用轨迹控制逻辑，判断是否到达目标距离
            if move_toward_cylinder(closest_distance, closest_angle, target_distance):
                rospy.loginfo("Successfully approached cylinder.")
                rospy.sleep(2)  # 到达目标后停留 2 秒
                rospy.loginfo(f"Task at {self.task_name} completed.")
                return 'completed'

            rospy.sleep(0.1)  # 每隔 0.1 秒检查一次



# 定义一个通用的导航状态
class NavigateToPoint(smach.State):
    def __init__(self, point_name, x, y, z=0, w=1.0):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.point_name = point_name
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.z = z
        self.goal.target_pose.pose.orientation.w = w
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo(f"Navigating to {self.point_name}")
        self.client.wait_for_server()
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Arrived at {self.point_name}")
            return 'succeeded'
        else:
            rospy.logerr(f"Failed to reach {self.point_name}")
            return 'aborted'

# 定义不同任务状态
class TaskAtPointA(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("Performing task at Point A")
        # 模拟任务逻辑
        rospy.sleep(2)
        rospy.loginfo("Task at Point A completed")
        return 'completed'

class TaskAtPointB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("Performing task at Point B")
        # 模拟任务逻辑
        rospy.sleep(2)
        rospy.loginfo("Task at Point B completed")
        return 'completed'

class TaskAtPointC(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("Performing task at Point C")
        # 模拟任务逻辑
        rospy.sleep(2)
        rospy.loginfo("Task at Point C completed")
        return 'completed'

class TaskAtPointD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("Performing task at Point D")
        # 模拟任务逻辑
        rospy.sleep(2)
        rospy.loginfo("Task at Point D completed")
        return 'completed'

# 主函数
def main():
    rospy.init_node('smach_navigation_example')

    # 创建状态机
    sm = smach.StateMachine(outcomes=['all_tasks_completed', 'aborted'])

    # 添加导航点和任务
    with sm:
        # 导航到点A并执行任务
        smach.StateMachine.add('NAVIGATE_TO_POINT_A',
                               NavigateToPoint('Point A', 1.0, 2.0),
                               transitions={'succeeded': 'TASK_AT_POINT_A',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('TASK_AT_POINT_A',
                               TaskAtPointA(),
                               transitions={'completed': 'NAVIGATE_TO_POINT_B',
                                            'failed': 'aborted'})

        # 导航到点B并执行任务
        smach.StateMachine.add('NAVIGATE_TO_POINT_B',
                               NavigateToPoint('Point B', 2.0, 3.0),
                               transitions={'succeeded': 'TASK_AT_POINT_B',
                                            'aborted': 'aborted'})
        
        smach.StateMachine.add('TASK_AT_POINT_B',
                               TaskAtPointB(),
                               transitions={'completed': 'NAVIGATE_TO_POINT_C',
                                            'failed': 'aborted'})

        # 导航到点C并执行任务
        smach.StateMachine.add('NAVIGATE_TO_POINT_C',
                               NavigateToPoint('Point C', 3.0, 1.0),
                               transitions={'succeeded': 'TASK_AT_POINT_C',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('TASK_AT_POINT_C',
                               TaskAtPointC(),
                               transitions={'completed': 'NAVIGATE_TO_POINT_D',
                                            'failed': 'aborted'})

        # 导航到点D并执行任务
        smach.StateMachine.add('NAVIGATE_TO_POINT_D',
                               NavigateToPoint('Point D', 0.0, 0.0),
                               transitions={'succeeded': 'TASK_AT_POINT_D',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('TASK_AT_POINT_D',
                               TaskAtPointD(),
                               transitions={'completed': 'all_tasks_completed',
                                            'failed': 'aborted'})

    # 执行状态机
    outcome = sm.execute()
    rospy.loginfo(f"State Machine finished with outcome: {outcome}")

if __name__ == '__main__':
    main()
