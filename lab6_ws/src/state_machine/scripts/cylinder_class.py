# #!/usr/bin/env python3
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN
import smach
import smach_ros

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

        rospy.init_node('scan_listener', anonymous=True)  # 初始化 ROS 节点
        rospy.Subscriber("scan", LaserScan, self.callback)  # 订阅 scan 话题

    def callback(self, data):
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
        if self.data_queue and self.ok:  # 检查队列中是否有数据
            # 聚类部分
            max_points = 11  # 默认最大点数
            thred = 0.2  # 默认最大距离
            
            # 聚类
            points = np.vstack((self.x, self.y)).T  # 创建用于聚类的点
            dbscan = DBSCAN(eps=0.4, min_samples=2)  # 定义DBSCAN聚类
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
                center = np.mean(points[labels == label], axis=0)
                distance_to_origin = np.linalg.norm(center)  # 计算质心到原点的距离
                if distance_to_origin <= 1.3:  # 找到第一个满足条件的聚类
                    selected_cluster = label
                    selected_center = center
                    break

            if selected_cluster is not None:
                # 调用移动函数
                goal_reached = self.move_to_goal(selected_center[0], selected_center[1])
                if goal_reached:
                    return 'goal_reached'
            else:
                print("No cluster within 1.3 meters from the origin was found.")

        return 'no_goal'  # 如果没有找到目标或数据不符合条件


class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo("Finished task.")
        return 'finished'


if __name__ == '__main__':
    # 创建状态机
    sm = smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('SCAN_AND_MOVE', ScanAndMove(), transitions={'goal_reached': 'STOP', 'no_goal': 'SCAN_AND_MOVE'})
        smach.StateMachine.add('STOP', Stop(), transitions={'finished': 'finished'})

    # 执行状态机
    outcome = sm.execute()


# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# import matplotlib.pyplot as plt
# import numpy as np
# from sklearn.cluster import DBSCAN

# class TurtleBotController:
#     def __init__(self):
#         self.data_queue = []  # 只保留最新的数据
#         self.angles = []
#         self.x = []
#         self.y = []
#         self.ok = False
#         self.stop_distance = 0.3  # 距离目标质心15厘米时停止

#         rospy.init_node('scan_listener', anonymous=True)  # 初始化 ROS 节点
#         rospy.Subscriber("scan", LaserScan, self.callback)  # 订阅 scan 话题
#         self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#         plt.figure(figsize=(12, 8))  # 设置图形大小

#     def callback(self, data):
#         self.data_queue = data.ranges
        
#         # 将 data.ranges 转换为 numpy 数组
#         ranges_array = np.array(data.ranges)
#         self.angles = np.linspace(data.angle_min, data.angle_max, len(ranges_array))
        
#         # 创建掩码，筛选出非零的点
#         non_zero_mask = ranges_array > 0
        
#         # 仅保留非零点的 x 和 y
#         self.x = ranges_array[non_zero_mask] * np.cos(self.angles[non_zero_mask])
#         self.y = ranges_array[non_zero_mask] * np.sin(self.angles[non_zero_mask])
        
#         self.ok = True  # 更新ok标志位

#     def move_to_goal(self, goal_x, goal_y):
#         """
#         控制 TurtleBot3 移动到目标点并在指定距离停下。
#         """
#         twist = Twist()
#         rate = rospy.Rate(10)  # 10 Hz

#         distance_to_goal = np.sqrt(goal_x**2 + goal_y**2)

#         if distance_to_goal <= self.stop_distance:
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.velocity_publisher.publish(twist)
#             rospy.loginfo(f"TurtleBot stopped within {self.stop_distance} meters of the goal.")
#             return  # 正常退出函数

#         # 计算角度调整和线速度
#         angle_to_goal = np.arctan2(goal_y, goal_x)
#         rospy.loginfo(f"Moving to goal: distance={distance_to_goal:.2f}, angle={angle_to_goal:.2f}")    
#         twist.linear.x = min(0.2, distance_to_goal)  # 最大速度0.2米/秒
#         twist.angular.z = angle_to_goal * 3.0  # 调整角速度比例增益
#         self.velocity_publisher.publish(twist)
#         rate.sleep()

#     def run(self):
#         while not rospy.is_shutdown():
#             if self.data_queue and self.ok:  # 检查队列中是否有数据
#                 plt.clf()  # 清除之前的图形
                
#                 max_points = 11  # 默认最大点数
#                 thred = 0.2  # 默认最大距离
                
#                 # 聚类
#                 points = np.vstack((self.x, self.y)).T  # 创建用于聚类的点
#                 dbscan = DBSCAN(eps=0.4, min_samples=2)  # 定义DBSCAN聚类
#                 labels = dbscan.fit_predict(points)  # 进行聚类

#                 # 统计每个聚类的点数
#                 unique_labels = set(labels)  # 获取唯一标签
#                 cluster_sizes = {label: np.sum(labels == label) for label in unique_labels}

#                 # 按点数排序找到最少和次少的聚类
#                 sorted_clusters = sorted([(label, size) for label, size in cluster_sizes.items() if label != -1], 
#                                          key=lambda x: x[1])
#                 selected_cluster = None
#                 selected_center = None

#                 for cluster in sorted_clusters:  # 遍历从点数最少到最多的聚类
#                     label, size = cluster
#                     center = np.mean(points[labels == label], axis=0)
#                     distance_to_origin = np.linalg.norm(center)  # 计算质心到原点的距离
#                     if distance_to_origin <= 1.3:  # 找到第一个满足条件的聚类
#                         selected_cluster = label
#                         selected_center = center
#                         break

#                 # 打印每个聚类的信息
#                 for label in unique_labels:
#                     if label == -1:
#                         continue
#                     cluster_size = cluster_sizes[label]
#                     cluster_points = points[labels == label]
#                     if cluster_size < max_points:
#                         cluster_center = np.mean(cluster_points, axis=0)
#                         # 计算最大距离
#                         distances = np.linalg.norm(cluster_points - cluster_center, axis=1)
#                         max_distance = np.max(distances)
#                         if max_distance < thred:  # 符合条件的聚类
#                             plt.scatter(cluster_points[:, 0], cluster_points[:, 1], 
#                                     s=1, color='red', label=f'Filtered Cluster {label}')  # 符合条件的聚类使用红点显示
#                     else:
#                         plt.scatter(cluster_points[:, 0], cluster_points[:, 1], s=1, label=f'Cluster {label}')  # 其他聚类使用默认颜色显示

#                 # 标记选定的聚类质心
#                 if selected_cluster is not None:
#                     plt.scatter(selected_center[0], selected_center[1], 
#                                 color='red', marker='*', s=200, label=f'Selected Cluster {selected_cluster}')
#                     plt.text(selected_center[0], selected_center[1], 
#                              f"({selected_center[0]:.2f}, {selected_center[1]:.2f})", 
#                              fontsize=12, color='red', ha='center')

#                     # 调用移动函数
#                     self.move_to_goal(selected_center[0], selected_center[1])

#                 else:
#                     print("No cluster within 1.3 meters from the origin was found.")

#                 plt.title("Filtered Clusters with Selected Cluster Highlighted")
#                 plt.xlabel("X (meters)")
#                 plt.ylabel("Y (meters)")

#                 plt.axis('equal')  # 确保 X 和 Y 轴比例相同
#                 plt.grid(True)  # 显示网格

#                 plt.tight_layout()  # 自动调整子图参数
#                 plt.pause(1)  # 暂停以更新图形
#                 self.ok = False  # 重置ok标志位

#         plt.show()  # 显示最终图形

# if __name__ == '__main__':
#     controller = TurtleBotController()
#     controller.run()