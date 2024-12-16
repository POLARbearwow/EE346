import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import smach
from navigator import Navigator 

class Approachlinder(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.navigator = Navigator()
        self.data_queue = []
        self.angles = []
        self.x = []
        self.y = []
        self.ok = False
        self.stop_distance = 0.2  # 距离目标质心15厘米时停止

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

    def execute(self, userdata):
        rospy.init_node('scan_listener', anonymous=True)  # 初始化 ROS 节点
        rospy.Subscriber("scan", LaserScan, self.callback)  # 订阅 scan 话题
        results = []  # 存储三次的结果
        attempts = 3  # 设置尝试次数

        #while not rospy.is_shutdown()
        while not rospy.is_shutdown() and len(results) < attempts:
            if self.data_queue and self.ok:  # 检查队列中是否有数据
                max_points = 11  # 默认最大点数
                thred = 0.2  # 默认最大距离
                
                # 聚类
                points = np.vstack((self.x, self.y)).T  # 创建用于聚类的点
                dbscan = DBSCAN(eps=0.4, min_samples=2)  # 定义DBSCAN聚类
                labels = dbscan.fit_predict(points)  # 进行聚类

                # 统计每个聚类的点数
                unique_labels = set(labels)  # 获取唯一标签
                cluster_sizes = {label: np.sum(labels == label) for label in unique_labels}

                # 按点数排序
                sorted_clusters = sorted([(label, size) for label, size in cluster_sizes.items() if label != -1], 
                                         key=lambda x: x[1])
                selected_cluster = None
                selected_center = None

                #selected_cluster 满足distance_to_origin<=0.1点数最少的聚类
                for cluster in sorted_clusters:  # 遍历从点数最少到最多的聚类
                    label, size = cluster
                    center = np.mean(points[labels == label], axis=0)
                    distance_to_origin = np.linalg.norm(center)  # 计算质心到原点的距离
                    if distance_to_origin <= 1.0:  # 找到第一个满足条件的聚类
                        selected_cluster = label
                        selected_center = center
                        break

                if selected_center is not None:
                    rospy.loginfo(f"Attempt {len(results) + 1}: Selected center = {selected_center}")
                    results.append(selected_center)
                else:
                    rospy.logwarn("No valid cluster found in this attempt.")

                if len(results) == attempts:
                    # 计算三次结果的平均值
                    averaged_center = np.mean(results, axis=0)
                    rospy.loginfo(f"Averaged center after {attempts} attempts: {averaged_center}")
                    
                    #将center
                    current_position = self.navigator.get_robot_position()

                    current_x , current_y = current_position
                    cylinder_x = current_x + averaged_center[0]
                    cylinder_y = current_y + averaged_center[1]

                    #判断有没有到达？ 取消导航点

                    self.navigator.send_goal(cylinder_x, cylinder_y)
                    result = self.navigator.move_to_goal()
                    
                    if result:
                        if self.navigator.is_goal_reached():
                            rospy.loginfo("成功到达目标点！")
                            self.navigator.cancel_goal()
                            return 'success'
                        else:
                            rospy.logerr("未能到达目标点的距离阈值范围内。")
                            return 'aborted'
                    else:
                        rospy.logerr("导航任务失败。")
                        return 'aborted'
                

                # # 打印每个聚类的信息
                # for label in unique_labels:
                #     if label == -1:
                #         continue
                #     cluster_size = cluster_sizes[label]
                #     cluster_points = points[labels == label]
                #     if cluster_size < max_points:
                #         cluster_center = np.mean(cluster_points, axis=0)
                #         # 计算最大距离
                #         distances = np.linalg.norm(cluster_points - cluster_center, axis=1)
                #         max_distance = np.max(distances)
                #         if max_distance < thred:  # 符合条件的聚类
                #             rospy.loginfo(f'Filtered Cluster {label}: Center = {cluster_center}, Size = {cluster_size}')
                #     else:
                #         rospy.loginfo(f'Cluster {label}: Center = {cluster_center}, Size = {cluster_size}')

                # # 标记选定的聚类质心
                # if selected_cluster is not None:
                #     rospy.loginfo(f'Selected Cluster {selected_cluster}: Center = {selected_center}')
                #     return 'succeeded'
                # else:
                #     rospy.loginfo("No cluster within 1.3 meters from the origin was found.")

