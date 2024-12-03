#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN

# 用于存储接收到的激光数据
data_queue = []  # 只保留最新的数据
angles = []
x = []
y = []
ok = False

# 定义回调函数
def callback(data):
    global data_queue, angles, x, y, ok
    data_queue = data.ranges
    
    # 将 data.ranges 转换为 numpy 数组
    ranges_array = np.array(data.ranges)
    angles = np.linspace(data.angle_min, data.angle_max, len(ranges_array))
    
    # 创建掩码，筛选出非零的点
    non_zero_mask = ranges_array > 0
    
    # 仅保留非零点的 x 和 y
    x = ranges_array[non_zero_mask] * np.cos(angles[non_zero_mask])
    y = ranges_array[non_zero_mask] * np.sin(angles[non_zero_mask])
    
    ok = True  # 更新ok标志位

def listener():
    global data_queue, angles, x, y, ok
    rospy.init_node('scan_listener', anonymous=True)  # 初始化 ROS 节点
    rospy.Subscriber("scan", LaserScan, callback)  # 订阅 scan 话题

    plt.figure(figsize=(12, 8))  # 设置图形大小

    while not rospy.is_shutdown():
        if data_queue and ok:  # 检查队列中是否有数据
            plt.clf()  # 清除之前的图形
            
            
            max_points = 11  # 默认最大点数
            thred = 0.1  # 默认最大距离
            
            # 聚类
            points = np.vstack((x, y)).T  # 创建用于聚类的点
            dbscan = DBSCAN(eps=1, min_samples=1)  # 定义DBSCAN聚类
            labels = dbscan.fit_predict(points)  # 进行聚类

            # 统计每个聚类的点数
            unique_labels = set(labels)  # 获取唯一标签
            cluster_sizes = {label: np.sum(labels == label) for label in unique_labels}

            # 打印每个聚类的信息
            # for label, size in cluster_sizes.items():
            #     if label == -1:
            #         # 噪声点
            #         continue
            #     print(f'Cluster {label}: {size} points')

           
            
           
            for label in unique_labels:
                if label == -1:
                    continue
                cluster_size = cluster_sizes[label]
                cluster_points = points[labels == label]
                if cluster_size < max_points:
                    cluster_center = np.mean(cluster_points, axis=0)
                    # 计算最大距离
                    distances = np.linalg.norm(cluster_points - cluster_center, axis=1)
                    max_distance = np.max(distances)
                    # print(f'Cluster {label}: {cluster_size} points, max distance: {max_distance:.2f} meters')
                    if max_distance < thred:  # 符合条件的聚类
                        plt.scatter(cluster_points[:, 0], cluster_points[:, 1], 
                                s=1, color='red', label=f'Filtered Cluster {label}')  # 符合条件的聚类使用黑色显示
                        print(f'Filtered Cluster {label}: {cluster_size} points, ceter: {cluster_center}, max distance: {max_distance:.2f} meters')
                else:
                    plt.scatter(cluster_points[:, 0], cluster_points[:, 1], s=1, label=f'Cluster {label}')  # 其他聚类使用灰色显示

            plt.title("Filtered Clusters in Cartesian Coordinates")
            plt.xlabel("X (meters)")
            plt.ylabel("Y (meters)")

            plt.axis('equal')  # 确保 X 和 Y 轴比例相同
            plt.grid(True)  # 显示网格

            plt.tight_layout()  # 自动调整子图参数
            plt.pause(1)  # 暂停以更新图形
            ok = False  # 重置ok标志位

    plt.show()  # 显示最终图形

if __name__ == '__main__':
    listener()