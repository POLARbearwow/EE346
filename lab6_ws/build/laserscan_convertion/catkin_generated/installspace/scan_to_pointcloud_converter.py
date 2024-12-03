#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class ScanToPointCloud {
public:
    ScanToPointCloud() {
        // Initialize ROS
        ros::NodeHandle nh;
        // Create a publisher for the PointCloud data
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
        // Create a subscriber for LaserScan messages
        scan_sub = nh.subscribe("/scan", 1, &ScanToPointCloud::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Convert LaserScan to PointCloud2
        sensor_msgs::PointCloud2 cloud;
        try {
            projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, listener_);
            cloud.header.frame_id = "base_link";
            cloud.header.stamp = scan->header.stamp;
            // Publish the PointCloud
            cloud_pub.publish(cloud);
        } catch (tf::TransformException &ex) {
            ROS_WARN("Error during scan to point cloud conversion: %s", ex.what());
        }
    }

private:
    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_to_pointcloud_converter");
    ScanToPointCloud converter;
    ros::spin();
    return 0;
}
