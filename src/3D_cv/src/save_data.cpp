#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*cloud_msg, cloud);

    pcl::PCDWriter cloud_writer;
    std::string path = "/home/ros/master_ws/src/3D_cv/src/";
    cloud_writer.write<pcl::PointXYZ>(path + std::string("test.pcd"), cloud, false);

}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "point_cloud_data_saving");
    ros::NodeHandle nh;

    // Create a subscriber
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/rgbd_camera/depth/points", 1, cloud_cb);
    ros::spin();
    return (0);
}