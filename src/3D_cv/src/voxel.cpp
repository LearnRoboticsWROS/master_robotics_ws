#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Conversion to PCL
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // perform the filtering through Voxel
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize(0.05, 0.05, 0.05); // PLAY
    sor.filter(cloud_filtered);

    // Convert from PCL to ROS data type
    sensor_msgs::PointCloud2 filtered_points;
    pcl_conversions::moveFromPCL(cloud_filtered, filtered_points);

    // Publish the data
    pub.publish(filtered_points);

}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "voxel_filter");
    ros::NodeHandle nh;

    // Create a subscriber
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/rgbd_camera/depth/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("filtered", 1);
    ros::spin();
    return (0);
}