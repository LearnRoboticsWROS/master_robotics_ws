#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>

// Function to wait for Enter key press
void waitForEnter() {
    std::cout << "Press Enter to save the current waypoint..." << std::endl;
    std::cin.get();
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "waypoint_saver");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseArray>("/waypoints", 10);

    geometry_msgs::PoseArray waypoints;
    waypoints.header.frame_id = "base_link";

    ROS_INFO("Press Enter to save waypoints...");

    while (ros::ok()) 
    {
        tf::StampedTransform transform;
        try 
        {
            // Listen to the transform from the base frame to the end effector
            listener.lookupTransform("base_link", "tool0", ros::Time(0), transform);

            // Wait for Enter key press
            waitForEnter();
            geometry_msgs::Pose waypoint;

            // Fill the waypoint with the end effector's pose
            waypoint.position.x = transform.getOrigin().x();
            waypoint.position.y = transform.getOrigin().y();
            waypoint.position.z = transform.getOrigin().z();
            waypoint.orientation.x = transform.getRotation().x();
            waypoint.orientation.y = transform.getRotation().y();
            waypoint.orientation.z = transform.getRotation().z();
            waypoint.orientation.w = transform.getRotation().w();

            // Add waypoint to the array
            waypoints.poses.push_back(waypoint);

            // Update the header timestamp and publish the array
            waypoints.header.stamp = ros::Time::now();
            waypoint_pub.publish(waypoints);

            ROS_INFO("Waypoint saved at x: %f, y: %f, z: %f",
                     waypoint.position.x,
                     waypoint.position.y,
                     waypoint.position.z);

        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }

        ros::spinOnce();
    }
    return 0;
}

