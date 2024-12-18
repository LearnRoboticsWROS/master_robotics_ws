#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <iostream>

std::vector<geometry_msgs::Pose> waypoints;

void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    waypoints = msg->poses;
    ROS_INFO("Received %lu waypoints", waypoints.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_executor");
    ros::NodeHandle nh;

    ros::Subscriber waypoint_sub = nh.subscribe("waypoints", 10, waypointsCallback);
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPoseReferenceFrame("base_link");

    ROS_INFO("Waypoint Executor Node Initialized");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        ROS_INFO("Press [Enter] to execute the trajectory through saved waypoints...");
        std::cin.get(); // Wait for user to press Enter

        if (waypoints.empty())
        {
            ROS_WARN("No waypoints to execute. Add waypoints first.");
            ros::Duration(2.0).sleep();
            continue;
        }

        ROS_INFO("Executing trajectory through %lu waypoints", waypoints.size());

        // Debug: Print all waypoints
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            ROS_INFO("Waypoint %lu: Position(%.2f, %.2f, %.2f), Orientation(%.2f, %.2f, %.2f, %.2f)",
                     i, waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z,
                     waypoints[i].orientation.x, waypoints[i].orientation.y,
                     waypoints[i].orientation.z, waypoints[i].orientation.w);
        }

        // Create a trajectory from waypoints
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.05;  // play
        const double jump_threshold = 0.0; // play
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 1.0)
        {
            ROS_WARN("Could not compute the full trajectory. Fraction: %.2f", fraction);

            // Attempt interpolation if fraction is low
            ROS_INFO("Attempting to interpolate waypoints for better trajectory...");
            std::vector<geometry_msgs::Pose> interpolated_waypoints;

            for (size_t i = 1; i < waypoints.size(); ++i)
            {
                geometry_msgs::Pose start = waypoints[i - 1];
                geometry_msgs::Pose end = waypoints[i];

                // Linear interpolation
                for (double t = 0.0; t <= 1.0; t += 0.01)
                {
                    geometry_msgs::Pose interp;
                    interp.position.x = start.position.x + t * (end.position.x - start.position.x);
                    interp.position.y = start.position.y + t * (end.position.y - start.position.y);
                    interp.position.z = start.position.z + t * (end.position.z - start.position.z);

                    interp.orientation = start.orientation; 
                    interpolated_waypoints.push_back(interp);
                }
            }

            // Recompute Cartesian path with interpolated waypoints
            fraction = move_group.computeCartesianPath(interpolated_waypoints, eef_step, jump_threshold, trajectory);
            if (fraction < 1.0)
            {
                ROS_WARN("Even with interpolation, could not compute the full trajectory. Fraction: %.2f", fraction);
                continue;
            }
        }
        

        ROS_INFO("Trajectory computed successfully. Executing...");

        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);

        ROS_INFO("Trajectory execution completed.");

        ros::Duration(2.0).sleep(); // Add a delay before next execution
    }

    return 0;
}