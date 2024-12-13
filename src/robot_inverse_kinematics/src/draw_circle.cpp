#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>

const double tau = 2 * M_PI;

class GeometryTracer
{
public:
    GeometryTracer()
    {

    }

    void gotoStartPosition(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        // Circle parameter
        const double radius = 0.1; 
        const double center_x = 0.0;
        const double center_y = 0.4;
        const double center_z = 0.3;
        const int num_waypoints = 50;

        // Step 1: Move to the center of the circle with a specific joint configuration
        tf2::Quaternion orientation;
        orientation.setRPY(-tau/2, 0, tau/2);
        initial_target_pose.orientation = tf2::toMsg(orientation);
        initial_target_pose.position.x = center_x;
        initial_target_pose.position.y = center_y;
        initial_target_pose.position.z = center_z;

        // definition od joint contraints
        moveit_msgs::Constraints joint_constraints_ur5;

        // Constraint for joint1
        moveit_msgs::JointConstraint joint1_constraint;
        joint1_constraint.joint_name = "shoulder_pan_joint";
        joint1_constraint.position = (50.0 + 80.0) / 2 * M_PI / 180; 
        joint1_constraint.tolerance_above = (80.0 - 50.0) * M_PI / 180 / 2;
        joint1_constraint.tolerance_below = (80.0 - 50.0) * M_PI / 180 / 2;
        joint1_constraint.weight = 1.0;

        // Constraint for joint2
        moveit_msgs::JointConstraint joint2_constraint;
        joint2_constraint.joint_name = "shoulder_lift_joint";
        joint2_constraint.position = -(90.0 + 120.0) / 2 * M_PI / 180;
        joint2_constraint.tolerance_above = (120 - 90) * M_PI / 180 / 2;
        joint2_constraint.tolerance_below = (120 - 90) * M_PI / 180 / 2;
        joint2_constraint.weight = 1.0;

        // Constraint for joint3
        moveit_msgs::JointConstraint joint3_constraint;
        joint3_constraint.joint_name = "elbow_joint";
        joint3_constraint.position = (100.0 + 120.0) / 2 * M_PI / 180;
        joint3_constraint.tolerance_above = (120.0 - 100.0) * M_PI / 180 / 2;
        joint3_constraint.tolerance_below = (120.0 - 100.0) * M_PI / 180 / 2;
        joint3_constraint.weight = 1.0;

        moveit_msgs::JointConstraint joint4_constraint;
        joint4_constraint.joint_name = "wrist_1_joint";
        joint4_constraint.position = (220.0 + 270.0) / 2 * M_PI / 180;
        joint4_constraint.tolerance_above = (270.0 - 220.0) * M_PI / 180 / 2;
        joint4_constraint.tolerance_below = (270.0 - 220.0) * M_PI / 180 / 2;
        joint4_constraint.weight = 1.0;

        moveit_msgs::JointConstraint joint5_constraint;
        joint5_constraint.joint_name = "wrist_2_joint";
        joint5_constraint.position = (250.0 + 280.0) / 2 * M_PI / 180;
        joint5_constraint.tolerance_above = (280.0 - 250.0) * M_PI / 180 / 2;
        joint5_constraint.tolerance_below = (280.0 - 250.0) * M_PI / 180 / 2;
        joint5_constraint.weight = 1.0;

        // add constraint to the list
        joint_constraints_ur5.joint_constraints.push_back(joint1_constraint);
        joint_constraints_ur5.joint_constraints.push_back(joint2_constraint);
        joint_constraints_ur5.joint_constraints.push_back(joint3_constraint);
        joint_constraints_ur5.joint_constraints.push_back(joint4_constraint);
        joint_constraints_ur5.joint_constraints.push_back(joint5_constraint);

        // setup the constraint to the MoveGroup
        move_group.setPathConstraints(joint_constraints_ur5);

        move_group.setPoseTarget(initial_target_pose, "tool0");

        move_group.move();
        ros::WallDuration(2.0).sleep(); // deleay for syncronization
        move_group.setStartStateToCurrentState(); // update the initial state

    }
    void traceCircle(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        // Circle parameter
        const double radius = 0.1; 
        const double center_x = 0.0;
        const double center_y = 0.4;
        const double center_z = 0.3;
        const int num_waypoints = 50;

        move_group.clearPathConstraints();
        start_pose = move_group.getCurrentPose().pose;
        double position_tolerance = 0.01; // tolleranza in metri
        double orientation_tolerance = 0.01; // tolleranza in orientazione

        if (fabs(start_pose.position.x - initial_target_pose.position.x) > position_tolerance ||
            fabs(start_pose.position.y - initial_target_pose.position.y) > position_tolerance ||
            fabs(start_pose.position.z - initial_target_pose.position.z) > position_tolerance) {
            ROS_WARN("Robot is not in the initial target pose. Adjusting...");
            move_group.setPoseTarget(initial_target_pose);
            move_group.move();
            ros::WallDuration(1.0).sleep();
        }

        tf2::Quaternion orientation2;

        for (int i = 0; i < num_waypoints; ++i)
        {
            double angle = 2 * M_PI * i / num_waypoints; // current angle

            target_pose = start_pose;
            target_pose.position.x = center_x + radius * cos(angle);
            target_pose.position.y = center_y + radius * sin(angle);
            target_pose.position.z = center_z;
            orientation2.setRPY(-tau/2, 0, tau/2);  // play if you want to change the orientation of the eef
            target_pose.orientation = tf2::toMsg(orientation2);
            waypoints.push_back(target_pose);
        }
        

        // Plan the trajectory based on waypoints
        const double eef_step = 0.05;  // play
        const double jump_threshold = 1.0; // play
        const double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        ROS_INFO("pathway calculated with a fraction of success: %.2f", fraction);

        if (fraction > 0.9)
        {
            // execute the trajectory
            move_group.execute(trajectory);
            ROS_INFO("Trajectoory executed with success.");
        }
        else
        {
            ROS_WARN("Trajectory not well calculated.");
        }
    }




private:
    // moveit::planning_interface::MoveGroupInterface move_group;
    ros::NodeHandle nh;
    geometry_msgs::Pose initial_target_pose;
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose target_pose;

    std::vector<geometry_msgs::Pose> waypoints;

    moveit_msgs::RobotTrajectory trajectory;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracer_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    group.setPlanningTime(10.0);
    group.setMaxVelocityScalingFactor(0.2);
    group.setMaxAccelerationScalingFactor(0.2);

    group.setPoseReferenceFrame("base_link");
    group.setPlanningTime(60.0);

    ros::WallDuration(1.0).sleep();

    GeometryTracer tracer;
    tracer.gotoStartPosition(group);
    ros::WallDuration(1.0).sleep();
    tracer.traceCircle(group);

    ros::shutdown();
    return 0;
}