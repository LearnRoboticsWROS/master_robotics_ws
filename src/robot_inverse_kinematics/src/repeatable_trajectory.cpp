#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_msgs/RobotState.h> // add 
#include <moveit/robot_state/conversions.h> // add for conversion from a message of type moveit::core::RobotState to a ROS msg of type moveit_msgs::RobotState
const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    // node definition and spinner
    ros::init(argc, argv, "move_group_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    // instantiate the moveit object MoveGroupInterface PlanningScene Interface
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher= nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    //Setup the target poistion
    group.setPoseReferenceFrame("base_link");

     // 1. move the robot in position up
    ROS_INFO("Moving to 'up' position...");
    group.setNamedTarget("up"); // use the configuration saved inmoveit_setup_assistan
    group.move();

    // 2. set the seed to guarantee repeateble trajectories from the point "up"
    ROS_INFO("Setting seed and planning to target...");
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    std::vector<double> seed_joint_positions;
    current_state->copyJointGroupPositions(group.getName(), seed_joint_positions); // copy the actual state position (up)

    // modify thee sees if necessary for testing/debuging:
    //seed_joint_positions = {0.0, -tau / 4, tau / 4, 0.0, tau / 4, 0.0};

    current_state->setJointGroupPositions(group.getName(), seed_joint_positions);
    moveit_msgs::RobotState seed_robot_state;
    moveit::core::robotStateToRobotStateMsg(*current_state, seed_robot_state);
    group.setStartState(seed_robot_state);

    // 3. plan the movement to the target
    geometry_msgs::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(tau / 2, -tau / 4, 0); // specific rotation
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = -0.25;
    target_pose.position.y = 0.5;
    target_pose.position.z = 0.3;

    group.setPoseTarget(target_pose, "tool0");

    moveit::planning_interface::MoveGroupInterface::Plan plan_to_target;
    
    moveit::planning_interface::MoveItErrorCode  success = group.plan(plan_to_target);
    if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        group.execute(plan_to_target);
        ROS_INFO("Reached target position.");
    }
    else
    {
        ROS_ERROR("Failed to plan to target position.");
        ros::shutdown();
        return 1;
    }

    ros::shutdown();
    return 0;
}

