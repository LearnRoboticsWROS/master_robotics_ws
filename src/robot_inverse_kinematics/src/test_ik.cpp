#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    // moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher= nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // modify the object

    //Setup the target poistion
    group.setPoseReferenceFrame("base_link");

    geometry_msgs::Pose target_pose1;
    
    tf2::Quaternion orientation;
    orientation.setRPY(tau/2, -tau/4, 0);
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = -0.25;
    target_pose1.position.y = 0.5;
    target_pose1.position.z = 0.3;
    

    // group.setPoseTarget(target_pose1, "picking_point");
    group.setPoseTarget(target_pose1, "tool0");

    // visualize the planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    ROS_INFO("Visualizing plan %s", success.val ? "":"FAILED");

    // execution
    group.move();

    ros::shutdown();
    return 0;



}