#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <frame_transform/FrameTransform.h>

const double tau = 2 * M_PI;


class PickAndPlace
{
public:
    PickAndPlace()
    {
        client_picking_pose = nh.serviceClient<frame_transform::FrameTransform>("/get_position_base_link");
    }


    void close_gripper(moveit::planning_interface::MoveGroupInterface& gripper)
    {
        gripper.setMaxVelocityScalingFactor(0.5);
        gripper.setJointValueTarget("finger_right_joint", 0.04);
        gripper.move();
    }

    void open_gripper(moveit::planning_interface::MoveGroupInterface& gripper)
    {
        gripper.setJointValueTarget("finger_right_joint", 0.0);
        gripper.move();
    }

    

    void pick(moveit::planning_interface::MoveGroupInterface& move_group)
    {

        tf2::Quaternion orientation;
        orientation.setRPY(-tau/4, 0, -tau/2);
        pick_position.orientation = tf2::toMsg(orientation);
        // pick_position.position.x = 0.658;
        // pick_position.position.y = -0.036;
        // pick_position.position.z = 0.3;

        service.request.from_camera_to_base_link = true;

        if (client_picking_pose.call(service))
        {
            pick_position.position.x = service.response.x_base_link_frame;
            pick_position.position.y = service.response.y_base_link_frame;
            pick_position.position.z = service.response.z_base_link_frame;

            ROS_INFO_STREAM("Pick position (x, y, z): ("
                                <<
                                pick_position.position.x <<", "
                                <<
                                pick_position.position.y <<", " 
                                <<
                                pick_position.position.z <<")");
            move_group.setPoseTarget(pick_position, "picking_point");

            bool success = (move_group.move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_ERROR("MoveIt failed to plan and execute the pick motion.");
                return;
            }

        }
        else
        {
            ROS_ERROR("Service failed to call /get_position_base_link service, aborting the pick :( ");
            return;
        }

    }

    void place(moveit::planning_interface::MoveGroupInterface& move_group_place)
    {
        //geometry_msgs::Pose place_position;
        

        tf2::Quaternion orientation;
        orientation.setRPY(-tau/4, 0, 0);
        place_position.orientation = tf2::toMsg(orientation);
        place_position.position.x = 0;
        place_position.position.y = 0.8;
        place_position.position.z = 0.5;
        move_group_place.setPoseTarget(place_position, "picking_point");

        move_group_place.move();
    }

    
private:
        ros::NodeHandle nh;
        geometry_msgs::Pose pick_position;
        geometry_msgs::Pose place_position;

        ros::ServiceClient client_picking_pose;
        // Initialize the service
        frame_transform::FrameTransform service;

    
};

    void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
    {
    // add collision objects
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(3);

        // Add the first table
        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "base_link";

        // Define primitive dimension, position of the table 1
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 0.608;
        collision_objects[0].primitives[0].dimensions[1] = 2;
        collision_objects[0].primitives[0].dimensions[2] = 1;
        // pose of table 1
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.75;
        collision_objects[0].primitive_poses[0].position.y = 0;
        collision_objects[0].primitive_poses[0].position.z = -0.3;
        collision_objects[0].primitive_poses[0].orientation.w = 1.0;
        // Add tabe 1 to the scene

        collision_objects[0].operation = collision_objects[0].ADD;


        // Add the second table
        collision_objects[1].id = "table2";
        collision_objects[1].header.frame_id = "base_link";

        // Define primitive dimension, position of the table 2
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
        collision_objects[1].primitives[0].dimensions.resize(3);
        collision_objects[1].primitives[0].dimensions[0] = 1.3;
        collision_objects[1].primitives[0].dimensions[1] = 0.8;
        collision_objects[1].primitives[0].dimensions[2] = 1;
        // pose of table 2
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = 0;
        collision_objects[1].primitive_poses[0].position.y = 1;
        collision_objects[1].primitive_poses[0].position.z = -0.3;
        collision_objects[1].primitive_poses[0].orientation.w = 1.0;
        // Add tabe 2 to the scene
        collision_objects[1].operation = collision_objects[1].ADD;


        // add the basement
        collision_objects[2].id = "basement";
        collision_objects[2].header.frame_id = "base_link";

        // Define primitive dimension, position of the table 2
        collision_objects[2].primitives.resize(1);
        collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].CYLINDER;
        collision_objects[2].primitives[0].dimensions.resize(2);
        collision_objects[2].primitives[0].dimensions[0] = 0.8;
        collision_objects[2].primitives[0].dimensions[1] = 0.2;
        
        // pose of table 2
        collision_objects[2].primitive_poses.resize(1);
        collision_objects[2].primitive_poses[0].position.x = 0;
        collision_objects[2].primitive_poses[0].position.y = 0;
        collision_objects[2].primitive_poses[0].position.z = -0.4;
        collision_objects[2].primitive_poses[0].orientation.w = 1.0;
        // Add tabe 2 to the scene
        collision_objects[2].operation = collision_objects[2].ADD;


        planning_scene_interface.applyCollisionObjects(collision_objects);
    }


    void setGripperConstraints(moveit::planning_interface::MoveGroupInterface& gripper)
    {
        moveit_msgs::Constraints gripper_constraints;
        moveit_msgs::JointConstraint joint_constraint;
        joint_constraint.joint_name = "finger_right_joint";
        joint_constraint.position = 0.04;
        joint_constraint.tolerance_above = 0.001;
        joint_constraint.tolerance_below = 0.001;
        joint_constraint.weight = 1.0;

        gripper_constraints.joint_constraints.push_back(joint_constraint);
        gripper.setPathConstraints(gripper_constraints);
    }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cobot_pick_and_place");
    //ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    group.setPoseReferenceFrame("base_link");
    group.setPlanningTime(45.0);

    // setup the speed and accelleration
    group.setMaxVelocityScalingFactor(1.0);   // added
    group.setMaxAccelerationScalingFactor(1.0);   // added


    addCollisionObject(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    // If you do the following procedure, the robot goes in collision with the object that has to be picked

    PickAndPlace pick_and_place;
    
    ros::WallDuration(1.0).sleep();
    setGripperConstraints(gripper);
    pick_and_place.pick(group);
    gripper.clearPathConstraints();
    ros::WallDuration(2.0).sleep(); // increased to 2
    pick_and_place.close_gripper(gripper);
    ros::WallDuration(1.0).sleep();
    setGripperConstraints(gripper);
    pick_and_place.place(group);
    gripper.clearPathConstraints();
    ros::WallDuration(1.0).sleep();
    pick_and_place.open_gripper(gripper);
    ros::WallDuration(1.0).sleep();
    

    ros::waitForShutdown();
    return 0;
}