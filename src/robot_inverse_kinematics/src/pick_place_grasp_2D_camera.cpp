#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <frame_transform/FrameTransform.h>

// tau = 1 rotation in radiants
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  
    posture.joint_names.resize(1);
    posture.joint_names[0] = "finger_right_joint";
    
    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0;
    posture.points[0].time_from_start = ros::Duration(0.5);

    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    gripper.clearPathConstraints();


}


void closedGripper(trajectory_msgs::JointTrajectory& posture)
{

    posture.joint_names.resize(1);
    posture.joint_names[0] = "finger_right_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);

    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    gripper.clearPathConstraints();

    


}

void pick_object(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-tau/4, 0, tau/2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

    ros::NodeHandle nh;
    ros::ServiceClient client_picking_pose;
    // Initialize the service
    frame_transform::FrameTransform service;
    client_picking_pose = nh.serviceClient<frame_transform::FrameTransform>("/get_position_base_link");

    const double eef_offset = 0.1;

    service.request.from_camera_to_base_link = true;
    // grasps[0].grasp_pose.pose.position.x = 0.6;
    // grasps[0].grasp_pose.pose.position.y = 0.1;
    // grasps[0].grasp_pose.pose.position.z = 0.3;
    
    if (client_picking_pose.call(service))
    {
        double x_pose = service.response.x_base_link_frame;
        double y_pose = service.response.y_base_link_frame;
        double z_pose = service.response.z_base_link_frame;
    
        // add collision object to the position detected
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::CollisionObject> collision_object;
        //moveit_msgs::CollisionObject collision_object;
        collision_object.resize(1);
        collision_object[0].id = "object";
        collision_object[0].header.frame_id = "base_link";
        collision_object[0].primitives.resize(1);
        collision_object[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        collision_object[0].primitives[0].dimensions = {0.02, 0.02, 0.2};

        collision_object[0].primitive_poses.resize(1);
        collision_object[0].primitive_poses[0].position.x = x_pose;
        collision_object[0].primitive_poses[0].position.y = y_pose;
        collision_object[0].primitive_poses[0].position.z = z_pose;

        collision_object[0].operation = moveit_msgs::CollisionObject::ADD;
        planning_scene_interface.applyCollisionObjects(collision_object);

        grasps[0].grasp_pose.pose.position.x = x_pose;
        grasps[0].grasp_pose.pose.position.y = y_pose + eef_offset;
        grasps[0].grasp_pose.pose.position.z = z_pose;
    }

    else
    {
        ROS_WARN("service failed");
    }
    

    // Pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // Direction is set as positive x axis
    grasps[0].pre_grasp_approach.direction.vector.y = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.02;
    grasps[0].pre_grasp_approach.desired_distance = 0.025;

    // Post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    //Direction is set as positive z axis 
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.05;
    grasps[0].post_grasp_retreat.desired_distance = 0.1;

    // we need to open the gripper. We will define a function for that
    openGripper(grasps[0].pre_grasp_posture);

    // When it grasps it needs to close the gripper
    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table 1
    move_group.setSupportSurfaceName("table1");



    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);

    
}

void place_object(moveit::planning_interface::MoveGroupInterface& group)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose

    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;

    orientation.setRPY(-tau/2, 0, -tau / 4);  // A quarter turn about the z-axis
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.6;
    place_location[0].place_pose.pose.position.z = 0.5;

    // Setting pre-place approach
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    // Direction is set as negative z axis
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.01;
    place_location[0].pre_place_approach.desired_distance = 0.015;

    

    // Setting post-grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    // Direction is set as negative y axis
    place_location[0].post_place_retreat.direction.vector.x = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.05;
    place_location[0].post_place_retreat.desired_distance = 0.08;



    openGripper(place_location[0].post_place_posture);

    // Set support surface as table 2
    group.setSupportSurfaceName("table2");
    //group.setPoseTarget(place_location[0].place_pose, "picking_point");

    
    // Call place to palce the object using the place location given
    group.place("object", place_location);

    ROS_INFO_STREAM("x = "
                    << 
                    place_location[0].place_pose.pose.position.x 
                    << ", y = " << 
                    place_location[0].place_pose.pose.position.y 
                    << ", z = " << 
                    place_location[0].place_pose.pose.position.z);


}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
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
    collision_objects[0].primitive_poses[0].position.x = 0.8;
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
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 1.3;
    collision_objects[1].primitives[0].dimensions[1] = 0.8;
    collision_objects[1].primitives[0].dimensions[2] = 1;
    // pose of table 2
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.8;
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
    collision_objects[2].primitive_poses[0].position.z = -0.41;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the scene
    collision_objects[2].operation = collision_objects[2].ADD;



    // Add the object to be picked
    // collision_objects[3].id = "object";
    // collision_objects[3].header.frame_id = "base_link";

    // // Define primitive dimension, position of the object
    // collision_objects[3].primitives.resize(1);
    // collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
    // collision_objects[3].primitives[0].dimensions.resize(3);
    // collision_objects[3].primitives[0].dimensions[0] = 0.02;
    // collision_objects[3].primitives[0].dimensions[1] = 0.02;
    // collision_objects[3].primitives[0].dimensions[2] = 0.2;
    // // pose of object
    // collision_objects[3].primitive_poses.resize(1);
    // collision_objects[3].primitive_poses[0].position.x = 0;
    // collision_objects[3].primitive_poses[0].position.y = 0.6;
    // collision_objects[3].primitive_poses[0].position.z = 0.3;
    // collision_objects[3].primitive_poses[0].orientation.w = 1.0;
    // // Add tabe 2 to the object
    // collision_objects[3].operation = collision_objects[3].ADD;

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
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    group.setPlanningTime(45.0);

    // Put the object in the scene
    addCollisionObject(planning_scene_interface);

    // Wait for initialization
    ros::WallDuration(1.0).sleep();
    setGripperConstraints(gripper);
    // Pick the object
    pick_object(group);
    ros::WallDuration(1.0).sleep();
    setGripperConstraints(gripper);
    // Place the object
    place_object(group);
    ros::waitForShutdown();
    return 0;

}