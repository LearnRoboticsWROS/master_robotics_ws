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
        // geometry_msgs::Pose pick_position;
        //group.setPoseReferenceFrame("base_link");

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
            
        }
        else
        {
            ROS_WARN("Service failed :( ");
        }




        move_group.setPoseTarget(pick_position, "picking_point");

        move_group.move();
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
        //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        //moveit::planning_interface::MoveGroupInterface group;
        //moveit::planning_interface::MoveGroupInterface gripper;
        //std::vector<moveit_msgs::CollisionObject> collision_objects;
        //moveit_msgs::AttachedCollisionObject attached_object;

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


        // // add the object
        // collision_objects[3].id = "red_cuboid";
        // collision_objects[3].header.frame_id = "base_link";

        // // Define primitive dimension, position of the table 2
        // collision_objects[3].primitives.resize(1);
        // collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
        // collision_objects[3].primitives[0].dimensions.resize(3);
        // collision_objects[3].primitives[0].dimensions[0] = 0.02;
        // collision_objects[3].primitives[0].dimensions[1] = 0.02;
        // collision_objects[3].primitives[0].dimensions[2] = 0.2;
        
        // // pose of table 2
        // collision_objects[3].primitive_poses.resize(1);
        // collision_objects[3].primitive_poses[0].position.x = 0.642;
        // collision_objects[3].primitive_poses[0].position.y = -0.031;
        // collision_objects[3].primitive_poses[0].position.z = 0.3;
        // collision_objects[3].primitive_poses[0].orientation.w = 1.0;


        // // Add tabe 2 to the scene
        // collision_objects[3].operation = collision_objects[3].ADD;

        planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    // void attachCollisionObject()
    // {
    //     //moveit_msgs::AttachedCollisionObject attached_object;
    //     attached_object.link_name = "picking_point";
    //     attached_object.object = collision_objects[3];  

    //     attached_object.object.operation = attached_object.object.ADD;

    //     std::vector<std::string> touch_links;
    //     // touch_links.push_back("picking_point");
    //     // attached_object.touch_links = touch_links;

    //     touch_links.push_back("finger_left");
    //     touch_links.push_back("finger_right");
    //     touch_links.push_back("picking_point");
    //     attached_object.touch_links = touch_links;

    //     planning_scene_interface.applyAttachedCollisionObject(attached_object);
    // }

    // void detachCollisionObject()
    // {
    //     // Specify the link to which the object is currently attached
    //     attached_object.link_name = "picking_point";

    //     // Define the operation as removing the attachment
    //     attached_object.object.operation = attached_object.object.REMOVE;

    //     // Apply the detachment operation to the planning scene
    //     planning_scene_interface.applyAttachedCollisionObject(attached_object);

    // }



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
    group.setMaxVelocityScalingFactor(1.0);
    group.setMaxAccelerationScalingFactor(1.0);


    addCollisionObject(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    addCollisionObject(planning_scene_interface);

    // If you do the following procedure, the robot goes in collision with the object that has to be picked

    PickAndPlace pick_and_place;
    
    ros::WallDuration(1.0).sleep();
    pick_and_place.pick(group);
    ros::WallDuration(2.0).sleep(); // increased to 2
    pick_and_place.close_gripper(gripper);
    ros::WallDuration(1.0).sleep();
    pick_and_place.place(group);
    ros::WallDuration(1.0).sleep();
    pick_and_place.open_gripper(gripper);
    ros::WallDuration(1.0).sleep();
    

    ros::waitForShutdown();
    return 0;
}