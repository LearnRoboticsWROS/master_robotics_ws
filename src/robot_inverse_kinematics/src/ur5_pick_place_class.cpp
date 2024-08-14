#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
const double tau = 2 * M_PI;

class PickAndPlace
{
public:
    PickAndPlace()
        : group("manipulator"), gripper("gripper")
    {
        group.setPlanningTime(45.0);
    }

    void close_gripper()
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.1);
        gripper.move();
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
        gripper.move();
    }

    void pick()
    {
        geometry_msgs::Pose pick_position;
        group.setPoseReferenceFrame("base_link");

        tf2::Quaternion orientation;
        orientation.setRPY(tau/4, -tau/4, tau/2);
        pick_position.orientation = tf2::toMsg(orientation);
        pick_position.position.x = 0.634;
        pick_position.position.y = -0.16;
        pick_position.position.z = 0.360;
        group.setPoseTarget(pick_position, "tool0");

        group.move();
    }

    void place()
    {
        geometry_msgs::Pose place_position;
        group.setPoseReferenceFrame("base_link");

        tf2::Quaternion orientation;
        orientation.setRPY(tau/4, -tau/4, tau/2);
        place_position.orientation = tf2::toMsg(orientation);
        place_position.position.x = 0;
        place_position.position.y = 0.8;
        place_position.position.z = 0.5;
        group.setPoseTarget(place_position, "tool0");

        group.move();
    }

    void addCollisionObject()
    {
        // add collision objects
        //std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(4);

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


        // add the object
        collision_objects[3].id = "red_cuboid";
        collision_objects[3].header.frame_id = "base_link";

        // Define primitive dimension, position of the table 2
        collision_objects[3].primitives.resize(1);
        collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
        collision_objects[3].primitives[0].dimensions.resize(3);
        collision_objects[3].primitives[0].dimensions[0] = 0.02;
        collision_objects[3].primitives[0].dimensions[1] = 0.02;
        collision_objects[3].primitives[0].dimensions[2] = 0.2;
        
        // pose of table 2
        collision_objects[3].primitive_poses.resize(1);
        collision_objects[3].primitive_poses[0].position.x = 0.642;
        collision_objects[3].primitive_poses[0].position.y = -0.031;
        collision_objects[3].primitive_poses[0].position.z = 0.3;
        collision_objects[3].primitive_poses[0].orientation.w = 1.0;


        // Add tabe 2 to the scene
        collision_objects[3].operation = collision_objects[3].ADD;

        planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    void attachCollisionObject()
    {
        //moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "tool0";
        attached_object.object = collision_objects[3];  

        attached_object.object.operation = attached_object.object.ADD;

        std::vector<std::string> touch_links;
        touch_links.push_back("tool0");
        attached_object.touch_links = touch_links;

        planning_scene_interface.applyAttachedCollisionObject(attached_object);
    }

    void detachCollisionObject()
    {
        attached_object.object.operation = attached_object.object.REMOVE;
        //attached_object.link_name = "picking_point";
        //attached_object.object = collision_objects[3];
        planning_scene_interface.applyAttachedCollisionObject(attached_object);

    }

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group;
    moveit::planning_interface::MoveGroupInterface gripper;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::AttachedCollisionObject attached_object;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cobot_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();

    PickAndPlace pick_and_place;
    pick_and_place.addCollisionObject();
    ros::WallDuration(1.0).sleep();
    pick_and_place.pick();
    ros::WallDuration(1.0).sleep();
    pick_and_place.close_gripper();
    ros::WallDuration(1.0).sleep();
    pick_and_place.attachCollisionObject();
    ros::WallDuration(1.0).sleep();
    pick_and_place.place();
    ros::WallDuration(1.0).sleep();
    pick_and_place.open_gripper();
    ros::WallDuration(1.0).sleep();
    pick_and_place.detachCollisionObject();
    

    ros::waitForShutdown();
    return 0;
}