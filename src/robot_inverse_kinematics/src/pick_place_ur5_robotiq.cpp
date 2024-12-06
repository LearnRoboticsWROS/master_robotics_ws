#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <frame_transform/FrameTransform.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <std_srvs/Empty.h>

const double tau = 2 * M_PI;

class PickAndPlace
{
public:
    PickAndPlace()
    {
        client_picking_pose = nh.serviceClient<frame_transform::FrameTransform>("/get_position_base_link");
        set_link_state_client = nh.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
        get_link_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        attach_timer = nh.createTimer(ros::Duration(0.01), &PickAndPlace::updateAttachment, this); // Timer a 10 ms
    }

    void attachObjectToGripper(const std::string& object_name, const std::string& wrist_link) 
    {
        attached_object_name = object_name;
        attached_reference_frame = wrist_link;
        is_attached = true; // activate the attacch

        // Get the pose of the object with the respect of the robot link
        gazebo_msgs::GetLinkState get_link_state_srv;
        get_link_state_srv.request.link_name = object_name;
        get_link_state_srv.request.reference_frame = wrist_link;

        if (get_link_state_client.call(get_link_state_srv)) {
            relative_pose = get_link_state_srv.response.link_state.pose;
        } else {
            ROS_ERROR("Failed to get the relative pose between object and wrist link.");
            is_attached = false;
        }
    }

    void detachObject() 
    {
        is_attached = false; // de activate the attaching
    }

    void close_gripper(moveit::planning_interface::MoveGroupInterface& gripper)
    {
        gripper.setMaxVelocityScalingFactor(0.5);
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.2);
        gripper.move();
        attachObjectToGripper("red_cube::red_cuboid", "ur5::wrist_3_link");        
    }

    void open_gripper(moveit::planning_interface::MoveGroupInterface& gripper)
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
        gripper.move();
        detachObject();

    }

    

    void pick(moveit::planning_interface::MoveGroupInterface& move_group)
    {

        tf2::Quaternion orientation;
        orientation.setRPY(tau/2, -tau/4, 0);
        pick_position.orientation = tf2::toMsg(orientation);
        const double eef_offset = 0.2;
        pick_position.position.x = 0 - eef_offset;
        pick_position.position.y = 0.5;
        pick_position.position.z = 0.3;
        move_group.setPoseTarget(pick_position, "tool0");
        move_group.move();

        // service.request.from_camera_to_base_link = true;

        // if (client_picking_pose.call(service))
        // {
        //     pick_position.position.x = service.response.x_base_link_frame;
        //     pick_position.position.y = service.response.y_base_link_frame;
        //     pick_position.position.z = service.response.z_base_link_frame;

        //     ROS_INFO_STREAM("Pick position (x, y, z): ("
        //                         <<
        //                         pick_position.position.x <<", "
        //                         <<
        //                         pick_position.position.y <<", " 
        //                         <<
        //                         pick_position.position.z <<")");
        //     move_group.setPoseTarget(pick_position, "picking_point");

        //     bool success = (move_group.move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);
        //     if (!success)
        //     {
        //         ROS_ERROR("MoveIt failed to plan and execute the pick motion.");
        //         return;
        //     }

        // }
        // else
        // {
        //     ROS_ERROR("Service failed to call /get_position_base_link service, aborting the pick :( ");
        //     return;
        // }

    }

    void place(moveit::planning_interface::MoveGroupInterface& move_group_place)
    {
        //geometry_msgs::Pose place_position;
        

        tf2::Quaternion orientation;
        orientation.setRPY(0, -tau/4, tau/4);
        place_position.orientation = tf2::toMsg(orientation);
        place_position.position.x = -0.6;
        place_position.position.y = 0.0;
        place_position.position.z = 0.3;
        move_group_place.setPoseTarget(place_position, "tool0");

        move_group_place.move();
    }

    
private:
        ros::NodeHandle nh;
        geometry_msgs::Pose pick_position;
        geometry_msgs::Pose place_position;

        ros::ServiceClient client_picking_pose;
        // Initialize the service
        frame_transform::FrameTransform service; 

        ros::ServiceClient set_link_state_client;
        ros::ServiceClient get_link_state_client;

        ros::Timer attach_timer;

        std::string attached_object_name;
        std::string attached_reference_frame;
        bool is_attached = false;

        geometry_msgs::Pose relative_pose;

        void updateAttachment(const ros::TimerEvent&) {
            if (!is_attached || attached_object_name.empty() || attached_reference_frame.empty()) {
                return; // do nothing, there is no attaaching
            }

            gazebo_msgs::SetLinkState set_link_state_srv;
            gazebo_msgs::LinkState link_state;

            // get the relative pose
            link_state.link_name = attached_object_name;
            link_state.reference_frame = attached_reference_frame;
            link_state.pose = relative_pose; // Use the relative pose
            link_state.twist.linear.x = 0.0;
            link_state.twist.linear.y = 0.0;
            link_state.twist.linear.z = 0.0;
            link_state.twist.angular.x = 0.0;
            link_state.twist.angular.y = 0.0;
            link_state.twist.angular.z = 0.0;

            set_link_state_srv.request.link_state = link_state;
            if (!set_link_state_client.call(set_link_state_srv)) {
                ROS_ERROR_THROTTLE(1.0, "Failed to update link attachment.");
            }
        }       
    
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
        collision_objects[0].primitives[0].dimensions[0] = 2;
        collision_objects[0].primitives[0].dimensions[1] = 0.608;
        collision_objects[0].primitives[0].dimensions[2] = 1;
        // pose of table 1
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0;
        collision_objects[0].primitive_poses[0].position.y = 0.75;
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
        collision_objects[1].primitives[0].dimensions[0] = 0.8;
        collision_objects[1].primitives[0].dimensions[1] = 1.3;
        collision_objects[1].primitives[0].dimensions[2] = 1;
        // pose of table 2
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = -1;
        collision_objects[1].primitive_poses[0].position.y = 0;
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
        joint_constraint.joint_name = "robotiq_85_left_knuckle_joint";
        joint_constraint.position = 0.7;
        joint_constraint.tolerance_above = 0.05;
        joint_constraint.tolerance_below = 0.05;
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
    moveit::planning_interface::MoveGroupInterface group("manipulator");
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