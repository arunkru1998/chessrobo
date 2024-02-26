#include <ros/ros.h>
// Moveit
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// tau = 1 rotation in radians
const double tau = 2* M_PI;


void openGripper(trajectory_msgs::JointTrajectory& posture)
{

}




void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
}


void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    //grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;

    //pick test 1
    // orientation.setRPY(0, 0, 0);
    // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.orientation.x = 0.001793;   
    grasps[0].grasp_pose.pose.orientation.y = -0.000004;
    grasps[0].grasp_pose.pose.orientation.z = 0.000630;
    grasps[0].grasp_pose.pose.orientation.w = 0.999998;
    grasps[0].grasp_pose.pose.position.x = -0.48;
    grasps[0].grasp_pose.pose.position.y = 0.107;
    grasps[0].grasp_pose.pose.position.z = 0.53;

    // // Pre-grasp approach
    // grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // // the grasp will move in y direction
    // grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
    // grasps[0].pre_grasp_approach.min_distance = 0.095;
    // grasps[0].pre_grasp_approach.desired_distance = 0.115;
    // // now the object is picked, we need to retreat in a certain direction the end effector
    // // let's retreat the end effector upwards: positive z direction
    // grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    // grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    // grasps[0].post_grasp_retreat.min_distance = 0.1;
    // grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // we need to open the gripper. We will define a function for that
    openGripper(grasps[0].pre_grasp_posture);

    // When it grasps it need to close the gripper

    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table1
    move_group.setSupportSurfaceName("table1");

    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
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
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;
    // pose of table 1
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.1;
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
    collision_objects[1].primitives[0].dimensions[0] = 0.2;
    collision_objects[1].primitives[0].dimensions[1] = 0.4;
    collision_objects[1].primitives[0].dimensions[2] = 0.2;
    // pose of table 2
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.1;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the scene
    collision_objects[1].operation = collision_objects[1].ADD;

    // Add the object to be picked
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";

    // Define primitive dimension, position of the object
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.1;
    // pose of object
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.25;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the object
    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"ur5_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);  
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setPlanningTime(45.0);

    // Put the object in the scene
    addCollisionObject(planning_scene_interface);

    // wait for initialization
    ros::WallDuration(1.0).sleep();

    //Pick the object
    pick(group);

    ros::WallDuration(1.0).sleep();

    //Place the object
    place(group);

    ros::waitForShutdown();
}