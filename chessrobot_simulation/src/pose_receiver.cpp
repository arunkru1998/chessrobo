#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <chess_robot_service/motion_planning.h>  
// #include "robot_service/PoseList.h"


bool poseCallback(chess_robot_service::motion_planning::Request  &req, chess_robot_service::motion_planning::Response &res)
{
  //access the pose array portion of the service
  const geometry_msgs::PoseArray& poses = req.request;
  static const std::string PLANNING_GROUP_ARM = "manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group_arm =
      move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup* joint_model_group_gripper =
      move_group_interface_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
          move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success;
  
  // to use cartisan path planning
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  std::vector<geometry_msgs::Pose> waypoints;


  // to add a collision object to represent the table top
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
  collision_object.id = "base";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.8;
  primitive.dimensions[primitive.BOX_Y] = 3;
  primitive.dimensions[primitive.BOX_Z] = 1.03;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.55;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);


  // Set the pose target from the request
  for (const auto& pose : poses.poses)
  {
    waypoints.push_back(pose);
    // waypoints.push_back(pose.position.x-0.1);
    
    // move_group_interface_arm.setPoseTarget(pose);
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    success = (move_group_interface_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        // Execute the motion
        // move_group_interface_arm.move();
        move_group_interface_arm.execute(trajectory);
        ROS_INFO("Robot moved to the target pose successfully.");
    }
    else
    {
        ROS_ERROR("Failed to plan the motion to the target pose.");
    }
    waypoints.pop_back();
  
  }
  if (success)
  {
      // Execute the motion
      // move_group_interface_arm.move();
    res.feedback=true;
  }
  else
  {
    res.feedback=false;
  }
  return true;


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_receiver");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("robot_service", poseCallback);
  ROS_INFO("Ready to execute motion planning");

  // ros::ServiceClient client = nh.serviceClient<your_package::YourService>("your_service_name");


  // ros::Subscriber sub = nh.subscribe("pose_topic",10, poseCallback);

  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(100); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}

