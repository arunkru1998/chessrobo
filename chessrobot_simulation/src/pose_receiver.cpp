#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>  
// #include "robot_service/PoseList.h"


void poseCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  static const std::string PLANNING_GROUP_ARM = "manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  
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
  
  // Set the pose target
  for (const auto& pose : msg->poses)
  {
    move_group_interface_arm.setPoseTarget(pose);
    
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        // Execute the motion
        move_group_interface_arm.move();
        ROS_INFO("Robot moved to the target pose successfully.");
    }
    else
    {
        ROS_ERROR("Failed to plan the motion to the target pose.");
    }
  }
  // // Set the pose target
  // move_group_interface_arm.setPoseTarget(*msg);
  
  // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // if (success)
  // {
  //     // Execute the motion
  //     move_group_interface_arm.move();
  //     ROS_INFO("Robot moved to the target pose successfully.");
  // }
  // else
  // {
  //     ROS_ERROR("Failed to plan the motion to the target pose.");
  // }



}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_receiver");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("pose_topic",10, poseCallback);

  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(100); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}

