#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <chess_robot_service/motion_planning.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <locale> // for std::tolower
#include <cmath>  // for M_PI constant



// Function to convert a string to lowercase
std::string toLower(const std::string& str) {
    std::string result;
    for (char c : str) {
        result += std::tolower(c);
    }
    return result;
}

// Function to convert degrees to radians
double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}


// Function to find a joint angle from a dictionary
double findJointAngle(const std::string& key) {
    // Dictionary mapping strings to double values
    std::unordered_map<std::string, double> dictionary = {
        {"k", 40.50},
        {"q", 40.50},
        {"b", 41.00},
        {"r", 41.00},
        {"n", 41.00},
        {"p", 42.00}
        // Add more entries as needed
    };

    // Convert the key to lowercase
    std::string lowerKey = toLower(key);

    // Check if the key exists in the dictionary
    auto it = dictionary.find(lowerKey);
    if (it != dictionary.end()) {
        // Convert the found value from degrees to radians and return
        std::cout << it->second << std::endl;
        return degreesToRadians(it->second);
    } else {
        // If key not found, return some default value or throw an exception
        // Here, I'm returning -1.0 as a default value
        return -1.0; // Or throw an exception: throw std::runtime_error("Key not found");
    }
}


double find_z_offset(const std::string& key) {
    // Dictionary mapping strings to double values
    std::unordered_map<std::string, double> dictionary = {
        {"k", 0.035},
        {"q", 0.035},
        {"b", 0.035},
        {"r", 0.035},
        {"n", 0.035},
        {"p", 0.04}
        // Add more entries as needed
    };

    // Convert the key to lowercase
    std::string lowerKey = toLower(key);

    // Check if the key exists in the dictionary
    auto it = dictionary.find(lowerKey);
    if (it != dictionary.end()) {
        // Convert the found value from degrees to radians and return
        std::cout << it->second << std::endl;
        return it->second;
    } else {
        // If key not found, return some default value or throw an exception
        // Here, I'm returning -1.0 as a default value
        return -1.0; // Or throw an exception: throw std::runtime_error("Key not found");
    }
}


bool closeGripper(moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper, const moveit::core::JointModelGroup*& joint_model_group_gripper, const std::string& str)
{
  std::vector<double> gripper_joint_values;
    // 2. close the gripper
  moveit::core::RobotStatePtr current_state = move_group_interface_gripper.getCurrentState();

  std::vector<double> joint_group_positions;
  // Next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = 0.6981317;
  joint_group_positions[0] = findJointAngle(str);   
  move_group_interface_gripper.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

  bool success = (move_group_interface_gripper.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
      // Execute the planned motion to close the gripper
      move_group_interface_gripper.execute(gripper_plan);
      ROS_INFO("Gripper closed successfully.");
      ros::Duration(2.0).sleep(); // Sleep for 2 seconds (adjust as needed)
  }
  else
  {
      ROS_ERROR("Failed to close the gripper.");
  }
  return success;
}


bool openGripper(moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper, const moveit::core::JointModelGroup*& joint_model_group_gripper)
{
  std::vector<double> gripper_joint_values;
    // 2. open the gripper
  moveit::core::RobotStatePtr current_state = move_group_interface_gripper.getCurrentState();

  std::vector<double> joint_group_positions;
  // Next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = 0.6981317;
  joint_group_positions[0] = 0.53;   
  move_group_interface_gripper.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

  bool success = (move_group_interface_gripper.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
      // Execute the planned motion to close the gripper
      move_group_interface_gripper.execute(gripper_plan);
      ROS_INFO("Gripper opened successfully.");
  }
  else
  {
      ROS_ERROR("Failed to open the gripper.");
  }
  return success;
}

bool pick(moveit::planning_interface::MoveGroupInterface& move_group_interface_arm,const geometry_msgs::Pose& target_pose,moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper, const moveit::core::JointModelGroup*& joint_model_group_gripper, const std::string& str)
{
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::RobotTrajectory trajectory_2;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<geometry_msgs::Pose> waypoints_2;
  waypoints.push_back(target_pose);
  waypoints_2.push_back(target_pose);
  geometry_msgs::Pose z_offset_pose=target_pose;
  double offset=find_z_offset(str);
  z_offset_pose.position.z-=offset;
  waypoints.push_back(z_offset_pose);

  bool success = (move_group_interface_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
      // Execute the motion
      // move_group_interface_arm.move();
      move_group_interface_arm.execute(trajectory);
      closeGripper(move_group_interface_gripper,joint_model_group_gripper, str);
      move_group_interface_arm.computeCartesianPath(waypoints_2, eef_step, jump_threshold, trajectory_2);
      move_group_interface_arm.execute(trajectory_2);
      ROS_INFO("Robot moved to the target pose successfully.");
  }
  else
  {
      ROS_ERROR("Failed to plan the motion to the target pose.");
  }
  return success;
}

bool place(moveit::planning_interface::MoveGroupInterface& move_group_interface_arm,const geometry_msgs::Pose& target_pose,moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper, const moveit::core::JointModelGroup*& joint_model_group_gripper, const std::string& str)
{
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::RobotTrajectory trajectory_2;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<geometry_msgs::Pose> waypoints_2;
  waypoints.push_back(target_pose);
  waypoints_2.push_back(target_pose);
  geometry_msgs::Pose z_offset_pose=target_pose;
  double offset=find_z_offset(str);
  z_offset_pose.position.z-=offset;
  waypoints.push_back(z_offset_pose);

  bool success = (move_group_interface_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
      // Execute the motion
      // move_group_interface_arm.move();
      move_group_interface_arm.execute(trajectory);
      openGripper(move_group_interface_gripper,joint_model_group_gripper);
      move_group_interface_arm.computeCartesianPath(waypoints_2, eef_step, jump_threshold, trajectory_2);
      move_group_interface_arm.execute(trajectory_2);
      ROS_INFO("Robot moved to the target pose successfully.");
  }
  else
  {
      ROS_ERROR("Failed to plan the motion to the target pose.");
  }
  return success;
}




bool poseCallback(chess_robot_service::motion_planning::Request  &req, chess_robot_service::motion_planning::Response &res)
{
  //access the pose array portion of the service
  const geometry_msgs::PoseArray& poses = req.request;
  std::string move_command = req.command;
  std::vector<std::string> command_sub_parts;
  std::istringstream iss(move_command);
  std::string part;
  std::string chess_piece;
  std::string capturing;
  std::string castling; 

  // Splitting the move_command string by comma and storing the parts in a vector
  while (std::getline(iss, part, ',')) {
      command_sub_parts.push_back(part);
  }

  // Assuming at least 3 parts are present
  if (command_sub_parts.size() >= 3) {
      chess_piece = command_sub_parts[0];
      capturing = command_sub_parts[1];
      castling = command_sub_parts[2];
  }
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
  bool success=false;
  
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
  openGripper(move_group_interface_gripper,joint_model_group_gripper);
  int count=0;


  // Set the pose target from the request
  for (const auto& pose : poses.poses)
  {
    if(count==0)
    {
      success = pick(move_group_interface_arm, pose,move_group_interface_gripper,joint_model_group_gripper,chess_piece);

      if (success)
      {
          // Execute the motion
          // move_group_interface_arm.move();
          // move_group_interface_arm.execute(trajectory);
          ROS_INFO("Robot picked the chess piece successfully.");
          // closeGripper(move_group_interface_gripper,joint_model_group_gripper,chess_piece);
      }
      else
      {
          ROS_ERROR("Robot didnot pick the chess piece successfully.");
      }
      // waypoints.pop_back();

    }

    if(count==1)
   {
      success = place(move_group_interface_arm, pose,move_group_interface_gripper,joint_model_group_gripper,chess_piece);

      if (success)
      {
          // Execute the motion
          // move_group_interface_arm.move();
          // move_group_interface_arm.execute(trajectory);
          ROS_INFO("Robot placed the chess piece successfully.");
          // closeGripper(move_group_interface_gripper,joint_model_group_gripper,chess_piece);
      }
      else
      {
          ROS_ERROR("Robot didnot place the chess piece successfully.");
      }
    // waypoints.pop_back();
 

   }  
    // waypoints.push_back(pose);
    // waypoints.push_back(pose.position.x-0.1);
    
    // move_group_interface_arm.setPoseTarget(pose);
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // success = (move_group_interface_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    count++;
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

