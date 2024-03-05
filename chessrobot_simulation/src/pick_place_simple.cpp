#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

const double unit_square = 0.0375;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_simple");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
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
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

  
    // 2. Open the gripper
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    moveit::core::RobotStatePtr current_state = move_group_interface_gripper.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    // joint_group_positions[0] = 0.593; 
    joint_group_positions[0] = 0.55;  
    move_group_interface_gripper.setJointValueTarget(joint_group_positions);

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();

  
  
  
    // 3. Move TCP to top of e2
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("tool0");
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose target_pose1;

    // target_pose1.orientation = current_pose.pose.orientation;
    target_pose.orientation.x = 0.00212;
    target_pose.orientation.y = 0.99998;
    target_pose.orientation.z = -0.0059781;
    target_pose.orientation.w = 0.000400;
    target_pose.position.x = 0.020;
    target_pose.position.y = 0.4185 ;
    target_pose.position.z = 0.28;

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // 6. Move the TCP above the pawn
    target_pose.position.z = target_pose.position.z - 0.1;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // 2. close the gripper
    current_state = move_group_interface_gripper.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    // joint_group_positions[0] = 0.6981317;
    joint_group_positions[0] = 0.733;   
    move_group_interface_gripper.setJointValueTarget(joint_group_positions);

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();


    // Add a delay
    ros::Duration(2.0).sleep(); // Sleep for 2 seconds (adjust as needed)


    // 6. Move the TCP above the e4
    target_pose.position.z = target_pose.position.z + 0.1;
    target_pose.position.y = target_pose.position.y + 2*unit_square;    
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // 6. Move the TCP to e4
    target_pose.position.z = target_pose.position.z - 0.09;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();


    // 2. Open the gripper
    current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    // joint_group_positions[0] = 0.593; 
    joint_group_positions[0] = 0.55;  
    move_group_interface_gripper.setJointValueTarget(joint_group_positions);

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_gripper.move();


    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

   // move_group_interface_arm.move();

  ros::shutdown();
  return 0;
}

    // // 1. Move to home position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();


    // // 4. Move the TCP close to the object
    // target_pose1.orientation = target_pose.orientation;
    // target_pose1.position.x = target_pose.position.x + 3*unit_square;
    // target_pose1.position.y = target_pose.position.y - 1*unit_square;
    // target_pose1.position.z = target_pose.position.z;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();


    // // 2. Move to home position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();



    // // 4. Move the TCP close to the object
    // target_pose1.orientation = target_pose.orientation;
    // target_pose1.position.x = target_pose.position.x - 4*unit_square;
    // target_pose1.position.y = target_pose.position.y - 1*unit_square;
    // target_pose1.position.z = target_pose.position.z;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();


    // // 3. Move to home position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();



    // // 4. Move the TCP close to the object
    // target_pose1.orientation = target_pose.orientation;
    // target_pose1.position.x = target_pose.position.x + 3*unit_square;
    // target_pose1.position.y = target_pose.position.y + 6*unit_square;
    // target_pose1.position.z = target_pose.position.z;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();


    // // 1. Move to home position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();



    // // 4. Move the TCP close to the object
    // target_pose1.orientation = target_pose.orientation;
    // target_pose1.position.x = target_pose.position.x - 4*unit_square;
    // target_pose1.position.y = target_pose.position.y + 6*unit_square;
    // target_pose1.position.z = target_pose.position.z;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // // 5. Close the  gripper
    // move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));

    // success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_gripper.move();

    // // 6. Move the TCP above the plate
    // target_pose1.position.z = target_pose1.position.z + 0.2;
    // target_pose1.position.x = target_pose1.position.x - 0.6;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // // 7. Lower the TCP above the plate
    // target_pose1.position.z = target_pose1.position.z - 0.14;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // // 8. Open the gripper
    // move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    // success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_gripper.move();
