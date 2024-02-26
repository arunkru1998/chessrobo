#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangeably.
    static const std::string PLANNING_GROUP_ARM = "manipulator";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group_arm =
        move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

    const moveit::core::JointModelGroup* joint_model_group_gripper =
        move_group_interface_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface_arm.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface_arm.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
                move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // .. _move_group_interface-planning-to-pose-goal:
    //
    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    // geometry_msgs::Pose target_pose1;
    // target_pose1.orientation.w = 1.0;
    // target_pose1.position.x = -0.48;
    // target_pose1.position.y = 0.107;
    // target_pose1.position.z = 0.538;
    // move_group_interface_arm.setPoseTarget(target_pose1);

    // // Now, we call the planner to compute the plan and visualize it.
    // // Note that we are just planning, not asking move_group_interface
    // // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // bool success = (move_group_interface_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // // Visualizing plans
    // // ^^^^^^^^^^^^^^^^^
    // // We can also visualize the plan as a line with markers in RViz.
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    // visual_tools.publishAxisLabeled(target_pose1, "pose1");
    // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
    // Note that this can lead to problems if the robot moved in the meanwhile.
    // move_group_interface.execute(my_plan);

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // If you do not want to inspect the planned trajectory,
    // the following is a more robust combination of the two-step plan+execute pattern shown above
    // and should be preferred. Note that the pose goal we had set earlier is still active,
    // so the robot will try to move to that goal.

    // move_group_interface.move();

    // Planning to a joint-space goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Let's set a joint space goal and move towards it.  This will replace the
    // pose target we set above.
    //
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group_interface_arm.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_arm, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = -tau / 2;  // -1/6 turn in radians
    move_group_interface_arm.setJointValueTarget(joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group_interface_arm.setMaxVelocityScalingFactor(0.05);
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.05);
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_arm);
    visual_tools.trigger();
    move_group_interface_arm.execute(my_plan);
    move_group_interface_arm.move();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    // 3. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();
  ros::shutdown();
  return 0;
}
