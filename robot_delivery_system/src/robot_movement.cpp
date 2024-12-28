#include "robot_delivery_system/robot_movement.h"

RobotMovement::RobotMovement() : act_client_("move_base", true) {
    // Wait for the move_base action server to start
    ROS_INFO("Waiting for move_base action server...");
    act_client_.waitForServer();
    ROS_INFO("Connected to move_base server.");
}

bool RobotMovement::move_to_goal(const std::string& goal_name) {
    move_base_msgs::MoveBaseGoal goal;
    // Get the position of the goal based on the provided name
    geometry_msgs::Pose position = get_position(goal_name);

    // Set the target pose's frame and timestamp
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp  = ros::Time::now();
    goal.target_pose.pose = position;

    ROS_INFO("Sending goal to move_base...");
    // Update the robot state to indicate it's moving
    ros::param::set("ROBOT_STATE", "MOVING TO GOAL...");
    act_client_.sendGoal(goal);
    act_client_.waitForResult();

    // Check the result of the move action
    if (act_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Robot reached goal position");
        return true;
    } else {
        ROS_INFO("Failed to reach goal");
        return false;
    }
}

geometry_msgs::Pose RobotMovement::get_position(const std::string& position_name) {
    geometry_msgs::Pose pose;
    if (position_name == "kitchen") {
        set_position(pose, -2.0, -0.7, 1.572);
    } else if (position_name == "home") {
        set_position(pose, -4.0, -2.0, 0.0);
    } else if (position_name == "1") {
        set_position(pose, 1.5, 0.9, 1.572);
    } else if (position_name == "2") {
        set_position(pose, 2.5, -1.0, 0.0);
    } else if (position_name == "3") {
        set_position(pose, 1.0, -2.8, -1.572);
    }
    return pose;
}

void RobotMovement::set_position(geometry_msgs::Pose &pose, double x, double y, double yaw) {
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(yaw);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
}
