#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include <tf/tf.h>

class RobotMovement {
public:
    // Action client for moving the robot to a specific goal position
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> act_client_;

    // Constructor initializes the action client for move_base
    RobotMovement();

    // Move the robot to a specified goal by its name
    bool move_to_goal(const std::string& goal_name);

private:
    // Helper function to get the position of a goal based on its name
    geometry_msgs::Pose get_position(const std::string& position_name);

    // Helper function to set the position for a given pose object
    void set_position(geometry_msgs::Pose &pose, double x, double y, double yaw);
};

#endif  // ROBOT_MOVEMENT_H
