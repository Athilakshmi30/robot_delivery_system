#ifndef ORDER_MANAGER_H
#define ORDER_MANAGER_H

#include "ros/ros.h"
#include "robot_delivery_system/PopupService.h"
#include "robot_delivery_system/OrderList.h"
#include "robot_delivery_system/robot_movement.h"
#include <vector>
#include <mutex>

class OrderManager {
private:
    std::vector<int> order_list_;  // List to hold incoming orders
    std::vector<int> current_order_list_;  // Orders that are currently being processed
    std::vector<int> cancelled_list_;  // List of cancelled orders
    ros::NodeHandle nh_;
    ros::Subscriber sub_;  // Subscriber to receive order list
    ros::ServiceClient client_;  // Client to communicate with a service for popup confirmations
    bool order_received_ = false;  // Flag indicating if an order has been received
    bool in_home_ = true;  // Flag indicating if the robot is at home
    bool move_home_to_kitchen_ = false;  // Flag to indicate if the robot should move from home to kitchen

    RobotMovement robot_;  // Instance of RobotMovement class to control robot's movement
    bool return_to_kit_for_food_collection_ = false, return_to_kit_for_food_return_ = false;
    std::mutex mutex_;  // Mutex to protect shared resources across multiple threads

public:
    // Constructor to initialize ROS components and subscribe to topics
    OrderManager();

    // Callback function to handle incoming order list messages
    void order_list_cb(const robot_delivery_system::OrderListConstPtr &msg);

    // Check the robot's status and decide the next action
    void check_robot_status();

    // Get confirmation from the robot's popup service to proceed with order
    bool get_confirmation();

    // Process the orders in the current order list
    void process_orders();

    // Check and remove cancelled orders before receiving food
    void check_order_before_receiving_food();

    // Make decisions based on the current state and order status
    void decision_maker();

    // Move the robot to the kitchen
    void move_to_kitchen();

    // Main spin loop for the class to keep checking for orders and handling robot actions
    void spin_class();
};

#endif  // ORDER_MANAGER_H
