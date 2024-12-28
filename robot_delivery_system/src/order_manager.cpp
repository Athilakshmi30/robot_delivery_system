#include "robot_delivery_system/order_manager.h"

OrderManager::OrderManager() : robot_() {
    // Initialize the subscriber for the order list topic
    sub_ = nh_.subscribe("order_list", 10, &OrderManager::order_list_cb, this);

    // Initialize the service client to communicate with the popup service
    client_ = nh_.serviceClient<robot_delivery_system::PopupService>("popup_service");

    // Set the initial robot state parameter to an empty string
    ros::param::set("ROBOT_STATE", "");
}

void OrderManager::order_list_cb(const robot_delivery_system::OrderListConstPtr &msg) {
    if (msg->list != 0) {
        order_list_.push_back(msg->list);

    }
    if (msg->cancelled_list != 0) {
        cancelled_list_.push_back(msg->cancelled_list);

    }
    order_received_ = !order_list_.empty();
}

// The check_robot_status function is called when a new order is received.
// It checks if the robot is at home. If yes, it sets the move_home_to_kitchen_ flag to true,
// allowing the robot to move to the kitchen.
void OrderManager::check_robot_status() {
    if (in_home_) {
        in_home_ = false;
        move_home_to_kitchen_ = true;
        move_to_kitchen();
    }
}

// The get_confirmation function calls the popup server to confirm an action.
bool OrderManager::get_confirmation() {
    std::string robot_state;
    ros::param::get("ROBOT_STATE", robot_state);
    if (robot_state == "WAITING_FOR_FOOD" && order_list_.empty() && current_order_list_.empty()) {

        return false;
    } else {
        robot_delivery_system::PopupService srv;
        if (client_.call(srv)) {
            ROS_INFO("Popup confirmation succeeded.");
            return srv.response.result;
        } else {
            ROS_WARN("Popup confirmation failed.");
            return false;
        }
    }
}

// The process_orders function manages the robot's actions between the kitchen and the tables.
void OrderManager::process_orders() {
    // This loop processes orders for which the robot has collected food.
    while (!current_order_list_.empty()) {
        int current_order = current_order_list_.front();
        current_order_list_.erase(current_order_list_.begin());
        ROS_INFO("Processing order %d", current_order);

        // Before delivering the order to the table, it checks if the order has been canceled.
        // If canceled, the robot skips the order and continues processing the next order.
        if (std::find(cancelled_list_.begin(), cancelled_list_.end(), current_order) != cancelled_list_.end()) {
            cancelled_list_.erase(std::remove(cancelled_list_.begin(), cancelled_list_.end(), current_order), cancelled_list_.end());
            return_to_kit_for_food_return_ = true;
            continue;
        }

        if (robot_.move_to_goal(std::to_string(current_order)) && get_confirmation()) {
            ROS_INFO("Order %d processed successfully.", current_order);
        } else {
            ROS_WARN("Order %d failed. Returning to kitchen.", current_order);
            return_to_kit_for_food_return_ = true;
            continue;
        }
    }

    // If all current orders are processed and new orders exist, the robot will return to the kitchen.
    if (current_order_list_.empty() && !order_list_.empty()) {
        return_to_kit_for_food_collection_ = true;
    }
}

// The check_order_before_receiving_food function ensures that no orders have been canceled
// after the robot reaches the kitchen. If canceled, those orders are removed.
void OrderManager::check_order_before_receiving_food() {
    std::string robot_state;
    ros::param::get("ROBOT_STATE", robot_state);

    if (robot_state == "WAITING_FOR_FOOD") {
        for (auto it = order_list_.begin(); it != order_list_.end();) {
            if (std::find(cancelled_list_.begin(), cancelled_list_.end(), *it) != cancelled_list_.end()) {
                cancelled_list_.erase(std::remove(cancelled_list_.begin(), cancelled_list_.end(), *it), cancelled_list_.end());
                it = order_list_.erase(it);
            } else {
                ++it;
            }
        }
    }
}

// The decision_maker function is called after the robot reaches the kitchen.
// It handles order processing and robot actions based on the current state.
void OrderManager::decision_maker() {
    check_order_before_receiving_food();

    if (get_confirmation()) {
        current_order_list_ = order_list_;  // Copy orders to current_order_list_ after collecting food
        order_list_.clear();
        process_orders();

        if (return_to_kit_for_food_return_ || return_to_kit_for_food_collection_) {
            move_to_kitchen();
        } else if (order_list_.empty()) {
            ROS_INFO("All orders completed. Returning to home.");
            robot_.move_to_goal("home");
            ros::param::set("ROBOT_STATE", "IN_HOME_POSITION");
            in_home_ = true;
        }
    } else {
        // If popup confirmation fails, the robot will return home.
        ROS_WARN("Confirmation failed. Returning to home.");
        order_list_.clear();
        robot_.move_to_goal("home");
        ros::param::set("ROBOT_STATE", "IN_HOME_POSITION");
        in_home_ = true;
    }
}

// The move_to_kitchen function moves the robot to the kitchen based on specific conditions.
void OrderManager::move_to_kitchen() {
    if (return_to_kit_for_food_collection_) {
        if (robot_.move_to_goal("kitchen")) {
            return_to_kit_for_food_collection_ = false;
            ros::param::set("ROBOT_STATE", "WAITING_FOR_FOOD");
            decision_maker();
        }
    } else if (return_to_kit_for_food_return_) {
        if (robot_.move_to_goal("kitchen")) {
            return_to_kit_for_food_return_ = false;
            decision_maker();
        }
    } else if (move_home_to_kitchen_) {
        if (robot_.move_to_goal("kitchen")) {
            move_home_to_kitchen_ = false;
            ros::param::set("ROBOT_STATE", "WAITING_FOR_FOOD");
            decision_maker();
        }
    } else {
        ROS_INFO("Already on a task. Skipping kitchen move.");
    }
}

// The spin_class function continuously spins the OrderManager class to process incoming orders.
void OrderManager::spin_class() {
    ros::Rate rate(10);
    while (ros::ok()) {
        // Check if an order has been received
        if (order_received_) {
            order_received_ = false;

            // Ensure there are orders in the list before proceeding
            if (!order_list_.empty()) {
                check_robot_status();
            }
        }
        rate.sleep();
    }
}
