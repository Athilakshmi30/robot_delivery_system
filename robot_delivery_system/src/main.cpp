#include "ros/ros.h"
#include "robot_delivery_system/order_manager.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "order_manager_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    OrderManager manager;
    manager.spin_class();

    return 0;
}
