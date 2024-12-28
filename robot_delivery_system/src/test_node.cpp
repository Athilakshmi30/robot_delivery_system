#include "ros/ros.h"
#include "robot_delivery_system/OrderList.h"


using namespace std;



int order_list_input;
int cancel_list_input;
int table_number;
string action;
bool start_pub = false;

void manage_user_input(int table_number, string action) {
    start_pub = true;
    if (action == "order" || action == "ORDER") {
        order_list_input = table_number;
    } else if (action == "cancel" || action == "CANCEL") {

        cancel_list_input = table_number;
    }
}



int main(int argc,char **argv){
  ros::init(argc,argv,"test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<robot_delivery_system::OrderList>("order_list",10);



  while(ros::ok()){
      cout << "Please enter the table number to place or cancel the order:" << endl;
      cout << "Available options: 1, 2, 3" << endl;
      cout << "1 = Table 1" << endl;
      cout << "2 = Table 2" << endl;
      cout << "3 = Table 3" << endl;
      std::cout << "    " << '\n';
      cout << "Table number: ";
      cin >> table_number;
      std::cout << "    " << '\n';
      cout << "Please choose an action:" << endl;
      cout << "Available options: order, cancel" << '\n';
      cout << "'order' = To place an order request" << '\n';
      cout << "'cancel' = To cancel the order request" << '\n';
      cout << "Desired action: ";
      cin >> action;
      std::cout << "    " << '\n';
      manage_user_input(table_number,action);

      // Construct and publish the message
        robot_delivery_system::OrderList msg;
        msg.list = order_list_input; // Assign the vector to the message field
        msg.cancelled_list = cancel_list_input;
        order_list_input = 0;
        cancel_list_input = 0;

        pub.publish(msg);

  }

  ros::spin();

}
