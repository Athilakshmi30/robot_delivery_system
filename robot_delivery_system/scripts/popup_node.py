#!/usr/bin/env python3
import rospy
from robot_delivery_system.srv import PopupService, PopupServiceResponse
import tkinter as tk

# Global variable to track rosservice status
rosservice_response = False

# Function to handle the pop-up and user input
def show_popup():
    global rosservice_response
    rosservice_response = False  # Start with False

    # Set up the Tkinter root window
    root = tk.Tk()
    root.withdraw()  # Hide the root window, no need to show it

    # Create a new top-level window for the pop-up
    popup = tk.Toplevel(root)
    popup.title("Confirmation")

    # Set the size of the pop-up window
    popup.geometry("400x200")  # Width = 400px, Height = 200px

    # Label for the message
    label = tk.Label(popup, text="Click OK to proceed.")
    label.pack(pady=20)

    # Function to handle the OK button click
    def on_ok_click():
        global rosservice_response
        rosservice_response = True
        print("rosservice set to True")
        popup.quit()  # Close the pop-up window
        root.quit()   # Close the root window

    # OK button
    ok_button = tk.Button(popup, text="OK", command=on_ok_click)
    ok_button.pack(pady=10)

    # Timeout function after 30 seconds
    def timeout():
        global rosservice_response
        if not rosservice_response:  # If no response yet, set to False after timeout
            rosservice_response = False
            print("rosservice set to False after 30 seconds timeout")
            popup.quit()  # Close the pop-up after timeout
            root.quit()   # Close the root window after timeout

    # Set timeout to 30 seconds
    root.after(10000, timeout)  # Timeout function after 30 seconds

    # Start Tkinter event loop (this will block until user interaction)
    popup.mainloop()

    # Return the service response after pop-up is closed
    return rosservice_response

# ROS Service callback function
def handle_popup_request(req):
    rospy.loginfo("Service called. Showing pop-up.")

    # Call the pop-up function to show the window
    result = show_popup()

    # After the pop-up, return the service response
    return PopupServiceResponse(result)

def popup_server():
    # Initialize the ROS node
    rospy.init_node('popup_service_server')

    # Advertise the service
    rospy.Service('popup_service', PopupService, handle_popup_request)

    rospy.loginfo("Ready to show pop-ups and return service responses.")
    rospy.spin()

if __name__ == "__main__":
    popup_server()
