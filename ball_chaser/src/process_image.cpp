#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO("Attempting request to drive the robot...");

    //create a drive request
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // call the service and check for success
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot.");
        return;
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int side = 0;
    // sides will be:
    // 0 - missing
    // 1 - left
    // 2 - right
    // 3 - middle

    // Determine if there is a white ball and if yes, on which side
    for (int i = 0; i < img.height * img.step; i += 3){
        uint8_t red = img.data[i];
        uint8_t green = img.data[i + 1];
        uint8_t blue = img.data[i + 2];

        if (red == 255 && green == 255 && blue == 255) {
            auto pos = i % img.step;
            if (pos < img.step * 0.4) {
                side = 1;
            }
            else if (pos > img.step * 0.6) {
                side = 2;
            }
            else {
                side = 3;
            }
            break;
        }
    }

    // Drive robot in direction of white ball
    if (side == 1) {
        drive_robot(0.5, 1.0);
    }
    else if (side == 2) {
        drive_robot(0.5, -1.0);
    }
    else if (side == 3) {
        drive_robot(0.5, 0.0);
    }
    else { // No ball detected
        drive_robot(0.0, 0.0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}