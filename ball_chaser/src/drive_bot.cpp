#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
{
    //Set linear x and angular z velocities
    auto linear_x = (float)req.linear_x;
    auto angular_z = (float)req.angular_z;

    // Show output in ROS_INFO
    ROS_INFO("Drive to target requested: linear_x: %1.3f, angular_z: %1.3f", linear_x, angular_z);

    // Create a motor command message: Type will be geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities and yaw orientation
    motor_command.linear.x = linear_x;
    motor_command.angular.z = angular_z;

    // PUB
    motor_command_publisher.publish(motor_command);

    // Return a response message to ROS
    ROS_INFO("Motor command published: linear_x: %1.3f, angular_z: %1.3f", motor_command.linear.x, motor_command.angular.z);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // Send message to ROS INFO
    ROS_INFO("Robot ready to handle drive requests!");

    // DELETED LOOP, Now SPIN
    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}

