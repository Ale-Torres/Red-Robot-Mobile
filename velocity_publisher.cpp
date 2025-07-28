#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_publisher");
    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("runmotor", 10);

    while (ros::ok()) {
        // Get target velocity from somewhere
        float target_velocity;
	std::cout<< "Enter target velocity: ";
	std::cin >> target_velocity;

        std_msgs::Float32 msg;
        msg.data = target_velocity;

        vel_pub.publish(msg);

        ros::spinOnce();
        // Sleep or loop rate control
    }

    return 0;
}
