#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Char.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "direction_publisher");
    ros::NodeHandle nh;

    ros::Publisher dir_pub = nh.advertise<std_msgs::Float32>("controlmotor", 10);

    while (ros::ok()) {
        // Get direction from somewhere       
	char ch;
	std::cout<< "Enter direction (W/A/S/D): ";
	std::cin >> ch;

	std_msgs::Char choice;
        choice.data = ch;

	dir_pub.publish(choice);

        ros::spinOnce();
        // Sleep or loop rate control
    }

    return 0;
}
