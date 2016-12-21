#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_first_ros_node");
    ROS_INFO("Hello world from node");
    ros::spin();
}
