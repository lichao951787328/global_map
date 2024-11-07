#include <global_map/globalMapConstructor.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_map_node");
    ros::NodeHandle nh;
    globalMapConstructor global_map_constructor(nh);
    ros::spin();
    return 0;
}