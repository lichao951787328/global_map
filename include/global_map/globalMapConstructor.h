#pragma once
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
// 直接使用tf便可以获取当前时刻激光雷达的位姿，结合上一时刻向机想对于世界坐标系的位姿和激光雷达相对于相机的位姿，即可得到当前时刻激光雷达相对于世界坐标系的位姿。得到配准初始值
using namespace std;
// #define DEBUG
class globalMapConstructor
{
private:
    ros::NodeHandle nh;
    tf2::Transform odom_camera;
    tf2::Transform odom_last;
    string camera_topic;
    string world_frame;
    string LIDAR_frame;
    string camera_frame;
    ros::Subscriber sub_camera;
    ros::Publisher pub_globalcloud;
    pcl::PointCloud<pcl::PointXYZ> last_cloud;
    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2::Transform T_lidar_camera;
#ifdef DEBUG
    int debug_index = 0;
#endif
public:
    globalMapConstructor(ros::NodeHandle &n);
    void callback_camera(const sensor_msgs::PointCloud2::ConstPtr msg);
    ~globalMapConstructor();
};



