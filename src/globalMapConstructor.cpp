#include <global_map/globalMapConstructor.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <tf2_eigen/tf2_eigen.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
globalMapConstructor::globalMapConstructor(ros::NodeHandle &n):nh(n)
{

    nh.getParam("/camera/depth/color/points", camera_topic);
    nh.getParam("camera_init", world_frame);
    nh.getParam("body", LIDAR_frame);
    nh.getParam("camera_depth_optical_frame", camera_frame);
    camera_frame = "camera_depth_optical_frame";
    LIDAR_frame = "body";
    world_frame = "camera_init";
    camera_topic = "/camera/depth/color/points";
    sub_camera = nh.subscribe(camera_topic, 1, &globalMapConstructor::callback_camera, this);
    pub_globalcloud = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    odom_camera.setIdentity();
}

void globalMapConstructor::callback_camera(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    // cout<<"camera callback"<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    // 比较合适的leaf size，配准速度合适
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*filtered_cloud);
    // LOG(INFO)<<"point size: "<<filtered_cloud->size();
#ifdef DEBUG
    geometry_msgs::TransformStamped transformStamped_test;
    try
    {
        transformStamped_test = tf_buffer_->lookupTransform(LIDAR_frame, camera_frame, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        std::cerr << ex.what() << '\n';
    }
    tf2::Transform camera_T_lidar;
    tf2::fromMsg(transformStamped_test.transform, camera_T_lidar);
    LOG(INFO)<<"camera_T_lidar: "<<camera_T_lidar.getOrigin().x()<<" "<<camera_T_lidar.getOrigin().y()<<" "<<camera_T_lidar.getOrigin().z()<<" "<<camera_T_lidar.getRotation().x()<<" "<<camera_T_lidar.getRotation().y()<<" "<<camera_T_lidar.getRotation().z()<<" "<<camera_T_lidar.getRotation().w();
#endif
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        LOG(INFO)<<"camera_frame: "<<camera_frame<<" world_frame: "<<world_frame;
        transformStamped = tf_buffer_->lookupTransform(camera_frame, world_frame, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        std::cerr << ex.what() << '\n';
    }
    tf2::Transform T_camera_3dworld;
    LOG(INFO)<<"T_camera_3dworld: "<<transformStamped.transform.translation.x<<" "<<transformStamped.transform.translation.y<<" "<<transformStamped.transform.translation.z;
    LOG(INFO)<<"T_camera_3dworld: "<<transformStamped.transform.rotation.x<<" "<<transformStamped.transform.rotation.y<<" "<<transformStamped.transform.rotation.z<<" "<<transformStamped.transform.rotation.w;
    tf2::fromMsg(transformStamped.transform, T_camera_3dworld);
#ifdef DEBUG
    LOG(INFO)<<"T_camera_3dworld: "<<T_camera_3dworld.getOrigin().x()<<" "<<T_camera_3dworld.getOrigin().y()<<" "<<T_camera_3dworld.getOrigin().z();
#endif
    // 如果是第一次
    if(last_cloud.empty())
    {
        // 将点云转到世界坐标系下
        // tf2::Transform T_world_camera = (T_camera_lidar * T_LIDAR_3dworld).inverse();
        LOG(INFO)<<"T_camera_3dworld: "<<T_camera_3dworld.getOrigin().x()<<" "<<T_camera_3dworld.getOrigin().y()<<" "<<T_camera_3dworld.getOrigin().z();
        LOG(INFO)<<"T_camera_3dworld: "<<T_camera_3dworld.getRotation().x()<<" "<<T_camera_3dworld.getRotation().y()<<" "<<T_camera_3dworld.getRotation().z()<<" "<<T_camera_3dworld.getRotation().w();
        geometry_msgs::Transform geometry_transform = tf2::toMsg(T_camera_3dworld.inverse());
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(geometry_transform);
        pcl::transformPointCloud(*filtered_cloud, global_cloud, eigen_transform);
        odom_last = T_camera_3dworld;
    }
    else
    {
        // 当前帧到前一帧得转换猜测
        tf2::Transform guess =  odom_last * T_camera_3dworld.inverse();
        LOG(INFO)<<"guess:"<<guess.getOrigin().x()<<","<<guess.getOrigin().y()<<","<<guess.getOrigin().z();
        geometry_msgs::Transform geometry_transform = tf2::toMsg(guess);
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(geometry_transform);
        Eigen::Matrix4f initial_guess = eigen_transform.matrix().cast<float>();
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(filtered_cloud);
        gicp.setInputTarget(last_cloud.makeShared());
        gicp.setMaxCorrespondenceDistance(0.05); //设置对应点对之间的最大距离
        gicp.setTransformationEpsilon(1e-2);
        gicp.setEuclideanFitnessEpsilon(0.01);  //设置收敛条件是均方误差和小于阈值， 停止迭代
        gicp.setMaximumIterations(40);
        pcl::PointCloud<pcl::PointXYZ> final;

        auto start = std::chrono::high_resolution_clock::now();
        gicp.align(final, initial_guess);
        auto finish = std::chrono::high_resolution_clock::now();
        std::cout << "ICP took " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << "ms" << std::endl;
        cout<<" 111";
        Eigen::Matrix4f transformation_matrix = gicp.getFinalTransformation();

        // Eigen::Matrix3f rotation_matrix_guess = guess.getBasis().cast<float>();
        Eigen::Quaternionf quaternion_guess_eigen;
        quaternion_guess_eigen.w() = guess.getRotation().w();
        quaternion_guess_eigen.x() = guess.getRotation().x();
        quaternion_guess_eigen.y() = guess.getRotation().y();
        quaternion_guess_eigen.z() = guess.getRotation().z();
        Eigen::Quaternionf quaternion_transformation(transformation_matrix.block<3, 3>(0, 0));

        double angle = quaternion_guess_eigen.angularDistance(quaternion_transformation);
        std::cout << "Angle between quaternions: " << angle << " radians" << std::endl;
        // 当前帧到前一帧的变换矩阵
        std::cout << "Final transformation matrix:\n" << transformation_matrix << std::endl;
#ifdef DEBUG
        pcl::PointCloud<pcl::PointXYZ> total_cloud;
        // total_cloud += last_cloud;

        pcl::transformPointCloud(*filtered_cloud, total_cloud, transformation_matrix);
        total_cloud += last_cloud;
        pcl::io::savePCDFileASCII("/home/lichao/navigation/src/global_map/data/" + std::to_string(debug_index) + ".pcd", total_cloud);
        debug_index++;
#endif
        // Extract translation
        tf2::Vector3 tf_translation(
            transformation_matrix(0, 3), 
            transformation_matrix(1, 3), 
            transformation_matrix(2, 3)
        );

        // Extract rotation
        Eigen::Matrix3f rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
        tf2::Matrix3x3 tf_rotation(
            rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
            rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
            rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
        );
        // Combine translation and rotation into a tf2::Transform
        tf2::Transform tf_transform(tf_rotation, tf_translation);
        // odom_camera =  odom_camera * tf_transform;
        odom_last = (odom_last.inverse() * tf_transform).inverse();
        

        geometry_msgs::Transform transform_world_camera = tf2::toMsg(odom_last.inverse());
        Eigen::Affine3d T_world_camera = tf2::transformToEigen(transform_world_camera);
        pcl::PointCloud<pcl::PointXYZ> trans_cloud;
        pcl::transformPointCloud(*filtered_cloud, trans_cloud, T_world_camera);
        global_cloud += trans_cloud;
    }
    
    last_cloud = *filtered_cloud;
    // odom_last = odom_camera;

    sensor_msgs::PointCloud2 global_cloud_msg;
    pcl::toROSMsg(global_cloud, global_cloud_msg);
    global_cloud_msg.header.frame_id = "camera_init";
    pub_globalcloud.publish(global_cloud_msg);
}

globalMapConstructor::~globalMapConstructor()
{
    pcl::io::savePCDFileASCII("/home/lichao/navigation/src/global_map/data/globalmap.pcd", global_cloud);
}