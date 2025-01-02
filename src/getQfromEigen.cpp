#include <iostream>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

int main() {
    // 相机到相机安装孔的位姿
    tf2::Transform camera_T_cameraInstall = tf2::Transform::getIdentity();
    tf2::Vector3 camera_t_cameraInstall(0, -0.001, 0.0215);
    camera_T_cameraInstall.setOrigin(camera_t_cameraInstall);

    // 相机安装孔到激光雷达安装孔的位姿
    tf2::Transform cameraInstall_T_lidarInstall = tf2::Transform::getIdentity();
    tf2::Vector3 cameraInstall_t_lidarInstall(0.11211, 0, 0.0175);
    Eigen::Matrix3d cameraInstall_R_lidarInstall;
    cameraInstall_R_lidarInstall << 0, -0.866, 0.5, -1, 0, 0, 0, -0.5, -0.866;

    Eigen::Quaterniond QD(cameraInstall_R_lidarInstall);
    tf2::Quaternion cameraInstall_Q_lidarInstall;
    cameraInstall_Q_lidarInstall.setW(QD.w());
    cameraInstall_Q_lidarInstall.setX(QD.x());
    cameraInstall_Q_lidarInstall.setY(QD.y());
    cameraInstall_Q_lidarInstall.setZ(QD.z());
    cameraInstall_T_lidarInstall.setOrigin(cameraInstall_t_lidarInstall);
    cameraInstall_T_lidarInstall.setRotation(cameraInstall_Q_lidarInstall);

    // 激光雷达安装孔到激光雷达点云
    tf2::Transform lidar_T_lidarInstall = tf2::Transform::getIdentity();
    tf2::Vector3 lidar_t_lidarInstall(0, 0, 0.047);
    lidar_T_lidarInstall.setOrigin(lidar_t_lidarInstall);

    tf2::Transform camera_T_lidar = camera_T_cameraInstall * cameraInstall_T_lidarInstall * lidar_T_lidarInstall.inverse();
    tf2::Quaternion camera_T_lidar_quat = camera_T_lidar.getRotation();
    tf2::Vector3 camera_T_lidar_trans = camera_T_lidar.getOrigin();

    std::cout << "Quaternion: (" 
              << camera_T_lidar_quat.x() << ", " 
              << camera_T_lidar_quat.y() << ", " 
              << camera_T_lidar_quat.z() << ", " 
              << camera_T_lidar_quat.w() << ")" << std::endl;

    std::cout << "Translation: (" 
              << camera_T_lidar_trans.x() << ", " 
              << camera_T_lidar_trans.y() << ", " 
              << camera_T_lidar_trans.z() << ")" << std::endl;

    return 0;
}