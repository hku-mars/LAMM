#include "std.h"
#include "std_ba.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h> // 包含用于读取PCD文件的头文件
#include "DynObjFilter.h"

#define BACKWARD_HAS_DW 1
#define debug

#include "backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}

// dynamic points
string out_file = "";
string out_file_origin = "";
string steady_save = "/home/weihairuo/bag/sz_north/steady/";
double lidar_end_time = 0;

void use_m_detector(shared_ptr<DynObjFilter> &DynObjFilt,
                    pcl::PointCloud<pcl::PointXYZI> &pcl_cloud, int &my_index, int &data_id,
                    std::vector<double> &time_list, std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &pose_list,
                    std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>> pose_list_array,
                    ConfigSetting &config_setting, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &steady_pcd,
                    std::vector<pcl::PointCloud<pcl::PointXYZINormal>> &result_clouds, int &m_th,
                    ros::Publisher &pubOdomAftMapped, ros::Publisher &pubRegisterCloud,
                    ros::Publisher &pubLaserCloudEffect, ros::Publisher &pubLaserCloudEffect_depth,
                    std::ofstream &debug_file, ros::Rate &loop) {

    int pose_index = my_index - 1;
    std::cout << " pose_index " << my_index << std::endl;
    // time starts from 0
    if (m_th == 0) {
        lidar_end_time = (time_list[pose_index] - time_list[0]) * 0.000000001;
    } else {
        lidar_end_time =
                (time_list[pose_list_array[data_id].size() - 1] - (time_list[pose_index] - time_list[0])) * 0.000000001;
    }

    // load odom through pose list load from txt
    debug_file << pose_index << "lidar time "
               << lidar_end_time << std::endl;
    Eigen::Vector3d translation = pose_list[pose_index].first;
    Eigen::Matrix3d rotation = pose_list[pose_index].second;
    Eigen::Quaterniond q(rotation);

    // publish odom
    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.pose.pose.position.x = translation[0];
    odom.pose.pose.position.y = translation[1];
    odom.pose.pose.position.z = translation[2];
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    pubOdomAftMapped.publish(odom);
    loop.sleep();

    debug_file << pose_index << "lidar time "
               << lidar_end_time << " x "
               << odom.pose.pose.position.x << " y "
               << odom.pose.pose.position.y << " z "
               << odom.pose.pose.position.z << " x "
               << odom.pose.pose.orientation.x << " y "
               << odom.pose.pose.orientation.y << " z "
               << odom.pose.pose.orientation.z << " w "
               << odom.pose.pose.orientation.w << std::endl;

    // cloud used to use M-Detector, in lidar frame
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_for_m(new pcl::PointCloud<pcl::PointXYZINormal>);


    // pcl_cloud is in imu frame only the first time to use M-Detector
    // turn into lidar frame to get dynamic_cloud (body frame)
    // turn into XYZINormal to use M-Detector
    for (size_t i = 0; i < pcl_cloud.size(); i++) {
        Eigen::Vector3d pv(pcl_cloud.points[i].x, pcl_cloud.points[i].y,
                           pcl_cloud.points[i].z);

        /// need to change pcl_cloud to lidar frame from imu frame only the first time to use M-Detector
        if (m_th == 0) {
            pv = config_setting.rot_lidar_to_vehicle_ * pv +
                 config_setting.t_lidar_to_vehicle_;
//            pv = rotation.transpose() * (pv - translation);
        }
        pcl::PointXYZINormal pi1;
        pi1.x = pv[0];
        pi1.y = pv[1];
        pi1.z = pv[2];
        cloud_for_m->push_back(pi1);
    }

    /// no need to down sample
    down_sampling_voxel(*cloud_for_m, 0.5);

    /// detect and save steady points
    if (m_th == 0) {
        /// need rotation and translation to transform cloud_for_m to world frame
        DynObjFilt->filter(cloud_for_m, rotation, translation, lidar_end_time);

        /// save steady points in body frame for the second M-Detector
        Eigen::Matrix3d rotation_t = rotation.transpose();
        Eigen::Matrix3d rot_lidar_to_vehicle_inv = config_setting.rot_lidar_to_vehicle_.transpose();
        steady_pcd->clear();
        DynObjFilt->publish_dyn(steady_pcd, rotation_t, rot_lidar_to_vehicle_inv, translation,
                                config_setting.t_lidar_to_vehicle_, pubLaserCloudEffect, pubLaserCloudEffect_depth,
                                lidar_end_time);
    } else {
        DynObjFilt->filter(cloud_for_m, rotation, translation, lidar_end_time);
        steady_pcd->clear();
        DynObjFilt->publish_dyn(steady_pcd, pubLaserCloudEffect, pubLaserCloudEffect_depth, lidar_end_time);
    }

    /// pub and save steady points in world frame as the result of M-Detector
    PointCloudXYZI::Ptr pub_steady_pcd(new pcl::PointCloud<pcl::PointXYZINormal>);
    for (size_t i = 0; i < steady_pcd->points.size(); i++) {
        Eigen::Vector3d body_p(steady_pcd->points[i].x, steady_pcd->points[i].y,
                               steady_pcd->points[i].z);
        if (m_th == 0) {
            body_p = rotation * body_p + translation;
        }

        pcl::PointXYZINormal r_p = steady_pcd->points[i];
        r_p.x = body_p[0];
        r_p.y = body_p[1];
        r_p.z = body_p[2];
        pcl::PointXYZINormal pd;
        result_clouds[m_th].push_back(r_p);
        pub_steady_pcd->push_back(r_p);
    }

    string all_points_dir(
            steady_save + to_string(data_id) + "/" + to_string(my_index) + string(".pcd"));
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, *pub_steady_pcd);
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*pub_steady_pcd, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubRegisterCloud.publish(pub_cloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "std_loop");
    ros::NodeHandle nh;
    std::string data_name = "";
    std::string setting_path = "";

    int data_num = 2;
    int m_num = 2;
    int m_th = 0;
    std::string pcd_dir = "";
    std::string load_dir = "";
    std::string save_dir = "";

    std::string dir_front = "";
    std::string dir_back = "";

    std::string bag_file1 = "";
    std::string bag_file2 = "";
    std::string pose_file1 = "";
    std::string pose_file2 = "";
    std::string loop_gt_file = "";
    double icp_threshold = 0.5;
    bool calc_gt_enable = false;

    // dynamic points
    nh.param<string>("dyn_obj/out_file", out_file, "");
    nh.param<string>("dyn_obj/out_file_origin", out_file_origin, "");
    nh.param<int>("data_num", data_num, 2);
    nh.param<int>("m_num", m_num, 2);
    nh.param<string>("pcd_dir", pcd_dir, "");
    nh.param<std::string>("load_dir", load_dir, "");
    nh.param<std::string>("save_dir", save_dir, "");
    nh.param<std::string>("data_name", data_name, "");
    nh.param<std::string>("setting_path", setting_path, "");
    nh.param<std::string>("loop_gt_file", loop_gt_file, "");
    nh.param<bool>("calc_gt_enable", calc_gt_enable, false);

    nh.param<std::string>("dir_front", dir_front, "");
    nh.param<std::string>("dir_back", dir_back, "");
    nh.param<std::string>("steady_save", steady_save, "");

    nh.param<double>("icp_threshold", icp_threshold, 0.5);
    std::string icp_string = std::to_string(icp_threshold);
    std::string result_path =
            "/home/weihairuo/bag/" + data_name + "/" +
            data_name + "_" + icp_string.substr(0, icp_string.find(".") + 3) + ".txt";
    std::ofstream result_file(result_path);
    std::ofstream debug_file("/home/weihairuo/bag/log.txt");

    ros::Publisher pubOdomAftMapped =
            nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    ros::Publisher pubRegisterCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);

    // dynamic points
    ros::Publisher pub_pcl_dyn_extend = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl_dyn_extend", 10000);
    ros::Publisher cluster_vis_high = nh.advertise<visualization_msgs::MarkerArray>("/cluster_vis_high", 10000);
    ros::Publisher pub_ground_points = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected",
                                                                                100000); // dynamic points?
    ros::Publisher pubLaserCloudEffect_depth = nh.advertise<sensor_msgs::PointCloud2>("/cloud_dyn_obj_removed",
                                                                                      100000);


    ros::Rate loop(50000);
    ros::Rate late_loop(1000);

    ConfigSetting config_setting;
    load_config_setting(setting_path, config_setting);

    /// load pose and time(0.1s interval) from txt produced by Multi-Map
    std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>>
            pose_list_array;
    std::vector<std::vector<double>> time_list_array;

    /// load data from different sequences
    for (size_t data_id = 0; data_id < data_num; data_id++) {
        /// necessary preparation for DynObjFilter
        shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());
        DynObjFilt->SetParam(nh);
        DynObjFilt->Cluster.Init(pub_pcl_dyn_extend, cluster_vis_high, pub_ground_points);
        PointCloudXYZI::Ptr steady_pcd(new pcl::PointCloud<pcl::PointXYZINormal>);
        std::vector<pcl::PointCloud<pcl::PointXYZINormal>> result_clouds;
        result_clouds.resize(m_num);
        m_th = 0;

        /// load pose and time(0.1s interval) from txt produced by Multi-Map
        std::string pose_file = load_dir + std::to_string(data_id) + ".txt";
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
        std::vector<double> time_list;
        load_m_pose_with_time(pose_file, pose_list, time_list);
        pose_list_array.push_back(pose_list);
        time_list_array.push_back(time_list);
        std::string print_msg = "Sucessfully load pose file:" + pose_file +
                                ". pose size:" + std::to_string(time_list.size());
        ROS_INFO_STREAM(print_msg.c_str());

        /// load point clouds from pcd files
        for (int my_index = 1; my_index <= pose_list_array[data_id].size(); my_index++) {
            std::string pcd_file =
                    dir_front + std::to_string(data_id) + "/" + std::to_string(my_index) + ".pcd";
            debug_file << pcd_file << std::endl;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud_ptr) == -1) {
                PCL_ERROR("Failed to read PCD file: %s\n", pcd_file.c_str());
                continue;
            }

            if (cloud_ptr != NULL) {

                use_m_detector(DynObjFilt, *cloud_ptr, my_index, reinterpret_cast<int &>(data_id),
                               time_list, pose_list, pose_list_array,
                               config_setting, steady_pcd,
                               result_clouds, m_th,
                               pubOdomAftMapped, pubRegisterCloud,
                               pubLaserCloudEffect, pubLaserCloudEffect_depth,
                               debug_file, loop);

                late_loop.sleep();
//                getchar();
            }
        }

        m_th++;

        for (int my_index = pose_list_array[data_id].size(); my_index > 0; my_index--) {
            std::string pcd_file = dir_back + "/" + std::to_string(my_index) + ".pcd";
            debug_file << pcd_file << std::endl;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud_ptr) == -1) {
                PCL_ERROR("Failed to read PCD file: %s\n", pcd_file.c_str());
                continue;
            }

            if (cloud_ptr != NULL) {

                use_m_detector(DynObjFilt, *cloud_ptr, my_index, reinterpret_cast<int &>(data_id),
                               time_list, pose_list, pose_list_array,
                               config_setting, steady_pcd,
                               result_clouds, m_th,
                               pubOdomAftMapped, pubRegisterCloud,
                               pubLaserCloudEffect, pubLaserCloudEffect_depth,
                               debug_file, loop);

                late_loop.sleep();
//                getchar();
            }
        }
        string all_points_dir("/home/weihairuo/bag/pcd_correct/once_scan.pcd");
        pcl::PCDWriter pcd_writer;
        std::cout << "[ONCE]: " << result_clouds[0].size() << std::endl;
        pcd_writer.writeBinary(all_points_dir, result_clouds[0]);
        string all_points_dir_2("/home/weihairuo/bag/pcd_correct/twice_scan.pcd");
        pcl::PCDWriter pcd_writer_2;
        std::cout << "[TWICE]: " << result_clouds[1].size() << std::endl;
        pcd_writer_2.writeBinary(all_points_dir_2, result_clouds[1]);
    }
    return 0;
}