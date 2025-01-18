#include "../include/std.h"
#include "../include/std_ba.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

int findPoseIndexUsingTime(std::vector<double> &time_list, double &time) {
    double time_inc = 10000000000;
    int min_index = -1;
    for (size_t i = 0; i < time_list.size(); i++) {
        if (fabs(time_list[i] - time) < time_inc) {
            time_inc = fabs(time_list[i] - time);
            min_index = i;
        }
    }
    return min_index;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "std_loop");
    ros::NodeHandle nh;
    std::string data_name = "";
    std::string setting_path = "";
//  std::string save_dir = "";

    int data_num = 2;
    std::string root_dir = "";
    std::string load_dir = "";

    std::string bag_file1 = "";
    std::string bag_file2 = "";
    std::string pose_file1 = "";
    std::string pose_file2 = "";
    std::string loop_gt_file = "";
    double icp_threshold = 0.5;
    bool calc_gt_enable = false;
    bool is_rgb = true;
    bool is_body = false;
    pcl::PointCloud<pcl::PointXYZRGB> all_color_cloud;

    nh.param<int>("data_num", data_num, 2);
    nh.param<std::string>("root_dir", root_dir, "");
    nh.param<std::string>("load_dir", load_dir, "");
    nh.param<std::string>("data_name", data_name, "");
    nh.param<std::string>("setting_path", setting_path, "");
    nh.param<std::string>("loop_gt_file", loop_gt_file, "");
    nh.param<bool>("calc_gt_enable", calc_gt_enable, false);

    nh.param<double>("icp_threshold", icp_threshold, 0.5);
    nh.param<bool>("is_rgb", is_rgb, true);
    nh.param<bool>("is_body", is_body, false);
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

    ros::Rate loop(50000);
    ros::Rate late_loop(1000);

    ConfigSetting config_setting;
    load_config_setting(setting_path, config_setting);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_list;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> key_pose_list;

    int count = 0;
    bool is_skip_frame = false;
    bool is_build_descriptor = false;
    bool is_init_bag = false;
    Eigen::Vector3d init_translation(0, 0, 0);
    int key_frame_id = 0;
    int key_frame_last = 0;
    std::vector<int> id_inc_list;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_key_cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>>
            pose_list_array;
    std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>>
            pose_correct_list_array;
    std::vector<std::vector<double>> time_list_array;
    std::vector<Eigen::Vector3d> color_list;

    color_list.push_back(Eigen::Vector3d(255, 255, 255));
    color_list.push_back(Eigen::Vector3d(252, 233, 79));
    color_list.push_back(Eigen::Vector3d(138, 226, 52));
    color_list.push_back(Eigen::Vector3d(255, 0, 0));
    color_list.push_back(Eigen::Vector3d(0, 0, 255));

    for (size_t data_id = 0; data_id < data_num; data_id++) {
//        getchar();
        count = 0;
        id_inc_list.push_back(key_frame_id);
        key_frame_last = key_frame_id;
        std::string pose_file =
                load_dir + std::to_string(data_id) + ".txt";
        std::string pose_correct_file =
                load_dir + "pose_correct/" + std::to_string(data_id) + ".txt";
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_correct_list;
        std::vector<double> time_list;
        // path where the pcd restored
        std::string pcd_folder =
                "/home/weihairuo/Workspaces/FastLio2_ws/src/FAST_LIO/PCD/" + std::to_string(data_id) + "/";
        boost::filesystem::path pcd_path(pcd_folder);
        boost::filesystem::directory_iterator end_itr;
        load_evo_pose_with_time(pose_file, pose_list, time_list);
        load_evo_pose_with_time(pose_correct_file, pose_correct_list, time_list);
        pose_list_array.push_back(pose_list);
        pose_correct_list_array.push_back(pose_correct_list);
        time_list_array.push_back(time_list);
        std::string print_msg = "Sucessfully load pose file:" + pose_file +
                                ". pose size:" + std::to_string(time_list.size());
        ROS_INFO_STREAM(print_msg.c_str());
        debug_file << "[Data id]: " << data_id << ", key frame id:" << key_frame_id
                   << std::endl;

        for (int my_index = 1; my_index <= pose_list_array[data_id].size(); my_index++) {
            std::string pcd_file = root_dir + std::to_string(data_id) + "/" + std::to_string(my_index) + ".pcd";
            debug_file << pcd_file << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud_ptr) == -1) {
                PCL_ERROR("Failed to read PCD file: %s\n", pcd_file.c_str());
                continue;
            }

            if (cloud_ptr != NULL) {
                if (count == 0) {
                    if (!is_skip_frame) {
                        current_key_cloud->clear();
                    }
                }
                double laser_time = 0;
                pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
                pcl::copyPointCloud(*cloud_ptr, pcl_cloud);
                int pose_index = my_index - 1;
//              my_index = my_index + 4;
                std::cout << " pose_index " << my_index << std::endl;
                is_skip_frame = false;
                Eigen::Vector3d translation = pose_list[pose_index].first;
                Eigen::Matrix3d rotation = pose_list[pose_index].second;
                Eigen::Quaterniond q(rotation);

                nav_msgs::Odometry odom;
                odom.header.frame_id = "camera_init";
                odom.pose.pose.position.x = translation[0];
                odom.pose.pose.position.y = translation[1];
                odom.pose.pose.position.z = translation[2];
                odom.pose.pose.orientation.w = q.w();
                odom.pose.pose.orientation.x = q.x();
                odom.pose.pose.orientation.y = q.y();
                odom.pose.pose.orientation.z = q.z();
//              pubOdomAftMapped.publish(odom);
//              loop.sleep();

                debug_file << pose_index << " "
                           << odom.pose.pose.position.x << " "
                           << odom.pose.pose.position.y << " "
                           << odom.pose.pose.position.z << " "
                           << odom.pose.pose.orientation.x << " "
                           << odom.pose.pose.orientation.y << " "
                           << odom.pose.pose.orientation.z << " "
                           << odom.pose.pose.orientation.w << std::endl;

                Eigen::Vector3d translation_correct = pose_correct_list[pose_index].first;
                Eigen::Matrix3d rotation_correct = pose_correct_list[pose_index].second;
                Eigen::Quaterniond q_correct(rotation_correct);

                nav_msgs::Odometry odom_correct;
                odom_correct.header.frame_id = "camera_init";
                odom_correct.pose.pose.position.x = translation[0];
                odom_correct.pose.pose.position.y = translation[1];
                odom_correct.pose.pose.position.z = translation[2];
                odom_correct.pose.pose.orientation.w = q.w();
                odom_correct.pose.pose.orientation.x = q.x();
                odom_correct.pose.pose.orientation.y = q.y();
                odom_correct.pose.pose.orientation.z = q.z();
                pubOdomAftMapped.publish(odom_correct);
                loop.sleep();

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr register_cloud(
                        new pcl::PointCloud<pcl::PointXYZRGB>);
                for (size_t i = 0; i < pcl_cloud.size(); i++) {
                    Eigen::Vector3d pv(pcl_cloud.points[i].x, pcl_cloud.points[i].y,
                                       pcl_cloud.points[i].z);
//                  if (pv.norm() > 100) {
//                      continue;
//                  }
                    pv = config_setting.rot_lidar_to_vehicle_ * pv +
                         config_setting.t_lidar_to_vehicle_;
                    if (!is_body) {
                        pv = rotation.inverse() * (pv - translation);
                    }
                    pv = rotation_correct * pv + translation_correct;
                    pcl::PointXYZRGB pi = pcl_cloud.points[i];
                    pi.x = pv[0];
                    pi.y = pv[1];
                    pi.z = pv[2];
                    register_cloud->push_back(pi);
                }
//              down_sampling_voxel(*register_cloud, 0.5);
                pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
                for (size_t i = 0; i < register_cloud->size(); i += 2) {
                    pcl::PointXYZRGB pi;
                    pi.x = register_cloud->points[i].x;
                    pi.y = register_cloud->points[i].y;
                    pi.z = register_cloud->points[i].z;
                    if (is_rgb) {
                        pi.r = register_cloud->points[i].r;
                        pi.g = register_cloud->points[i].g;
                        pi.b = register_cloud->points[i].b;
                    } else {
                        pi.r = color_list[data_id][0];
                        pi.g = color_list[data_id][1];
                        pi.b = color_list[data_id][2];
                    }
//                  std::cout << " pi.r " << pi.r << " pi.g " << pi.g << " pi.b " << pi.b << std::endl;
                    if (is_rgb) {
                        if (pi.r != 0 && pi.g != 0 && pi.b != 0) {
                            color_cloud.push_back(pi);
                        }
                    } else {
                        color_cloud.push_back(pi);
                    }
                }
                sensor_msgs::PointCloud2 pub_cloud;
                pcl::toROSMsg(color_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubRegisterCloud.publish(pub_cloud);
//                pubRegisterCloud.publish(pub_cloud);
                all_color_cloud += color_cloud;
                late_loop.sleep();
            }
        }
    }
    //save point cloud
    std::string save_pcd_file = root_dir + "result.pcd";
    pcl::io::savePCDFileBinary(save_pcd_file, all_color_cloud);
    return 0;
}