//#include "std.h"
//#include "std_ba.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
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
string steady_save = "";
double lidar_end_time = 0;
bool is_rgb;

typedef struct ConfigSetting {
  /* for point cloud pre-preocess*/
  int stop_skip_enable_ = 0;
  float ds_size_ = 0.5;
  int useful_corner_num_ = 30;
  int point_skip_ = 1;

  /* for key points*/
  float plane_merge_normal_thre_;
  float plane_merge_dis_thre_;
  float plane_detection_thre_ = 0.01;
  float voxel_size_ = 1.0;
  int voxel_init_num_ = 10;
  int proj_plane_num_ = 3;
  float proj_image_resolution_ = 0.5;
  float proj_image_high_inc_ = 0.5;
  float proj_dis_min_ = 0;
  float proj_dis_max_ = 5;
  float summary_min_thre_ = 10;
  int line_filter_enable_ = 0;
  int touch_filter_enable_ = 0;

  /* for STD */
  float descriptor_near_num_ = 10;
  float descriptor_min_len_ = 1;
  float descriptor_max_len_ = 10;
  float non_max_suppression_radius_ = 3.0;
  float std_side_resolution_ = 0.2;

  /* for place recognition*/
  int skip_near_num_ = 20;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  float rough_dis_threshold_ = 0.03;
  float similarity_threshold_ = 0.7;
  float icp_threshold_ = 0.5;
  float normal_threshold_ = 0.1;
  float dis_threshold_ = 0.3;

  /* for data base*/
  int std_add_skip_frame_ = 1;

  /* for result record*/
  int is_kitti_ = 1;
  /* extrinsic for lidar to vehicle*/
  Eigen::Matrix3d rot_lidar_to_vehicle_;
  Eigen::Vector3d t_lidar_to_vehicle_;

  /* for gt file style*/
  int gt_file_style_ = 0;

} ConfigSetting;

void load_config_setting(std::string &config_file,
                         ConfigSetting &config_setting) {
  cv::FileStorage fSettings(config_file, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    std::cerr << "Failed to open settings file at: " << config_file
              << std::endl;
    exit(-1);
  }
  // pre-preocess
  config_setting.ds_size_ = fSettings["ds_size"];
  config_setting.useful_corner_num_ = fSettings["useful_corner_num"];
  config_setting.stop_skip_enable_ = fSettings["stop_skip_enable"];
  config_setting.point_skip_ = fSettings["point_skip"];

  // key points
  config_setting.plane_merge_normal_thre_ =
      fSettings["plane_merge_normal_thre"];
  config_setting.plane_merge_dis_thre_ = fSettings["plane_merge_dis_thre"];
  config_setting.plane_detection_thre_ = fSettings["plane_detection_thre"];
  config_setting.voxel_size_ = fSettings["voxel_size"];
  config_setting.voxel_init_num_ = fSettings["voxel_init_num"];
  config_setting.proj_plane_num_ = fSettings["proj_plane_num"];
  config_setting.proj_image_resolution_ = fSettings["proj_image_resolution"];
  config_setting.proj_image_high_inc_ = fSettings["proj_image_high_inc"];
  config_setting.proj_dis_min_ = fSettings["proj_dis_min"];
  config_setting.proj_dis_max_ = fSettings["proj_dis_max"];
  config_setting.summary_min_thre_ = fSettings["summary_min_thre"];
  config_setting.line_filter_enable_ = fSettings["line_filter_enable"];
  config_setting.touch_filter_enable_ = fSettings["touch_filter_enable"];

  // std descriptor
  config_setting.descriptor_near_num_ = fSettings["descriptor_near_num"];
  config_setting.descriptor_min_len_ = fSettings["descriptor_min_len"];
  config_setting.descriptor_max_len_ = fSettings["descriptor_max_len"];
  config_setting.non_max_suppression_radius_ = fSettings["max_constrait_dis"];
  config_setting.std_side_resolution_ = fSettings["triangle_resolution"];

  // candidate search
  config_setting.skip_near_num_ = fSettings["skip_near_num"];
  config_setting.candidate_num_ = fSettings["candidate_num"];
  config_setting.sub_frame_num_ = fSettings["sub_frame_num"];
  config_setting.rough_dis_threshold_ = fSettings["rough_dis_threshold"];
  config_setting.similarity_threshold_ = fSettings["similarity_threshold"];
  config_setting.icp_threshold_ = fSettings["icp_threshold"];
  config_setting.normal_threshold_ = fSettings["normal_threshold"];
  config_setting.dis_threshold_ = fSettings["dis_threshold"];

  // result record
  config_setting.is_kitti_ = fSettings["is_kitti"];

  // data base
  config_setting.std_add_skip_frame_ = fSettings["std_add_skip_frame"];

  // extrinsic
  cv::Mat T_lidar_to_vehicle;
  fSettings["T_lidar_to_vehicle"] >> T_lidar_to_vehicle;
  config_setting.rot_lidar_to_vehicle_ << T_lidar_to_vehicle.at<double>(0, 0),
      T_lidar_to_vehicle.at<double>(0, 1), T_lidar_to_vehicle.at<double>(0, 2),
      T_lidar_to_vehicle.at<double>(1, 0), T_lidar_to_vehicle.at<double>(1, 1),
      T_lidar_to_vehicle.at<double>(1, 2), T_lidar_to_vehicle.at<double>(2, 0),
      T_lidar_to_vehicle.at<double>(2, 1), T_lidar_to_vehicle.at<double>(2, 2);
  config_setting.t_lidar_to_vehicle_ << T_lidar_to_vehicle.at<double>(0, 3),
      T_lidar_to_vehicle.at<double>(1, 3), T_lidar_to_vehicle.at<double>(2, 3);

  config_setting.gt_file_style_ = fSettings["gt_file_style"];

  std::cout << "Sucessfully load config file:" << config_file << std::endl;
}

void load_m_pose_with_time(
        const std::string &pose_file,
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &pose_list,
        std::vector<double> &time_list) {
    time_list.clear();
    pose_list.clear();
    std::ifstream fin(pose_file);
    std::string line;
    Eigen::Matrix<double, 1, 7> temp_matrix;
    while (getline(fin, line)) {
        std::istringstream sin(line);
        std::vector<std::string> Waypoints;
        std::string info;
        int number = 0;
        while (getline(sin, info, ' ')) {
            if (number == 0) {
                double time;
                std::stringstream data;
                data << info;
                data >> time;
//                std::cout << "string:" << info << ", time: " << time << std::endl;
                time_list.push_back(time);
                number++;
            } else {
                double p;
                std::stringstream data;
                data << info;
                data >> p;
                temp_matrix[number - 1] = p;
                if (number == 7) {
                    Eigen::Vector3d translation(temp_matrix[0], temp_matrix[1],
                                                temp_matrix[2]);
                    Eigen::Quaterniond q(temp_matrix[6], temp_matrix[3], temp_matrix[4],
                                         temp_matrix[5]);
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
                    single_pose.first = translation;
                    single_pose.second = q.toRotationMatrix();
                    pose_list.push_back(single_pose);
                }
                number++;
            }
        }
    }
}

void use_m_detector(shared_ptr<DynObjFilter> &DynObjFilt,
                    pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, int &my_index, int &data_id,
                    std::vector<double> &time_list, std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &pose_list,
                    std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>> pose_list_array,
                    ConfigSetting &config_setting, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &steady_pcd,
                    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &result_clouds, int &m_th,
                    ros::Publisher &pubOdomAftMapped, ros::Publisher &pubRegisterCloud,
                    ros::Publisher &pubLaserCloudEffect, ros::Publisher &pubLaserCloudEffect_depth,
                    std::ofstream &debug_file, ros::Rate &loop) {

    int pose_index = my_index - 1;
    std::cout << " pose_index " << pose_index << std::endl;
    // time starts from 0, interval is 0.1s
    if (m_th == 0) {
        if (is_rgb) {
            lidar_end_time = (time_list[pose_index] - time_list[0]);
        } else {
            lidar_end_time = (time_list[pose_index] - time_list[0]) * 0.000000001;
        }
    } else {
        if (is_rgb) {
            lidar_end_time =
                    (time_list[pose_list_array[data_id].size() - 1] - (time_list[pose_index] - time_list[0]));

        } else {
            lidar_end_time =
                    (time_list[pose_list_array[data_id].size() - 1] - (time_list[pose_index] - time_list[0])) *
                    0.000000001;
        }
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

    // cloud used to use M-Detector, in body frame
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_for_m(new pcl::PointCloud<pcl::PointXYZINormal>);


    // pcl_cloud is in world frame only the first time to use M-Detector
    // turn into lidar frame to get dynamic_cloud (body frame)
    // turn into XYZINormal to use M-Detector
    for (size_t i = 0; i < pcl_cloud.size(); i++) {
        Eigen::Vector3d pv(pcl_cloud.points[i].x, pcl_cloud.points[i].y,
                           pcl_cloud.points[i].z);

        /// need to change pcl_cloud to lidar frame from imu frame only the first time to use M-Detector
        if (m_th == 0) {
            pv = config_setting.rot_lidar_to_vehicle_ * pv +
                 config_setting.t_lidar_to_vehicle_;
            pv = rotation.transpose() * (pv - translation);
        }
        pcl::PointXYZINormal pi1;
        pi1.x = pv[0];
        pi1.y = pv[1];
        pi1.z = pv[2];
        ///--------------------!!!!!!--------------------
        pi1.normal_x = static_cast<float>((uint16_t) pcl_cloud.points[i].r) / 255.0f;
        pi1.normal_y = static_cast<float>((uint16_t) pcl_cloud.points[i].g) / 255.0f;
        pi1.normal_z = static_cast<float>((uint16_t) pcl_cloud.points[i].b) / 255.0f;
        cloud_for_m->push_back(pi1);
        ///debug
        if (i == 0) {
            debug_file << "rgb for pcl_cloud: "
                       << static_cast<float>((uint16_t) pcl_cloud.points[i].r) / 255.0f << " "
                       << static_cast<float>((uint16_t) pcl_cloud.points[i].g) / 255.0f << " "
                       << static_cast<float>((uint16_t) pcl_cloud.points[i].b) / 255.0f << std::endl;
            debug_file << "rgb for cloud_for_m: "
                       << cloud_for_m->points[i].normal_x << " "
                       << cloud_for_m->points[i].normal_y << " "
                       << cloud_for_m->points[i].normal_z << std::endl;
        }
        ///debug
    }

    /// no need to down sample
//    down_sampling_voxel(*cloud_for_m, 0.5);

    /// detect and save steady points
    if (m_th == 0) {
        /// need rotation and translation to transform cloud_for_m to world frame
        DynObjFilt->filter(1, cloud_for_m, rotation, translation, lidar_end_time);

        /// save steady points in body frame for the second M-Detector
        Eigen::Matrix3d rotation_t = rotation.transpose();
        Eigen::Matrix3d rot_lidar_to_vehicle_inv = config_setting.rot_lidar_to_vehicle_.transpose();
        steady_pcd->clear();
        DynObjFilt->publish_dyn_rgb(steady_pcd, rotation_t, rot_lidar_to_vehicle_inv, translation,
                                    config_setting.t_lidar_to_vehicle_, pubLaserCloudEffect, pubLaserCloudEffect_depth,
                                    lidar_end_time);
        ///debug
        int i = 0;
        if (i == 0) {
            debug_file << "rgb for Dyn: " << DynObjFilt->laserCloudSteadObj_clus->points[i].normal_x << " "
                       << DynObjFilt->laserCloudSteadObj_clus->points[i].normal_y << " "
                       << DynObjFilt->laserCloudSteadObj_clus->points[i].normal_z << std::endl;
            debug_file << "rgb for steady_pcd: " << steady_pcd->points[i].normal_x << " "
                       << steady_pcd->points[i].normal_y << " " << steady_pcd->points[i].normal_z << std::endl;
        }
        ///debug
    } else {
        DynObjFilt->filter(1, cloud_for_m, rotation, translation, lidar_end_time);
        steady_pcd->clear();
        DynObjFilt->publish_dyn(steady_pcd, pubLaserCloudEffect, pubLaserCloudEffect_depth, lidar_end_time);
    }

    /// pub and save steady points in world frame as the result of M-Detector
    pcl::PointCloud<pcl::PointXYZRGB> pub_steady_pcd;
    for (size_t i = 0; i < steady_pcd->points.size(); i++) {
        Eigen::Vector3d body_p(steady_pcd->points[i].x, steady_pcd->points[i].y,
                               steady_pcd->points[i].z);
        if (m_th == 0) {
            body_p = rotation * body_p + translation;
        }

        pcl::PointXYZRGB r_p;
        r_p.x = body_p[0];
        r_p.y = body_p[1];
        r_p.z = body_p[2];
        r_p.r = static_cast<std::uint8_t>(steady_pcd->points[i].normal_x * 255.0f);
        r_p.g = static_cast<std::uint8_t>(steady_pcd->points[i].normal_y * 255.0f);
        r_p.b = static_cast<std::uint8_t>(steady_pcd->points[i].normal_z * 255.0f);
        if (r_p.r != 0 && r_p.g != 0 && r_p.b != 0) {
            result_clouds[m_th].push_back(r_p);
        }
        pub_steady_pcd.push_back(r_p);
    }

    string all_points_dir(
            steady_save + to_string(my_index) + string(".pcd"));
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, pub_steady_pcd);
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(pub_steady_pcd, pub_cloud);
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
    std::string load_dir = "";
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
    nh.param<std::string>("load_dir", load_dir, "");
    nh.param<std::string>("data_name", data_name, "");
    nh.param<std::string>("setting_path", setting_path, "");
    nh.param<std::string>("loop_gt_file", loop_gt_file, "");
    nh.param<bool>("calc_gt_enable", calc_gt_enable, false);
    nh.param<std::string>("dir_back", dir_back, "");
    nh.param<std::string>("steady_save", steady_save, "");

    nh.param<double>("icp_threshold", icp_threshold, 0.5);
    nh.param<bool>("is_rgb", is_rgb, true);

    std::string icp_string = std::to_string(icp_threshold);
    std::ofstream debug_file(steady_save + "log.txt");

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

//    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> result_clouds;
//    result_clouds.resize(m_num);

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
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>> result_clouds;
        result_clouds.resize(m_num);
        m_th = 0;

        /// load pose and time(0.1s interval) from txt produced by Multi-Map
        std::string pose_file = load_dir + "pose.txt";
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
                    load_dir + "pcd/" + std::to_string(my_index) + ".pcd";
            debug_file << pcd_file << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_no_rgb(new pcl::PointCloud<pcl::PointXYZI>);
            if (is_rgb) {
                if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud_ptr) == -1) {
                    PCL_ERROR("Failed to read PCD file: %s\n", pcd_file.c_str());
                    continue;
                }
            } else {
                pcd_file =
                        load_dir + "pcd/" + std::to_string(my_index) + ".pcd";
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud_ptr_no_rgb) == -1) {
                    PCL_ERROR("Failed to read PCD file: %s\n", pcd_file.c_str());
                    continue;
                }
                for (int i = 0; i < cloud_ptr_no_rgb->size(); i++) {
                    pcl::PointXYZRGB point;
                    point.x = cloud_ptr_no_rgb->points[i].x;
                    point.y = cloud_ptr_no_rgb->points[i].y;
                    point.z = cloud_ptr_no_rgb->points[i].z;
                    point.r = static_cast<std::uint8_t>(1.0 * 255.0f);
                    point.g = static_cast<std::uint8_t>(1.0 * 255.0f);
                    point.b = static_cast<std::uint8_t>(1.0 * 255.0f);
                    cloud_ptr->push_back(point);
                }
            }

            if (cloud_ptr != NULL) {

                use_m_detector(DynObjFilt, *cloud_ptr, my_index, reinterpret_cast<int &>(data_id),
                               time_list, pose_list, pose_list_array,
                               config_setting, steady_pcd,
                               result_clouds, m_th,
                               pubOdomAftMapped, pubRegisterCloud,
                               pubLaserCloudEffect, pubLaserCloudEffect_depth,
                               debug_file, loop);
//                getchar();

                late_loop.sleep();
            }
        }

        m_th++;

        for (int my_index = pose_list_array[data_id].size(); my_index > 0; my_index--) {
            std::string pcd_file = dir_back + "/" + std::to_string(my_index) + ".pcd";
            debug_file << pcd_file << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud_ptr) == -1) {
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
            }
        }
        string all_points_dir(steady_save + "once_scan.pcd");
        pcl::PCDWriter pcd_writer;
        std::cout << "[ONCE]: " << result_clouds[0].size() << std::endl;
        pcd_writer.writeBinary(all_points_dir, result_clouds[0]);
        string all_points_dir_2(steady_save + "twice_scan.pcd");
        pcl::PCDWriter pcd_writer_2;
        std::cout << "[TWICE]: " << result_clouds[1].size() << std::endl;
        pcd_writer_2.writeBinary(all_points_dir_2, result_clouds[1]);
    }
    return 0;
}