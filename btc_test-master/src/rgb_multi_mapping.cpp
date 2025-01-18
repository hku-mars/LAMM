#include "../include/std.h"
#include "../include/std_ba.h"
#include "../include/multi_mapping.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#define debug

// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward{
//     backward::SignalHandling sh;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "std_loop");
    ros::NodeHandle nh;
    std::string data_name = "";
    std::string setting_path = "";

    int data_num = 2;
    std::string root_dir = "";

    std::string bag_file1 = "";
    std::string bag_file2 = "";
    std::string pose_file1 = "";
    std::string pose_file2 = "";
    std::string loop_gt_file = "";
    double icp_threshold = 0.15; //0.5
    double dist_threshold = 0.5;
    double dist_threshold_multi = 0.5;
    double mean_threshold = 1.0;
    int frame_num_threshold = 50;
    int threshold_frame_neighbour = 30;
    bool calc_gt_enable = false;
    bool if_self;
    bool if_debug;
    bool is_body;
    double loopNoiseScore = 0.1;
    double odomNoiseScore = 0.1;

    nh.param<int>("data_num", data_num, 2);
    nh.param<std::string>("root_dir", root_dir, "");
    nh.param<std::string>("data_name", data_name, "");
    nh.param<std::string>("setting_path", setting_path, "");
    nh.param<std::string>("loop_gt_file", loop_gt_file, "");
    nh.param<bool>("calc_gt_enable", calc_gt_enable, false);
    nh.param<bool>("if_self", if_self, false);
    nh.param<bool>("if_debug", if_debug, false);
    nh.param<bool>("is_body", is_body, false);
    nh.param<double>("loopNoiseScore", loopNoiseScore, 0.1);
    nh.param<double>("odomNoiseScore", odomNoiseScore, 0.1);

    nh.param<double>("icp_threshold", icp_threshold, 0.15);
    nh.param<double>("dist_threshold", dist_threshold, 0.5);
    nh.param<double>("dist_threshold_multi", dist_threshold_multi, 0.5);
    nh.param<double>("mean_threshold", mean_threshold, 1.0);
    nh.param<int>("frame_num_threshold", frame_num_threshold, 50);
    nh.param<int>("threshold_frame_neighbour", threshold_frame_neighbour, 30);
    std::string icp_string = std::to_string(icp_threshold);
    std::string debug_path = root_dir + "pose_correct/debug.txt";
    std::ofstream debug_file(debug_path);
    std::string frame_path = root_dir + "pose_correct/frame.txt";
    std::ofstream frame_file(frame_path);
    std::string multi_graph_path = root_dir + "pose_correct/multi_graph.txt";
    std::ofstream multi_graph_file(multi_graph_path);
    std::string loop_filter_path = root_dir + "pose_correct/loop_filter.txt";
    std::ofstream loop_filter_file(loop_filter_path);
    std::string time_consump_path = root_dir + "pose_correct/time.txt";
    std::ofstream time_consump_file(time_consump_path);

    ros::Publisher pubOdomAftMapped =
            nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    ros::Publisher pubRegisterCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    ros::Publisher pubFirstCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_first", 100);
    ros::Publisher pubMergeCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_merge", 100);

    ros::Publisher pubCurrentCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
    ros::Publisher pubCurrentBinary =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
    ros::Publisher pubMatchedCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
    ros::Publisher pubSelfCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_self", 100);
    ros::Publisher pubMatchedBinary =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
    ros::Publisher pubMatchedBinary_now =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_now_key_points", 100);
    ros::Publisher pubSTD =
            nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
    ros::Publisher markerPub =
            nh.advertise<visualization_msgs::Marker>("marker_topic", 10);
    ros::Publisher pubCorrectCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_correct", 10000);
    ros::Publisher pubOdomCorreted =
            nh.advertise<nav_msgs::Odometry>("/odom_corrected", 10);

    ros::Publisher pubPairCloud1 =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_src", 100);
    ros::Publisher pubPairCloud2 =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_tar", 100);

    ros::Rate loop(50000);
    ros::Rate late_loop(100);

    ConfigSetting config_setting;
    load_config_setting(setting_path, config_setting);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_list;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> key_pose_list;

    std::vector<Eigen::Affine3d> pose_vec;
    std::vector<double> time_vec;

    // save all planes of key frame
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> history_plane_list;

    // save all binary descriptors of key frame
    std::vector<std::vector<BinaryDescriptor>> history_binary_list;

    // save all STD descriptors of key frame
    std::vector<std::vector<STD>> history_STD_list;

    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    //------------------split graph--------------------------
    gtsam::Values bags_maps;
    gtsam::NonlinearFactorGraph bags_graph;
    //------------------split graph--------------------------

    gtsam::Vector Vector6(6);
    Vector6 << odomNoiseScore, odomNoiseScore, odomNoiseScore,
            odomNoiseScore, odomNoiseScore, odomNoiseScore;//1e-6
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
            gtsam::noiseModel::Diagonal::Variances(Vector6);

    gtsam::Vector LVector6(6);
    LVector6 << 10000, 10000, 10000, 10000, 10000, 10000;
    gtsam::noiseModel::Diagonal::shared_ptr LodometryNoise =
            gtsam::noiseModel::Diagonal::Variances(LVector6);

    gtsam::Vector FixVector6(6);
    FixVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoiseFix =
            gtsam::noiseModel::Diagonal::Variances(FixVector6);
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise;
    gtsam::Vector robustNoiseVector6(
            6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
            loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

    int count = 0;
    int cloudInd = 0;
    int start_frame[data_num];

    bool is_skip_frame = false;
    bool is_build_descriptor = false;
    bool is_init_bag = false;
    Eigen::Vector3d init_translation(0, 0, 0);
    int key_frame_id = 0;
    int key_frame_last = 0;
    std::vector<int> id_inc_list;
    std::vector<int> pose_id_inc_list;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_key_cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>>
            pose_list_array;
    std::vector<std::vector<double>> time_list_array;
    //------------------------------build a vector to save STD_map of different bag---------------------------------------
    std::vector<std::unordered_map<STD_LOC, std::vector<STD>>> other_STD_map;
    for (size_t data_id = 0; data_id < data_num; data_id++) {
        // hash table, save all descriptor
        std::unordered_map<STD_LOC, std::vector<STD>> a_STD_map;
        other_STD_map.push_back(a_STD_map);
    }

    std::vector<std::pair<int, std::vector<std::pair<STD, STD>>>> std_pair_to_publish;

    int time_count = 0;

    /// time comsumption
    auto t_all_begin = std::chrono::high_resolution_clock::now();

    for (size_t data_id = 0; data_id < data_num; data_id++) {
        bool new_bag = true;
        count = 0;
        id_inc_list.push_back(key_frame_id);
        pose_id_inc_list.push_back(cloudInd);
        key_frame_last = key_frame_id;
        std::string pose_file = root_dir + std::to_string(data_id) + ".txt";
        std::string time_file = root_dir + "truth/" + std::to_string(data_id) + ".txt";
        /***----------------- open bag -------------***/
        std::string bag_file = root_dir + std::to_string(data_id) + ".bag";
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> no_use_pose_list;
        std::vector<double> time_list;
        boost::filesystem::directory_iterator end_itr;
        load_evo_pose_with_time(pose_file, pose_list, time_list);
        pose_list_array.push_back(pose_list);
        load_evo_pose_with_time(time_file, no_use_pose_list, time_list);
        time_list_array.push_back(time_list);
        std::string print_msg = "Sucessfully load pose file:" + pose_file +
                                ". pose size:" + std::to_string(time_list.size());
        ROS_INFO_STREAM(print_msg.c_str());
        debug_file << "[Data id]: " << data_id << ", key frame id:" << key_frame_id
                   << std::endl;

        // save all loop information for RANSAC
        std::vector<std::vector<LOOP_INFO>> loop_info_list;
        for (size_t i = 0; i < data_id + 1; i++) {
            std::vector<LOOP_INFO> loop_info;
            loop_info_list.push_back(loop_info);
        }

        int keyInd = 0;

        auto t_detect_begin = std::chrono::high_resolution_clock::now();

        for (int my_index = 1; my_index <= pose_list_array[data_id].size(); my_index++) {
            std::string pcd_file = root_dir + std::to_string(data_id) + "/" + std::to_string(my_index) + ".pcd";
            Eigen::Vector3d current_translation;
            Eigen::Matrix3d current_rotation;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud_ptr) == -1) {
                PCL_ERROR("Failed to read PCD file: %s\n", pcd_file.c_str());
                continue;
            }

            if (cloud_ptr != NULL) {
                if (count == 0) {
                    if (!is_skip_frame) {
                        current_key_cloud->clear();
                        keyInd = cloudInd;
                    }
                }
                pcl::PCLPointCloud2 pcl_pc;
                pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
                pcl::copyPointCloud(*cloud_ptr, pcl_cloud);
                int pose_index = my_index - 1;
                double laser_time = time_list[pose_index];
                time_count++;
                if (pose_index < 0) {
                    is_skip_frame = true;
                    continue;
                }
                is_skip_frame = false;
                Eigen::Vector3d translation = pose_list[pose_index].first;
                Eigen::Matrix3d rotation = pose_list[pose_index].second;
                Eigen::Quaterniond q(rotation);
                Eigen::Affine3d single_pose = Eigen::Affine3d::Identity();
                single_pose.translate(translation);
                single_pose.rotate(q);
                pose_vec.push_back(single_pose);
                time_vec.push_back(laser_time);
                initial.insert(cloudInd, gtsam::Pose3(pose_vec[cloudInd].matrix()));
                if (new_bag) {
                    if (data_id == 0) {
                        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                                cloudInd, gtsam::Pose3(pose_vec[cloudInd].matrix()),
                                odometryNoiseFix));
                        //-----------------split graphs---------------------------------
                        bags_maps.insert(cloudInd, gtsam::Pose3(pose_vec[cloudInd].matrix()));
                        bags_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                                data_id, gtsam::Pose3(pose_vec[cloudInd].matrix()),
                                odometryNoiseFix));
                        start_frame[data_id] = cloudInd;
                    } else {
                        bags_maps.insert(cloudInd, gtsam::Pose3(pose_vec[cloudInd].matrix()));
                        bags_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                                data_id, gtsam::Pose3(pose_vec[cloudInd].matrix()),
                                odometryNoiseFix));
                        start_frame[data_id] = cloudInd;
                    }
                    new_bag = false;
                } else {
                    int src_frame = cloudInd;
                    int tar_frame = cloudInd - 1;
                    //相邻帧之间的transform
                    auto prev_pose = gtsam::Pose3(pose_vec[cloudInd - 1].matrix());
                    auto curr_pose = gtsam::Pose3(pose_vec[cloudInd].matrix());

                    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            cloudInd - 1, cloudInd, prev_pose.between(curr_pose),
                            odometryNoise));
                }

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
                // loop.sleep();

                pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud(
                        new pcl::PointCloud<pcl::PointXYZI>);
                /// transform to first odom frame
                gtsam::Pose3 between_pose = gtsam::Pose3(pose_vec[keyInd].matrix()).between(
                        gtsam::Pose3(pose_vec[cloudInd].matrix()));
                Eigen::Vector3d trans_betw = between_pose.translation().vector();
                Eigen::Matrix3d rot_betw = between_pose.rotation().matrix();
                frame_file << "count: " << count << std::endl;
                frame_file << "cloudInd: " << cloudInd << " keyInd: " << keyInd << std::endl;
                frame_file << "trans_betw: " << trans_betw.transpose() << std::endl;
                frame_file << "rot_betw: " << rot_betw << std::endl;

                cloudInd++;

                for (size_t i = 0; i < pcl_cloud.size(); i++) {
                    Eigen::Vector3d pv(pcl_cloud.points[i].x, pcl_cloud.points[i].y,
                                       pcl_cloud.points[i].z);
                    pv = config_setting.rot_lidar_to_vehicle_ * pv +
                         config_setting.t_lidar_to_vehicle_;
                    if (!is_body) {
                        pv = rotation.inverse() * (pv - translation);
                    }

                    // back to key odom frame
                    pv = rot_betw * pv + trans_betw;
                    pcl::PointXYZI pi = pcl_cloud.points[i];
                    pi.x = pv[0];
                    pi.y = pv[1];
                    pi.z = pv[2];
                    register_cloud->push_back(pi);
                }
                if (is_body) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pub_world_cloud(
                            new pcl::PointCloud<pcl::PointXYZI>);
                    for (size_t i = 0; i < pcl_cloud.size(); i++) {
                        Eigen::Vector3d pv(pcl_cloud.points[i].x, pcl_cloud.points[i].y,
                                           pcl_cloud.points[i].z);
                        pv = rotation * pv + translation;
                        pcl::PointXYZI pi = pcl_cloud.points[i];
                        pi.x = pv[0];
                        pi.y = pv[1];
                        pi.z = pv[2];
                        pub_world_cloud->push_back(pi);
                    }
                    sensor_msgs::PointCloud2 pub_cloud;
                    pcl::toROSMsg(*pub_world_cloud, pub_cloud);
                    pub_cloud.header.frame_id = "camera_init";
                    pubRegisterCloud.publish(pub_cloud);
                } else {
                    sensor_msgs::PointCloud2 pub_cloud;
                    pcl::toROSMsg(pcl_cloud, pub_cloud);
                    pub_cloud.header.frame_id = "camera_init";
                    pubRegisterCloud.publish(pub_cloud);
                }

                late_loop.sleep();
                if (count == 0) {
                    if (!is_skip_frame) {
                        current_translation = translation;
                        current_rotation = rotation;
                    }
                }

                frame_file << "[key frame id]: " << key_frame_id
                           << ", [my_index]: " << my_index
                           << ", [pose index]: " << pose_index
                           << ", [cloud name]: " << pcd_file
                           << std::endl;

                /// down sample
                down_sampling_voxel(*register_cloud, config_setting.ds_size_);

                /// accumulate point cloud after down sampling
                for (size_t i = 0; i < register_cloud->size(); i++) {
                    current_key_cloud->points.push_back(register_cloud->points[i]);
                }
                if (count < config_setting.sub_frame_num_ - 1) {
                    count++;
                } else {
                    count = 0;
                    is_build_descriptor = true;
//                    getchar();
                }
            }
            if (is_build_descriptor) {
                debug_file << std::endl;
                debug_file << "Key frame:" << key_frame_id
                           << ", cloud size:" << current_key_cloud->size() << std::endl;
                Eigen::Quaterniond quaternion(current_rotation);
                std::pair<Eigen::Vector3d, Eigen::Matrix3d> key_pose;
                key_pose.first = current_translation;
                key_pose.second = current_rotation;
                key_pose_list.push_back(key_pose);
                std::cout << std::endl;
                std::cout << "Key Frame:" << key_frame_id
                          << ", cloud size:" << current_key_cloud->size() << std::endl;

                sensor_msgs::PointCloud2 curr_pub_cloud;
                pcl::toROSMsg(*current_key_cloud, curr_pub_cloud);
                curr_pub_cloud.header.frame_id = "camera_init";
                pubCurrentCloud.publish(curr_pub_cloud);

                std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
                /// voxel map is a hash table, key is the location of voxel, value is the pointer of octree
                /// in init function, octree will be used to calculate planes for each voxel
                /// calculated planes will be saved in octotrees in voxel_map's value
                init_voxel_map(config_setting, *current_key_cloud, voxel_map);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr frame_plane_cloud(
                        new pcl::PointCloud<pcl::PointXYZINormal>);
                /// get all planes in voxel_map
                get_plane(voxel_map, frame_plane_cloud);
                history_plane_list.push_back(frame_plane_cloud);
                std::vector<Plane *> proj_plane_list;
                std::vector<Plane *> merge_plane_list;
                /// get merge plane through voxel_map
                get_project_plane(config_setting, voxel_map, proj_plane_list);
                /// if current key frame cloud has no merge plane, add a single plane
                ///-------------------- maybe not a good idea --------------------------------
                if (proj_plane_list.size() == 0) {
                    debug_file << "key frame: " << key_frame_id << "No plane detected" << std::endl;
                    Plane *single_plane = new Plane;
                    single_plane->normal_ << 0, 0, 1;
                    single_plane->center_ = current_translation;
                    merge_plane_list.push_back(single_plane);
                } else {
                    /// sort the merge plane list by the number of the points in planes
                    sort(proj_plane_list.begin(), proj_plane_list.end(),
                         plane_greater_sort);
                    /// merge the plane in proj_plane_list, now planes in proj_plane_list are merged planes
                    /// in case the merged plane is in the same plane again
                    merge_plane(config_setting, proj_plane_list, merge_plane_list);
                    sort(merge_plane_list.begin(), merge_plane_list.end(),
                         plane_greater_sort);
                }
                std::cout << "merge_plane_list size:" << merge_plane_list.size()
                          << std::endl;
                std::vector<BinaryDescriptor> binary_list;
                std::vector<BinaryDescriptor> binary_around_list;
                /// extract binary descriptor from merge_plane_list
                binary_extractor(config_setting, merge_plane_list, current_key_cloud,
                                 binary_list);
                history_binary_list.push_back(binary_list);
                std::vector<STD> STD_list;
                /// extract STD descriptor from binary_list
                generate_std(config_setting, binary_list, key_frame_id, STD_list);

                //------------------------------do loop detection between different STD map-------------------------------------
                int loop_num;
                if (if_self) {
                    loop_num = data_id + 1;
                } else {
                    loop_num = data_id;
                }
                for (size_t map_num = 0; map_num < loop_num && ros::ok(); map_num++) {
                    bool loop_flag = false;
                    int match_frame = 0;
                    double this_icp = 0;
                    Eigen::Vector3d loop_translation;
                    Eigen::Matrix3d loop_rotation;
                    loop_detection(data_id, map_num, this_icp,
                                   loop_flag, icp_threshold, dist_threshold, frame_num_threshold, is_build_descriptor,
                                   config_setting,
                                   other_STD_map[map_num],
                                   history_plane_list,
                                   history_binary_list,
                                   key_frame_id, match_frame, STD_list,
                                   frame_plane_cloud,
                                   key_cloud_list,
                                   pose_vec,
                                   std_pair_to_publish,
                                   pubMatchedCloud, pubMatchedBinary, pubSTD,
                                   loop, debug_file, multi_graph_file,
                                   loop_translation, loop_rotation);

                    if (loop_flag) {
//                        if (key_frame_id - match_frame > frame_num_threshold) {
                        LOOP_INFO loop_info;
                        loop_info.map_num = map_num;
                        loop_info.data_id = data_id;
                        loop_info.icp_score = this_icp;
                        loop_info.key_frame_id = key_frame_id;
                        loop_info.match_frame = match_frame;
                        loop_info.enable = true;
                        loop_info.loop_translation = loop_translation;
                        loop_info.loop_rotation = loop_rotation;
                        /// calculate the external reference frame of the loop
                        int now_frame = key_frame_id * config_setting.sub_frame_num_;
                        auto delta_trans = Eigen::Affine3d::Identity();
                        delta_trans = Eigen::Affine3d::Identity();
                        delta_trans.translate(loop_translation);
                        delta_trans.rotate(loop_rotation);
                        Eigen::Affine3d now_pose = delta_trans * pose_vec[now_frame];
                        int old_frame = match_frame * config_setting.sub_frame_num_;
                        Eigen::Affine3d old_pose = pose_vec[old_frame];
                        gtsam::Pose3 loop_pose = gtsam::Pose3(old_pose.matrix()).between(
                                gtsam::Pose3(now_pose.matrix()));
                        loop_info.loop_pose = loop_pose;
                        /// for loop filter
                        loop_info.odom_pose_src.first = pose_vec[now_frame].translation();
                        loop_info.odom_pose_src.second = pose_vec[now_frame].rotation().matrix();
                        loop_info.odom_pose_tar.first = pose_vec[old_frame].translation();
                        loop_info.odom_pose_tar.second = pose_vec[old_frame].rotation().matrix();

                        pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud(
                                new pcl::PointCloud<pcl::PointXYZI>);
                        for (int i = 0; i < current_key_cloud->size(); i++) {
                            Eigen::Vector3d p(current_key_cloud->points[i].x, current_key_cloud->points[i].y,
                                              current_key_cloud->points[i].z);
                            p = current_rotation * p + current_translation;
                            pcl::PointXYZI pi = current_key_cloud->points[i];
                            pi.x = p[0];
                            pi.y = p[1];
                            pi.z = p[2];
                            src_cloud->points.push_back(pi);
                        }
                        pcl::PointCloud<pcl::PointXYZI>::Ptr tar_cloud(
                                new pcl::PointCloud<pcl::PointXYZI>);
                        for (int i = 0; i < key_cloud_list[match_frame]->size(); i++) {
                            Eigen::Vector3d p(key_cloud_list[match_frame]->points[i].x,
                                              key_cloud_list[match_frame]->points[i].y,
                                              key_cloud_list[match_frame]->points[i].z);
                            p = old_pose.rotation() * p + old_pose.translation();
                            pcl::PointXYZI pi = key_cloud_list[match_frame]->points[i];
                            pi.x = p[0];
                            pi.y = p[1];
                            pi.z = p[2];
                            tar_cloud->points.push_back(pi);
                        }
                        loop_info.src_cloud = *src_cloud;
                        loop_info.tar_cloud = *tar_cloud;

                        loop_info_list[map_num].push_back(loop_info);

                        sensor_msgs::PointCloud2 pub_self_cloud;
                        pcl::toROSMsg(*current_key_cloud, pub_self_cloud);
                        pub_self_cloud.header.frame_id = "camera_init";
                        pubSelfCloud.publish(pub_self_cloud);
                        loop.sleep();
//                        }
                    }
                }

                //------------------------------add STD of different map to different STD map-----------------------------------
                add_STD(other_STD_map[data_id], STD_list);


                is_build_descriptor = false;
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
                        new pcl::PointCloud<pcl::PointXYZI>);
                key_cloud_list.push_back(temp_cloud);
                down_sampling_voxel(*current_key_cloud, 0.5); // 0.1
                for (size_t i = 0; i < current_key_cloud->size(); i++) {
                    key_cloud_list.back()->push_back(current_key_cloud->points[i]);
                }

                key_frame_id++;
                for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
                    delete (iter->second);
                }
            }
        }

        auto t_detect_end = std::chrono::high_resolution_clock::now();
        std::cout << data_id << " detect time:" << time_inc(t_detect_end, t_detect_begin) << " ms" << std::endl;
        time_consump_file << data_id << " detect time:" << time_inc(t_detect_end, t_detect_begin) << " ms" << std::endl;

        ///
        auto t_filter_begin = std::chrono::high_resolution_clock::now();
        LoopFilter(loop_info_list, dist_threshold, mean_threshold, dist_threshold_multi, multi_graph_file, markerPub);
        auto t_filter_end = std::chrono::high_resolution_clock::now();
        std::cout << data_id << " loop filter time:" << time_inc(t_filter_end, t_filter_begin) << " ms" << std::endl;
        time_consump_file << data_id << " loop filter time:" << time_inc(t_filter_end, t_filter_begin) << " ms" << std::endl;

        if (if_debug) {
            debug_loop_after(loop_info_list, pubPairCloud1, pubPairCloud2, multi_graph_file, loop_filter_file,
                             std_pair_to_publish, pubSTD);
        }

        ///
        for (size_t loop_map_num = 0; loop_map_num <= data_id; loop_map_num++) {
            for (size_t loop_number = 0; loop_number < loop_info_list[loop_map_num].size(); loop_number++) {
                if (loop_info_list[loop_map_num][loop_number].enable) {
                    debug_file << loop_map_num << " size: " << loop_info_list[loop_map_num].size() << std::endl;
                    int key_frame_id = loop_info_list[loop_map_num][loop_number].key_frame_id;
                    int match_frame = loop_info_list[loop_map_num][loop_number].match_frame;
                    Eigen::Vector3d loop_translation = loop_info_list[loop_map_num][loop_number].loop_translation;
                    Eigen::Matrix3d loop_rotation = loop_info_list[loop_map_num][loop_number].loop_rotation;
                    int map_num = loop_info_list[loop_map_num][loop_number].map_num;
                    int sub_frame_num = config_setting.sub_frame_num_;
                    for (size_t j = 1; j <= sub_frame_num; j++) {
                        int src_frame = key_frame_id * sub_frame_num + j;

                        auto delta_T = Eigen::Affine3d::Identity();
                        delta_T.translate(loop_translation);
                        delta_T.rotate(loop_rotation);
                        Eigen::Affine3d src_pose_refined = delta_T * pose_vec[src_frame];

                        int tar_frame = match_frame * sub_frame_num + j;
                        // old
                        // Eigen::Affine3d tar_pose = pose_vec[tar_frame];
                        Eigen::Affine3d tar_pose = pose_vec[tar_frame];
                        gtsam::Pose3 loop_cons(delta_T.matrix());

                        if (src_frame < pose_vec.size()) {
                            multi_graph_file << "final add loop constraint:" << src_frame << "--"
                                             << tar_frame << "in: " << map_num << std::endl;
                            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                                    tar_frame, src_frame,
                                    loop_cons,
                                    robustLoopNoise));
                        }
                        //-----------------split graphs---------------------------
                        bags_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                                data_id, map_num,
                                gtsam::Pose3(tar_pose.matrix())
                                        .between(gtsam::Pose3(src_pose_refined.matrix())),
                                robustLoopNoise));
                        multi_graph_file << key_frame_id << "--"
                                         << match_frame << " " << data_id << "between " << map_num << std::endl;
                        debug_file << "final add loop constraint:" << src_frame << "--"
                                   << tar_frame << "in: " << map_num << std::endl;
                        //-----------------split graphs---------------------------
                        std::cout << "add loop constraint:" << src_frame << "--"
                                  << tar_frame << std::endl;
                    }
                }
            }
        }
    }

    pose_id_inc_list.push_back(cloudInd);

    auto t_split_begin = std::chrono::high_resolution_clock::now();

    //-------------------split graph-------------------------------
    std::vector<gtsam::NonlinearFactorGraph> subGraphs = splitFactorGraph(bags_graph);
    multi_graph_file << "Subgraph size: " << subGraphs.size() << std::endl;
    for (int num = 0; num < subGraphs.size(); num++) {
        multi_graph_file << "Number: " << num << std::endl;
        const gtsam::NonlinearFactor::shared_ptr &firstFactor = subGraphs[num].front();
        gtsam::Key firstKey = firstFactor->keys().front();
        int cloudnum = static_cast<int>(firstKey); //
        /// print match message----------------？？？？
        bool in_graph = false;
        for (int z = cloudnum; z < data_num; z++) {
            if (subGraphs[num].exists(gtsam::Key(z))) {
                multi_graph_file << "[key] " << z << std::endl;
            }
        }
        /// print match message
        multi_graph_file << "[first key] " << start_frame[cloudnum] << std::endl;
        if (cloudnum != 0) {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                    start_frame[cloudnum], gtsam::Pose3(pose_vec[cloudnum].matrix()), odometryNoiseFix));
        }
    }
    //-------------------split graph-------------------------------
    auto t_split_end = std::chrono::high_resolution_clock::now();
    std::cout << "gtsam split time:" << time_inc(t_split_end, t_split_begin) << " ms" << std::endl;
    time_consump_file << "gtsam split time:" << time_inc(t_split_end, t_split_begin) << " ms" << std::endl;

    auto t_pgo_begin = std::chrono::high_resolution_clock::now();

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);
    for (int i = 0; i < 10; i++) {
        isam.update();
    }
    auto t_pgo_end = std::chrono::high_resolution_clock::now();
    std::cout << "gtsam pgo time:" << time_inc(t_pgo_end, t_pgo_begin) << " ms"
              << std::endl;
    time_consump_file << "gtsam pgo time:" << time_inc(t_pgo_end, t_pgo_begin) << " ms"
                      << std::endl;
    gtsam::Values results = isam.calculateEstimate();
    std::vector<std::ofstream> correct_pose_list;
    int last_key_pose_index = 0;
    std::string truth_pose_file = root_dir + "pose_correct/" + "truth.txt";
    std::cout << "truth file:" << truth_pose_file << std::endl;
    std::ofstream truth_file(truth_pose_file);
    for (size_t data_id = 0; data_id < data_num; data_id++) {
        std::string correct_pose_file =
                root_dir + "pose_correct/" + std::to_string(data_id) + ".txt";
        std::cout << "correct file:" << correct_pose_file << std::endl;
        std::ofstream correct_file(correct_pose_file);

        for (size_t i = pose_id_inc_list[data_id];
             i < pose_id_inc_list[data_id + 1]; i++) {
            gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
            Eigen::Quaterniond correct_q(pose.rotation().matrix());
            correct_file << std::fixed << std::setprecision(6) << time_vec[i] * 1e-9
                         << std::setprecision(7) << " " << pose.translation()[0]
                         << " " << pose.translation()[1] << " "
                         << pose.translation()[2] << " " << correct_q.x() << " "
                         << correct_q.y() << " " << correct_q.z() << " "
                         << correct_q.w() << std::endl;
            truth_file << std::fixed << std::setprecision(6) << time_vec[i] * 1e-9
                       << std::setprecision(7) << " " << pose.translation()[0]
                       << " " << pose.translation()[1] << " "
                       << pose.translation()[2] << " " << correct_q.x() << " "
                       << correct_q.y() << " " << correct_q.z() << " "
                       << correct_q.w() << std::endl;
        }
    }

    auto t_all_end = std::chrono::high_resolution_clock::now();
    std::cout << "all time:" << time_inc(t_all_end, t_all_begin) << " ms" << std::endl;
    time_consump_file << "all time:" << time_inc(t_all_end, t_all_begin) << " ms" << std::endl;

    return 0;
}