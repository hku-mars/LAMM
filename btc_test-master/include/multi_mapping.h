#include "include/std.h"
#include "include/std_ba.h"
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
#include "backward.hpp"
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_histogram.h>
#include <gsl/gsl_histogram2d.h>
#include <pcl/segmentation/sac_segmentation.h>

#define debug

// Function to estimate the mode using Kernel Density Estimation (KDE)
double estimateMode(const std::vector<double>& data) {
    size_t n = data.size();

    // Sort the data in ascending order
    std::vector<double> sortedData = data;
    gsl_sort(&sortedData[0], 1, n);

    // Calculate the bandwidth using Silverman's rule of thumb
    double iqr = gsl_stats_quantile_from_sorted_data(&sortedData[0], 1, n, 0.75) - gsl_stats_quantile_from_sorted_data(&sortedData[0], 1, n, 0.25);
    double stddev = gsl_stats_sd(&sortedData[0], 1, n);
    double bandwidth = 0.9 * std::min(stddev, iqr / 1.34) * std::pow(n, -0.2);

    // Create a histogram
    gsl_histogram* hist = gsl_histogram_alloc(100);
    gsl_histogram_set_ranges_uniform(hist, sortedData[0], sortedData[n - 1]);

    // Calculate the histogram using KDE
    for (size_t i = 0; i < n; i++) {
        gsl_histogram_increment(hist, sortedData[i]);
    }

    // Find the bin with the highest count
    size_t maxBin = gsl_histogram_max_bin(hist);
    double mode = gsl_histogram_get(hist, maxBin);

    // Free the histogram memory
    gsl_histogram_free(hist);

    return mode;
}

std::vector<Eigen::Vector3d> randomSampling(const std::vector<Eigen::Vector3d>& data, double sampleRatio)
{
    std::vector<Eigen::Vector3d> shuffledData(data);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(shuffledData.begin(), shuffledData.end(), gen);

    int sampleSize = static_cast<int>(data.size() * sampleRatio);
    std::vector<Eigen::Vector3d> sampledData(shuffledData.begin(), shuffledData.begin() + sampleSize);

    return sampledData;
}

void meanRansac(const std::vector<Eigen::Vector3d> &data, Eigen::Vector3d &mean_point, double &percent, int &iters,
                double &threshold, double &mean, double &covariance) {
    // every time choose 50% points
    // if the number of points is less than 4, choose all points
    size_t n = data.size();
    if (n <= 4) {
        double x = 0;
        double y = 0;
        double z = 0;
        for (size_t i = 0; i < n; i++) {
            x += data[i][0];
            y += data[i][1];
            z += data[i][2];
        }
        mean_point[0] = x / n;
        mean_point[1] = y / n;
        mean_point[2] = z / n;
        // cout << "data size: " << n << endl;
        return;
    }
    double min_dist = 10000000;
    double size = data.size();
    std::vector<Eigen::Vector3d> finalData;
    for (int i = 0; i < iters; i++) {
        // randomly choose 50% points to find mean point
        std::vector<Eigen::Vector3d> sampledData = randomSampling(data, percent);

        // record the mean of each iteration, calculate the overall dists
        double x = 0;
        double y = 0;
        double z = 0;
        size = sampledData.size();
        for (size_t i = 0; i < sampledData.size(); i++) {
            x += sampledData[i][0] / sampledData.size();
            y += sampledData[i][1] / sampledData.size();
            z += sampledData[i][2] / sampledData.size();
        }
        Eigen::Vector3d mean;
        mean[0] = x;
        mean[1] = y;
        mean[2] = z;

        double dist = 0;
        for (size_t i = 0; i < sampledData.size(); i++) {
            dist += (sampledData[i] - mean).norm();

        }
        if (dist < min_dist) {
            min_dist = dist;
            mean_point = mean;
            finalData = sampledData;
        }
        // keep doing this until the minimal overall dists is less than threshold
        // or the number of iterations is larger than max_iter
        if (min_dist < threshold * sampledData.size()) {
            // cout << "min_dist: " << min_dist << endl;
            // cout << "threshold: " << threshold * sampledData.size() << endl;
            // cout << "iter: " << i << endl;
            break;
        }
    }
    // cout << "min_dist: " << min_dist << endl;
    // cout << "threshold: " << threshold * size << endl;
    // cout << "iter: " << iters << endl;
    for(int i = 0; i < finalData.size(); i++){
//        mean += (finalData[i] - mean_point).norm()/finalData.size();
        if ((finalData[i] - mean_point).norm() > mean) {
            mean = (finalData[i] - mean_point).norm();
        }
        covariance += (finalData[i] - mean_point).norm() * (finalData[i] - mean_point).norm()/finalData.size();
    }
    covariance = sqrt(covariance);
}

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

/// build sub-graphs ///
std::vector<gtsam::NonlinearFactorGraph> splitFactorGraph(const gtsam::NonlinearFactorGraph &graph) {
    std::vector<gtsam::Key> keys;
    std::unordered_map<gtsam::Key, bool> visited;

    for (const auto &factor: graph) {
        gtsam::KeyVector factorKeys = factor->keys();
        for (const auto &key: factorKeys) {
            keys.push_back(key);
            visited[key] = false;
        }
    }

    std::vector<gtsam::NonlinearFactorGraph> subgraphs;

    while (!keys.empty()) {
        gtsam::NonlinearFactorGraph subgraph;

        gtsam::Key startKey = *std::min_element(keys.begin(), keys.end());

        std::queue<gtsam::Key> queue;
        queue.push(startKey);

        while (!queue.empty()) {
            gtsam::Key currentKey = queue.front();
            queue.pop();

            if (visited[currentKey]) {
                continue;
            }

            visited[currentKey] = true;

            for (const auto &factor: graph) {
                gtsam::KeyVector factorKeys = factor->keys();
                if (std::find(factorKeys.begin(), factorKeys.end(), currentKey) != factorKeys.end()) {
                    subgraph.add(factor);
                }
            }

            for (const auto &factor: graph) {
                gtsam::KeyVector factorKeys = factor->keys();
                if (std::find(factorKeys.begin(), factorKeys.end(), currentKey) != factorKeys.end()) {
                    for (const auto &key: factorKeys) {
                        if (key != currentKey && !visited[key]) {
                            queue.push(key);
                        }
                    }
                }
            }
        }

        subgraphs.push_back(subgraph);

        keys.erase(std::remove_if(keys.begin(), keys.end(), [&visited](const gtsam::Key &key) {
            return visited[key];
        }), keys.end());
    }

    return subgraphs;
}
/// end build sub-graphs ///

void publish_std_world(const std::vector<std::pair<STD, STD>> &match_std_list,
                       const ros::Publisher &std_publisher,
                       std::pair<Eigen::Vector3d, Eigen::Matrix3d> odom_pose_src,
                       std::pair<Eigen::Vector3d, Eigen::Matrix3d> odom_pose_tar) {
    // publish descriptor
    // bool transform_enable = true;
    Eigen::Vector3d translation_src = odom_pose_src.first;
    Eigen::Matrix3d rotation_src = odom_pose_src.second;
    Eigen::Vector3d translation_tar = odom_pose_tar.first;
    Eigen::Matrix3d rotation_tar = odom_pose_tar.second;

    visualization_msgs::MarkerArray ma_line;
    visualization_msgs::Marker m_line;
    m_line.type = visualization_msgs::Marker::LINE_LIST;
    m_line.action = visualization_msgs::Marker::ADD;
    m_line.ns = "lines";
    // Don't forget to set the alpha!
    m_line.scale.x = 0.25;
    m_line.pose.orientation.w = 1.0;
    m_line.header.frame_id = "camera_init";
    m_line.id = 0;
    int max_pub_cnt = 1;
    for (auto var: match_std_list) {
        if (max_pub_cnt > 100) {
            break;
        }
        max_pub_cnt++;
        m_line.color.a = 0.8;
        m_line.points.clear();
        m_line.color.r = 0 / 255;
        m_line.color.g = 233.0 / 255;
        m_line.color.b = 0 / 255;
        geometry_msgs::Point p;
        p.x = var.second.binary_A_.location_[0];
        p.y = var.second.binary_A_.location_[1];
        p.z = var.second.binary_A_.location_[2];
        Eigen::Vector3d t_p;
        t_p << p.x, p.y, p.z;
        // if (transform_enable)
        t_p = rotation_tar * t_p + translation_tar;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        p.x = var.second.binary_B_.location_[0];
        p.y = var.second.binary_B_.location_[1];
        p.z = var.second.binary_B_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_tar * t_p + translation_tar;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        p.x = var.second.binary_C_.location_[0];
        p.y = var.second.binary_C_.location_[1];
        p.z = var.second.binary_C_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_tar * t_p + translation_tar;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        p.x = var.second.binary_B_.location_[0];
        p.y = var.second.binary_B_.location_[1];
        p.z = var.second.binary_B_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_tar * t_p + translation_tar;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        p.x = var.second.binary_C_.location_[0];
        p.y = var.second.binary_C_.location_[1];
        p.z = var.second.binary_C_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_tar * t_p + translation_tar;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        p.x = var.second.binary_A_.location_[0];
        p.y = var.second.binary_A_.location_[1];
        p.z = var.second.binary_A_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_tar * t_p + translation_tar;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        // another
        m_line.points.clear();
        // 252; 233; 79

        m_line.color.r = 1;
        m_line.color.g = 1;
        m_line.color.b = 1;
        // m_line.color.r = 252.0 / 255;
        // m_line.color.g = 233.0 / 255;
        // m_line.color.b = 79.0 / 255;
        p.x = var.first.binary_A_.location_[0];
        p.y = var.first.binary_A_.location_[1];
        p.z = var.first.binary_A_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_src * t_p + translation_src;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        p.x = var.first.binary_B_.location_[0];
        p.y = var.first.binary_B_.location_[1];
        p.z = var.first.binary_B_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_src * t_p + translation_src;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        p.x = var.first.binary_C_.location_[0];
        p.y = var.first.binary_C_.location_[1];
        p.z = var.first.binary_C_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_src * t_p + translation_src;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        p.x = var.first.binary_B_.location_[0];
        p.y = var.first.binary_B_.location_[1];
        p.z = var.first.binary_B_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_src * t_p + translation_src;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        p.x = var.first.binary_C_.location_[0];
        p.y = var.first.binary_C_.location_[1];
        p.z = var.first.binary_C_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_src * t_p + translation_src;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        p.x = var.first.binary_A_.location_[0];
        p.y = var.first.binary_A_.location_[1];
        p.z = var.first.binary_A_.location_[2];
        t_p << p.x, p.y, p.z;
        t_p = rotation_src * t_p + translation_src;
        p.x = t_p[0];
        p.y = t_p[1];
        p.z = t_p[2];
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        // debug
        // std_publisher.publish(ma_line);
        // std::cout << "var first: " << var.first.triangle_.transpose()
        //           << " , var second: " << var.second.triangle_.transpose()
        //           << std::endl;
        // getchar();
    }
    for (int j = 0; j < 100 * 6; j++) {
        m_line.color.a = 0.00;
        ma_line.markers.push_back(m_line);
        m_line.id++;
    }
    std_publisher.publish(ma_line);
    m_line.id = 0;
    ma_line.markers.clear();
}

/// detect loop between different maps ///
void loop_detection(size_t &data_id, size_t &map_num, double &this_icp,
                    bool &loop_flag, double &icp_threshold, double &dist_threshold, int &frame_num_threshold,
                    bool &is_build_descriptor,
                    ConfigSetting &config_setting,
                    std::unordered_map<STD_LOC, std::vector<STD>> &STD_map,
                    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &history_plane_list,
                    std::vector<std::vector<BinaryDescriptor>> &history_binary_list,
                    int &key_frame_id, int &match_frame, std::vector<STD> &STD_list,
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &frame_plane_cloud,
                    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &key_cloud_list,
                    std::vector<Eigen::Affine3d> &pose_vec,
                    std::vector<std::pair<int, std::vector<std::pair<STD, STD>>>> &std_pair_to_publish,
                    ros::Publisher &pubMatchedCloud, ros::Publisher &pubMatchedBinary, ros::Publisher &pubSTD,
                    ros::Rate &loop, std::ofstream &debug_file, std::ofstream &multi_graph_file,
                    Eigen::Vector3d &loop_translation, Eigen::Matrix3d &loop_rotation) {
    std::vector<STDMatchList> alternative_match;
/// search for candidate loop key frame and save match STD list
    candidate_searcher_old(config_setting, STD_map, STD_list,
                           alternative_match);
// geometrical verification
    bool triggle_loop = false;
    Eigen::Vector3d best_t;
    Eigen::Matrix3d best_rot;
    std::vector<std::pair<STD, STD>> sucess_match_list;
    std::vector<std::pair<STD, STD>> unsucess_match_list;
    std::vector<std::pair<STD, STD>> sucess_match_list_publish;
    std::vector<std::pair<STD, STD>> unsucess_match_list_publish;
    int match_size = 0;
    int rough_size = 0;
    int candidate_id = -1;
    double mean_triangle_dis = 0;
    double mean_binary_similarity = 0;
    double outlier_mean_triangle_dis = 0;
    double outlier_mean_binary_similarity = 0;
    match_frame = 0;
    double best_score = 0;
    double best_icp_score = 0;
    int best_frame = -1;
    for (int i = 0; i < alternative_match.size(); i++) {
        if (alternative_match[i].match_list_.size() >= 4) {
#ifdef debug
            debug_file << "[Rough match] rough match frame:"
                       << alternative_match[i].match_frame_ << " match size:"
                       << alternative_match[i].match_list_.size() << std::endl;
#endif
            bool fine_sucess = false;
            Eigen::Matrix3d std_rot;
            Eigen::Vector3d std_t;
            sucess_match_list.clear();

            fine_loop_detection_tbb(config_setting, alternative_match[i].match_list_, fine_sucess,
                                    std_rot, std_t, sucess_match_list, unsucess_match_list);
/// ------std_rot and std_t are in world frame------
            if (fine_sucess) {
/// use std_rot and std_t to transform current key frame plane cloud to match frame
/// calculate overlap planes percentage
                double plane_icp_score = geometric_verify(config_setting, frame_plane_cloud,
                                                          history_plane_list[alternative_match[i].match_frame_],
                                                          std_rot, std_t);
                double icp_score = plane_icp_score;
// if (plane_icp_score > 0.5) {
//   icp_score = ICP_verify(
//       current_key_cloud,
//       key_cloud_list[alternative_match[i].match_frame_],
//       std_rot, std_t);
// }
#ifdef debug
                debug_file << "Fine sucess, Fine size:"
                           << sucess_match_list.size()
                           << "  ,Icp score:" << icp_score
                           << ", best score:" << best_icp_score
                           << ", icp_threshold:" << icp_threshold <<
                           std::endl;
#endif
                if (icp_score > best_icp_score) {
                    unsucess_match_list_publish = unsucess_match_list;
                    sucess_match_list_publish = sucess_match_list;
                    best_frame = alternative_match[i].match_frame_;
// best_score = score;
                    best_icp_score = icp_score;
                    best_rot = std_rot;
                    best_t = std_t;
                    rough_size = alternative_match[i].match_list_.size();
                    match_size = sucess_match_list.size();
                    candidate_id = i;
                }
            }
        }
    }
    debug_file << "[Here for trigger]: "
               << key_frame_id << "--" << best_frame << " "
               << "best icp score: " << best_icp_score
               << " ,icp_threshold: " << icp_threshold <<
               std::endl;
    if (best_icp_score > icp_threshold) {
        loop_translation = best_t;
        loop_rotation = best_rot;
        match_frame = best_frame;
        this_icp = best_icp_score;
        triggle_loop = true;
        mean_triangle_dis = calc_triangle_dis(sucess_match_list_publish);
        mean_binary_similarity =
                calc_binary_similaity(sucess_match_list_publish);
        outlier_mean_triangle_dis =
                calc_triangle_dis(unsucess_match_list_publish);
        outlier_mean_triangle_dis =
                calc_binary_similaity(unsucess_match_list_publish);
    } else {
        triggle_loop = false;
    }

    auto t_fine_loop_end = std::chrono::high_resolution_clock::now();
    is_build_descriptor = false;

    if (data_id == map_num) {
        if (key_frame_id - match_frame < frame_num_threshold) {
            triggle_loop = false;
        }
    }

    if (triggle_loop) {
#ifdef debug
//        getchar();
        debug_file << "[Loop Sucess] " << key_frame_id << "--"
                   << match_frame << ", candidate id:" << candidate_id
                   << ", icp:" << best_icp_score <<
                   std::endl;
        debug_file << "[Loop Info] "
                   << "rough size:" << rough_size
                   << ", match size:" << match_size
                   << ", rough triangle dis:" << outlier_mean_triangle_dis
                   << ", fine triangle dis:" << mean_triangle_dis
                   << ", rough binary similarity:"
                   << outlier_mean_triangle_dis
                   << ", fine binary similarity:" << mean_binary_similarity
                   <<
                   std::endl;
        multi_graph_file << "[Loop Sucess] " << key_frame_id << "--"
                         << match_frame << ", candidate id:" << candidate_id
                         << ", icp:" << best_icp_score <<
                         std::endl;
        multi_graph_file << "[Loop Info] "
                         << "rough size:" << rough_size
                         << ", match size:" << match_size
                         << ", rough triangle dis:" << outlier_mean_triangle_dis
                         << ", fine triangle dis:" << mean_triangle_dis
                         << ", rough binary similarity:"
                         << outlier_mean_triangle_dis
                         << ", fine binary similarity:" << mean_binary_similarity
                         <<
                         std::endl;
#endif
        std::cout << "triggle loop:" << key_frame_id << "--" << match_frame
                  <<
                  std::endl;
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(*key_cloud_list[match_frame], pub_cloud
        );
        pub_cloud.header.
                frame_id = "camera_init";
        pubMatchedCloud.
                publish(pub_cloud);

// loop.sleep();
        pcl::PointCloud<pcl::PointXYZ> matched_key_points_cloud;
        for (
            auto var
                : history_binary_list[match_frame]) {
            pcl::PointXYZ pi;
            pi.x = var.location_[0];
            pi.y = var.location_[1];
            pi.z = var.location_[2];
            matched_key_points_cloud.push_back(pi);
        }
        pcl::toROSMsg(matched_key_points_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedBinary.publish(pub_cloud);
        Eigen::Vector3d color2(0, 1, 0);
        publish_binary(history_binary_list[match_frame], color2,
                       "history", pubSTD);
//        loop.sleep();
        publish_std(sucess_match_list_publish, pubSTD);

        std::pair<int, std::vector<std::pair<STD, STD>>> pair_to_publish;
        pair_to_publish.first = key_frame_id;
        pair_to_publish.second = sucess_match_list_publish;
        std_pair_to_publish.push_back(pair_to_publish);

        geometric_icp(frame_plane_cloud, history_plane_list[match_frame],
                      loop_rotation, loop_translation);
        std::cout << "loop translation: " << loop_translation.transpose() << std::endl
                  << ", loop rotation:" << loop_rotation << std::endl;
// getchar();

/// add loop constraint to debug file ///
        Eigen::Vector3d loop_rotation_euler;
//roll pitch yaw
        loop_rotation_euler = loop_rotation.eulerAngles(0, 1, 2);
        debug_file << "[GTSAM] add loop connection:" << key_frame_id << "--"
                   << match_frame << std::endl;
        debug_file << "[LOOP TRANSFORMATION]: " << loop_translation.transpose() <<
                   std::endl;
        debug_file << "[LOOP ROTATION]: " << loop_rotation_euler[0] / 3.14 * 180
                   << " " << loop_rotation_euler[1] / 3.14 * 180 << " "
                   << loop_rotation_euler[2] / 3.14 * 180 << std::endl;

        std::cout << "[GTSAM] add loop connection:" << key_frame_id << "--"
                  << match_frame << std::endl;
        loop_flag = true;
    } else {
        std::cout << "loop fail:" << std::endl;
        loop_flag = false;
    }
}

typedef struct LOOP_INFO {
    int key_frame_id;
    int match_frame;
    int data_id;
    int map_num;
    bool enable;
    double icp_score;
    double dist;
    double mean;
    double covariance;
    Eigen::Vector3d loop_translation;
    Eigen::Matrix3d loop_rotation;
    gtsam::Pose3 loop_pose;
    Eigen::Vector3d external_point;
    Eigen::Vector3d mean_point;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> odom_pose_src;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> odom_pose_tar;
    pcl::PointCloud<pcl::PointXYZI> src_cloud;
    pcl::PointCloud<pcl::PointXYZI> tar_cloud;
} LOOP_INFO;

/// debug loop
void debug_loop(std::vector<std::vector<LOOP_INFO>> &loop_info_list,
                ros::Publisher &pubPairCloud1, ros::Publisher &pubPairCloud2,
                std::ofstream &multi_graph_file, std::ofstream &loop_filter_file,
                ros::Publisher &pubMatchedBinary, ros::Publisher &pubMatchedBinary_now,
                std::vector<std::vector<BinaryDescriptor>> &history_binary_list,
                std::vector<std::pair<int, std::vector<std::pair<STD, STD>>>> &std_pair_to_publish,
                ros::Publisher &pubSTD) {
// cycle for curr sequence with each sequence
    for (size_t i = 0; i < loop_info_list.size() - 1; i++) {
        loop_filter_file << "map: " << i << " with map: " << loop_info_list.size() - 1 << std::endl;
        for (size_t j = 0; j < loop_info_list[i].size(); j++) {
            if (loop_info_list[i][j].enable) {
                std::cout << "[" << loop_info_list.size() - 1 << "]--"
                          << "[" << i << "] loop: " << j
                          << " icp: " << loop_info_list[i][j].icp_score <<
                          std::endl;
                loop_filter_file << "[" << loop_info_list.size() - 1 << "]--"
                                 << "[" << i << "] loop: " << j
                                 << " icp: " << loop_info_list[i][j].icp_score <<
                                 std::endl;
                sensor_msgs::PointCloud2 pub_cloud;
                pcl::toROSMsg(loop_info_list[i][j].src_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubPairCloud1.publish(pub_cloud);
                pcl::toROSMsg(loop_info_list[i][j].tar_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubPairCloud2.publish(pub_cloud);

                int match_frame = loop_info_list[i][j].match_frame;
                int key_frame_id = loop_info_list[i][j].key_frame_id;
                pcl::PointCloud<pcl::PointXYZ> matched_key_points_cloud;

                Eigen::Vector3d translation = loop_info_list[i][j].odom_pose_tar.first;
                Eigen::Matrix3d rotation = loop_info_list[i][j].odom_pose_tar.second;
                Eigen::Vector3d t_p;

                for (auto var: history_binary_list[match_frame]) {
                    pcl::PointXYZ pi;
                    pi.x = var.location_[0];
                    pi.y = var.location_[1];
                    pi.z = var.location_[2];
                    t_p << pi.x, pi.y, pi.z;
                    t_p = rotation * t_p + translation;
                    pi.x = t_p[0];
                    pi.y = t_p[1];
                    pi.z = t_p[2];
                    matched_key_points_cloud.push_back(pi);
                }
                pcl::toROSMsg(matched_key_points_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubMatchedBinary.publish(pub_cloud);

                translation = loop_info_list[i][j].odom_pose_src.first;
                rotation = loop_info_list[i][j].odom_pose_src.second;
                for (auto var: history_binary_list[key_frame_id]) {
                    pcl::PointXYZ pi;
                    pi.x = var.location_[0];
                    pi.y = var.location_[1];
                    pi.z = var.location_[2];
                    t_p << pi.x, pi.y, pi.z;
                    t_p = rotation * t_p + translation;
                    pi.x = t_p[0];
                    pi.y = t_p[1];
                    pi.z = t_p[2];
                    matched_key_points_cloud.push_back(pi);
                }
                pcl::toROSMsg(matched_key_points_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubMatchedBinary_now.publish(pub_cloud);

                int pair_id = key_frame_id;
                for (int k = 0; k < std_pair_to_publish.size(); k++) {
                    if (std_pair_to_publish[k].first == pair_id) {
                        publish_std_world(std_pair_to_publish[k].second, pubSTD,
                                          loop_info_list[i][j].odom_pose_src,
                                          loop_info_list[i][j].odom_pose_tar);
                    }
                }

                getchar();

            }
        }
    }
    loop_filter_file << "map: " << loop_info_list.size() - 1
                     << " with map: " << loop_info_list.size() - 1 <<
                     std::endl;
    for (size_t j = 0; j < loop_info_list[loop_info_list.size() - 1].size(); j++) {
        std::cout << "[" << loop_info_list.size() - 1 << "]--"
                  << "[" << loop_info_list.size() - 1 << "] loop: " << j
                  << " icp: " << loop_info_list[loop_info_list.size() - 1][j].icp_score <<
                  std::endl;
        loop_filter_file << "[" << loop_info_list.size() - 1 << "]--"
                         << "[" << loop_info_list.size() - 1 << "] loop: " << j
                         << " icp: " << loop_info_list[loop_info_list.size() - 1][j].icp_score <<
                         std::endl;
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(loop_info_list[loop_info_list.size() - 1][j].src_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubPairCloud1.publish(pub_cloud);

        pcl::toROSMsg(loop_info_list[loop_info_list.size() - 1][j].tar_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubPairCloud2.publish(pub_cloud);

        int match_frame = loop_info_list[loop_info_list.size() - 1][j].match_frame;
        pcl::PointCloud<pcl::PointXYZ> matched_key_points_cloud;

        Eigen::Vector3d translation = loop_info_list[loop_info_list.size() - 1][j].odom_pose_tar.first;
        Eigen::Matrix3d rotation = loop_info_list[loop_info_list.size() - 1][j].odom_pose_tar.second;
        Eigen::Vector3d t_p;

        for (auto var: history_binary_list[match_frame]) {
            pcl::PointXYZ pi;
            pi.x = var.location_[0];
            pi.y = var.location_[1];
            pi.z = var.location_[2];
            t_p << pi.x, pi.y, pi.z;
            t_p = rotation * t_p + translation;
            pi.x = t_p[0];
            pi.y = t_p[1];
            pi.z = t_p[2];
            matched_key_points_cloud.push_back(pi);
        }
        pcl::toROSMsg(matched_key_points_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedBinary.publish(pub_cloud);

        translation = loop_info_list[loop_info_list.size() - 1][j].odom_pose_src.first;
        rotation = loop_info_list[loop_info_list.size() - 1][j].odom_pose_src.second;
        for (auto var: history_binary_list[loop_info_list[loop_info_list.size() - 1][j].key_frame_id]) {
            pcl::PointXYZ pi;
            pi.x = var.location_[0];
            pi.y = var.location_[1];
            pi.z = var.location_[2];
            t_p << pi.x, pi.y, pi.z;
            t_p = rotation * t_p + translation;
            pi.x = t_p[0];
            pi.y = t_p[1];
            pi.z = t_p[2];
            matched_key_points_cloud.push_back(pi);
        }
        pcl::toROSMsg(matched_key_points_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedBinary_now.publish(pub_cloud);

        int pair_id = loop_info_list[loop_info_list.size() - 1][j].key_frame_id;
        for (int k = 0; k < std_pair_to_publish.size(); k++) {
            if (std_pair_to_publish[k].first == pair_id) {
                publish_std_world(std_pair_to_publish[k].second, pubSTD,
                                  loop_info_list[loop_info_list.size() - 1][j].odom_pose_src,
                                  loop_info_list[loop_info_list.size() - 1][j].odom_pose_tar);
            }
        }
        getchar();
    }
}

void debug_loop_after(std::vector<std::vector<LOOP_INFO>> &loop_info_list,
                      ros::Publisher &pubPairCloud1, ros::Publisher &pubPairCloud2,
                      std::ofstream &multi_graph_file, std::ofstream &loop_filter_file,
                      std::vector<std::pair<int, std::vector<std::pair<STD, STD>>

                      >> &std_pair_to_publish,
                      ros::Publisher &pubSTD
) {
    for (size_t i = 0; i < loop_info_list.size() - 1; i++) {
        loop_filter_file << "map: " << i << " with map: " << loop_info_list.size() - 1 << std::endl;
        for (size_t j = 0; j < loop_info_list[i].size(); j++) {
            std::cout << "[" << loop_info_list.size() - 1 << "]--"
                      << "[" << i << "] loop: " << j
                      << " icp: " << loop_info_list[i][j].icp_score <<
                      std::endl;
            loop_filter_file << "[" << loop_info_list.size() - 1 << "]--"
                             << "[" << i << "] loop: " << j
                             << " icp: " << loop_info_list[i][j].icp_score <<
                             std::endl;
            loop_filter_file << "         external: "
                             << loop_info_list[i][j].external_point.transpose() <<
                             std::endl;
            loop_filter_file << "         mean: "
                             << loop_info_list[i][j].mean_point.transpose() <<
                             std::endl;
            std::cout << "         dis: "
                      << (loop_info_list[i][j].external_point - loop_info_list[i][j].mean_point).norm() <<
                      std::endl;
            loop_filter_file << "         dis: "
                             << (loop_info_list[i][j].external_point - loop_info_list[i][j].mean_point).norm() <<
                             std::endl;
            std::cout << "         threshold: "
                      << loop_info_list[i][j].covariance << std::endl;
            if (loop_info_list[i][j].enable) {
                std::cout << "Sucess!" << std::endl;
                loop_filter_file << "Sucess!" << std::endl;

                sensor_msgs::PointCloud2 pub_cloud;
                pcl::toROSMsg(loop_info_list[i][j].src_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubPairCloud1.publish(pub_cloud);

                pcl::toROSMsg(loop_info_list[i][j].tar_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubPairCloud2.publish(pub_cloud);
            } else {
                std::cout << "Fail!" << std::endl;
                loop_filter_file << "Fail!" << std::endl;

                sensor_msgs::PointCloud2 pub_cloud;
                pcl::toROSMsg(loop_info_list[i][j].src_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubPairCloud1.publish(pub_cloud);

                pcl::toROSMsg(loop_info_list[i][j].tar_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubPairCloud2.publish(pub_cloud);
            }
            getchar();
        }
    }
    loop_filter_file << "map: " << loop_info_list.size() - 1
                     << " with map: " << loop_info_list.size() - 1 << std::endl;
    for (size_t j = 0; j < loop_info_list[loop_info_list.size() - 1].size(); j++) {
        std::cout << "[" << loop_info_list.size() - 1 << "]--"
                  << "[" << loop_info_list.size() - 1 << "] loop: " << j
                  << " icp: " << loop_info_list[loop_info_list.size() - 1][j].icp_score <<
                  std::endl;
        loop_filter_file << "[" << loop_info_list.size() - 1 << "]--"
                         << "[" << loop_info_list.size() - 1 << "] loop: " << j
                         << " icp: " << loop_info_list[loop_info_list.size() - 1][j].icp_score <<
                         std::endl;
        std::cout << "         dis: "
                  << loop_info_list[loop_info_list.size() - 1][j].dist <<
                  std::endl;
        loop_filter_file << "         dis: "
                         << loop_info_list[loop_info_list.size() - 1][j].dist <<
                         std::endl;
        if (loop_info_list[loop_info_list.size() - 1][j].enable) {
            std::cout << "Sucess!" << std::endl;
            loop_filter_file << "Sucess!" << std::endl;

            sensor_msgs::PointCloud2 pub_cloud;
            pcl::toROSMsg(loop_info_list[loop_info_list.size() - 1][j].src_cloud, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pubPairCloud1.publish(pub_cloud);

            pcl::toROSMsg(loop_info_list[loop_info_list.size() - 1][j].tar_cloud, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pubPairCloud2.publish(pub_cloud);
        } else {
            std::cout << "Fail!" << std::endl;
            loop_filter_file << "Fail!" << std::endl;

            sensor_msgs::PointCloud2 pub_cloud;
            pcl::toROSMsg(loop_info_list[loop_info_list.size() - 1][j].src_cloud, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pubPairCloud1.publish(pub_cloud);

            pcl::toROSMsg(loop_info_list[loop_info_list.size() - 1][j].tar_cloud, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pubPairCloud2.publish(pub_cloud);
        }
        getchar();

    }
}

void publishLines(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &points, ros::Publisher &markerPub) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto &pair: points) {
        geometry_msgs::Point p1, p2;
        p1.x = pair.first.x();
        p1.y = pair.first.y();
        p1.z = pair.first.z();
        p2.x = pair.second.x();
        p2.y = pair.second.y();
        p2.z = pair.second.z();

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    markerPub.publish(marker);
}

/// filter according to frame distance
void LoopFilter_frame(std::vector<std::vector<LOOP_INFO>> &loop_info_list, int &threshold_frame_neighbour,
                      std::ofstream &multi_graph_file) {
    int old_frame_src = 0;
    int old_frame_tar = 0;
    int now_frame_src = 0;
    int now_frame_tar = 0;
    // cycle for curr sequence with each sequence
    for (size_t i = 0; i < loop_info_list.size() - 1; i++) {
        for (size_t j = 0; j < loop_info_list[i].size(); j++) {
            if (j == 0) {
                old_frame_src = loop_info_list[i][j].key_frame_id;
                old_frame_tar = loop_info_list[i][j].match_frame;
                now_frame_src = loop_info_list[i][j].key_frame_id;
                now_frame_tar = loop_info_list[i][j].match_frame;
            } else {
                old_frame_src = now_frame_src;
                old_frame_tar = now_frame_tar;
                now_frame_src = loop_info_list[i][j].key_frame_id;
                now_frame_tar = loop_info_list[i][j].match_frame;
                if (fabs((now_frame_src - old_frame_src) - (now_frame_tar - old_frame_tar)) >=
                    threshold_frame_neighbour) {
                    loop_info_list[i][j].enable = false;
                    multi_graph_file << "loop fail frame neighbour! "
                                     << loop_info_list[i][j].key_frame_id << " -- "
                                     << loop_info_list[i][j].match_frame << std::endl;
                    multi_graph_file << "loop frame dist: "
                                     << fabs((now_frame_src - old_frame_src) - (now_frame_tar - old_frame_tar))
                                     << std::endl;
                    multi_graph_file << "threshold: " << threshold_frame_neighbour << std::endl;
                    multi_graph_file << "[MATCH MAP]: "
                                     << loop_info_list[i][j].data_id
                                     << "--" << loop_info_list[i][j].map_num << std::endl;
                }
            }
        }
    }
}

/// calculate threshold for loop filter
void CalculateThreshold(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered, Eigen::Vector3d &mean_point,
                        double &threshold_neighbour, std::ofstream &multi_graph_file,
                        double &mean, double &covariance) {
    std::vector<double> dist;
    std::vector<Eigen::Vector3d> loop_points;
    double percent = 0.5;
    int iter = 100000;
    double threshold = 1;
    for(int i = 0; i < cloud_filtered->size(); i++) {
        Eigen::Vector3d point(cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].z);
        loop_points.push_back(point);
    }
    meanRansac(loop_points, mean_point, percent, iter, threshold, mean, covariance);
}

/// filter out the outlier match
void LoopFilter(std::vector<std::vector<LOOP_INFO>> &loop_info_list,
                double &threshold_self, double &mean_threshold, double &threshold_neighbor,
                std::ofstream &multi_graph_file, ros::Publisher &markerPub) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> loop_points;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> loop_point;
    // cycle for curr sequence with each sequence
    for (size_t i = 0; i < loop_info_list.size() - 1; i++) {
        // cycle for all loop with map i and curr map
        Eigen::Vector3d mean_point(0, 0, 0);
        pcl::PointCloud<pcl::PointXYZI>::Ptr test_cloud(
                new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
                new pcl::PointCloud<pcl::PointXYZI>);
        for (size_t j = 0; j < loop_info_list[i].size(); j++) {
            Eigen::Vector3d init_point(0, 0, 0);

            Eigen::Vector3d t1 = loop_info_list[i][j].odom_pose_src.first;
            Eigen::Vector3d t2 = loop_info_list[i][j].odom_pose_tar.first;
            Eigen::Matrix3d r1 = loop_info_list[i][j].odom_pose_src.second;
            Eigen::Matrix3d r2 = loop_info_list[i][j].odom_pose_tar.second;
            Eigen::Vector3d tt = loop_info_list[i][j].loop_translation;
            Eigen::Matrix3d rt = loop_info_list[i][j].loop_rotation;
            init_point = -r1 * rt.transpose() * r2.transpose() * t2 - r1 * tt + t1;
            loop_point.first = t1;
            loop_point.second = t2;
            loop_points.push_back(loop_point);

            pcl::PointXYZI pi;
            pi.x = init_point[0];
            pi.y = init_point[1];
            pi.z = init_point[2];
            loop_info_list[i][j].external_point = init_point;
            test_cloud->push_back(pi);
        }
        // save test_cloud to file
        // std::string save_pcd_file = "/home/weihairuo/result/test_cloud" + to_string(i)
        //         + "-" + to_string(loop_info_list[i][0].data_id) + ".pcd";
        // pcl::io::savePCDFileBinary(save_pcd_file, *test_cloud);

        double mean = 0;
        double covariance = 0;
        CalculateThreshold(test_cloud, mean_point, threshold_neighbor, multi_graph_file, mean, covariance);
        threshold_neighbor = mean;
        multi_graph_file << "[Calculate Threshold]" << std::endl;
        multi_graph_file << "mean: " << mean << " covariance: " << covariance << std::endl;
        multi_graph_file << "threshold: " << threshold_neighbor << std::endl;
        // 判断回环是否有效
        for (size_t j = 0; j < loop_info_list[i].size(); j++) {
            loop_info_list[i][j].mean_point = mean_point;
            loop_info_list[i][j].mean = mean;
            loop_info_list[i][j].covariance = threshold_neighbor;
            if ((loop_info_list[i][j].external_point - mean_point).norm() > threshold_neighbor) {
                loop_info_list[i][j].enable = false;
                multi_graph_file << "loop fail neighbour! "
                                 << loop_info_list[i][j].key_frame_id << " -- "
                                 << loop_info_list[i][j].match_frame << std::endl;
                multi_graph_file << "loop dist: " << (loop_info_list[i][j].external_point - mean_point).norm()
                                 << std::endl;
                multi_graph_file << "threshold: " << threshold_neighbor << std::endl;
                multi_graph_file << "[MATCH MAP]: "
                                 << loop_info_list[i][j].data_id
                                 << "--" << loop_info_list[i][j].map_num << std::endl;
            } else {
                loop_info_list[i][j].enable = true;
                multi_graph_file << "loop sucess neighbour! "
                                 << loop_info_list[i][j].key_frame_id << " -- "
                                 << loop_info_list[i][j].match_frame << std::endl;
                multi_graph_file << "loop dist: " << (loop_info_list[i][j].external_point - mean_point).norm()
                                 << std::endl;
                multi_graph_file << "threshold: " << threshold_neighbor << std::endl;
                multi_graph_file << "[MATCH MAP]: "
                                 << loop_info_list[i][j].data_id
                                 << "--" << loop_info_list[i][j].map_num << std::endl;
            }
        }
    }
    //cycle for curr sequence with itself
    for (size_t j = 0; j < loop_info_list[loop_info_list.size() - 1].size(); j++) {
        double loop_dist = loop_info_list[loop_info_list.size() - 1][j].loop_translation.norm();
        double odom_dist = (loop_info_list[loop_info_list.size() - 1][j].odom_pose_src.first -
                            loop_info_list[loop_info_list.size() - 1][j].odom_pose_tar.first).norm();
        double dist = abs(loop_dist - odom_dist);
        loop_info_list[loop_info_list.size() - 1][j].dist = dist;
        if (dist > threshold_self) {
            loop_info_list[loop_info_list.size() - 1][j].enable = false;
            multi_graph_file << "loop fail self! "
                             << loop_info_list[loop_info_list.size() - 1][j].key_frame_id << " -- "
                             << loop_info_list[loop_info_list.size() - 1][j].match_frame << std::endl;
            multi_graph_file << "loop dist: " << loop_dist << std::endl;
            multi_graph_file << "odom dist: " << odom_dist << std::endl;
            multi_graph_file << "dist: " << dist << std::endl;
            multi_graph_file << "[MATCH MAP]: "
                             << loop_info_list[loop_info_list.size() - 1][j].data_id
                             << "--" << loop_info_list[loop_info_list.size() - 1][j].map_num << std::endl;
        } else {
            loop_info_list[loop_info_list.size() - 1][j].enable = true;
            multi_graph_file << "loop sucess self! "
                             << loop_info_list[loop_info_list.size() - 1][j].key_frame_id << " -- "
                             << loop_info_list[loop_info_list.size() - 1][j].match_frame << std::endl;
            multi_graph_file << "loop dist: " << loop_dist << std::endl;
            multi_graph_file << "odom dist: " << odom_dist << std::endl;
            multi_graph_file << "dist: " << dist << std::endl;
            multi_graph_file << "[MATCH MAP]: "
                             << loop_info_list[loop_info_list.size() - 1][j].data_id
                             << "--" << loop_info_list[loop_info_list.size() - 1][j].map_num << std::endl;
        }
    }
    publishLines(loop_points, markerPub);
}