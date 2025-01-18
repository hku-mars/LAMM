#include "include/std_pgo.h"
#include "include/std.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#define debug
// copy from ceres

void BuildOptimizationProblem(const VectorOfConstraints &constraints,
                              MapOfPoses *poses, ceres::Problem *problem) {
  CHECK(poses != nullptr);
  CHECK(problem != nullptr);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction *loss_function = nullptr;

  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;

  for (const auto &constraint : constraints) {
    auto pose_begin_iter = poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    auto pose_end_iter = poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function =
        PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

    problem->AddResidualBlock(cost_function, loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());

    problem->SetManifold(pose_begin_iter->second.q.coeffs().data(),
                         quaternion_manifold);
    problem->SetManifold(pose_end_iter->second.q.coeffs().data(),
                         quaternion_manifold);
  }

  auto pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

void BuildOptimizationProblemOwn(const VectorOfConstraints &constraints,
                                 std::vector<double *> &poses,
                                 ceres::Problem *problem) {
  CHECK(problem != nullptr);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }
  // ceres::LossFunction *loss_function = nullptr;
  ceres::LossFunction *loss_function = new ceres::HuberLoss(10.0);
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;
  Eigen::Vector3d residual_p(0, 0, 0);
  Eigen::Vector3d residual_q(0, 0, 0);
  for (const auto &constraint : constraints) {
    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function = PoseGraph3dErrorTermOwn::Create(
        constraint.t_be.p, constraint.t_be.q, sqrt_information);

    ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;
    problem->AddParameterBlock(poses[constraint.id_begin], 3);
    problem->AddParameterBlock(poses[constraint.id_begin] + 3, 4,
                               quaternion_manifold);
    problem->AddParameterBlock(poses[constraint.id_end], 3);
    problem->AddParameterBlock(poses[constraint.id_end] + 3, 4,
                               quaternion_manifold);
    problem->AddResidualBlock(
        cost_function, loss_function, poses[constraint.id_begin],
        poses[constraint.id_begin] + 3, poses[constraint.id_end],
        poses[constraint.id_end] + 3);

    Eigen::Vector3d p_a(poses[constraint.id_begin][0],
                        poses[constraint.id_begin][1],
                        poses[constraint.id_begin][2]);
    Eigen::Quaterniond q_a(
        poses[constraint.id_begin][6], poses[constraint.id_begin][3],
        poses[constraint.id_begin][4], poses[constraint.id_begin][5]);
    Eigen::Vector3d p_b(poses[constraint.id_end][0],
                        poses[constraint.id_end][1],
                        poses[constraint.id_end][2]);
    Eigen::Quaterniond q_b(
        poses[constraint.id_end][6], poses[constraint.id_end][3],
        poses[constraint.id_end][4], poses[constraint.id_end][5]);
    Eigen::Vector3d p_ab_estimated = q_a.conjugate() * (p_b - p_a);
    Eigen::Vector3d p_ref = constraint.t_be.p;
    residual_p = (p_ab_estimated - p_ref);
    Eigen::Quaterniond q_ab_estimated = q_a.conjugate() * q_b;
    Eigen::Quaterniond delta_q = constraint.t_be.q * q_ab_estimated.conjugate();
    residual_q = (2.0) * delta_q.vec();
    // std::cout << "build residual, p_ab_estimated:" <<
    // p_ab_estimated.transpose()
    //           << " residual_p:" << residual_p.transpose()
    //           << ", residual_q:" << residual_q.transpose() << std::endl;
    // problem->SetManifold(poses[constraint.id_begin] + 3,
    // quaternion_manifold); problem->SetManifold(poses[constraint.id_end] + 3,
    // quaternion_manifold);
  }
  // problem->SetParameterBlockConstant(poses[0]);
  // problem->SetParameterBlockConstant(poses[0] + 3);
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem *problem) {
  CHECK(problem != nullptr);

  ceres::Solver::Options options;
  // options.max_num_iterations = 1000;
  // options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

int findPoseIndexUsingTime(std::vector<double> &time_list, long &time) {
  long time_inc = 10000000000;
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
  std::string bag_file = "";
  std::string pose_file = "";
  std::string refine_pose_file = "";
  std::string loop_gt_file = "";
  double icp_threshold = 0.5;
  bool calc_gt_enable = false;
  nh.param<std::string>("data_name", data_name, "");
  nh.param<std::string>("setting_path", setting_path, "");
  nh.param<std::string>("bag_file", bag_file, "");
  nh.param<std::string>("pose_file", pose_file, "");
  nh.param<std::string>("loop_gt_file", loop_gt_file, "");
  nh.param<bool>("calc_gt_enable", calc_gt_enable, false);
  nh.param<double>("icp_threshold", icp_threshold, 0.5);
  std::string icp_string = std::to_string(icp_threshold);
  std::string result_path =
      "/home/ycj/matlab_code/loop_detection/result/" + data_name + "/" +
      data_name + "_" + icp_string.substr(0, icp_string.find(".") + 3) + ".txt";
  std::ofstream result_file(result_path);
  std::ofstream debug_file("/home/ycj/catkin_ws/src/STD/Log/log.txt");
  std::ofstream debug_augment("/home/ycj/catkin_ws/src/STD/Log/augument.txt");

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);

  ros::Publisher pubOdomCorreted =
      nh.advertise<nav_msgs::Odometry>("/odom_corrected", 10);
  ros::Publisher pubRegisterCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  ros::Publisher pubCureentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  ros::Publisher pubCorrectCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_correct", 10000);
  ros::Publisher pubCorrectCloud2 =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_correct2", 10000);

  ros::Rate loop(50000);

  ConfigSetting config_setting;
  load_config_setting(setting_path, config_setting);

  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
  std::vector<double> time_list;
  if (config_setting.gt_file_style_ == 0) {
    load_pose_with_time(pose_file, pose_list, time_list);
  } else if (config_setting.gt_file_style_ == 1) {
    load_cu_pose_with_time(pose_file, pose_list, time_list);
  }

  std::string print_msg = "Sucessfully load pose file:" + pose_file +
                          ". pose size:" + std::to_string(time_list.size());
  ROS_INFO_STREAM(print_msg.c_str());

  // save all point clouds of key frame
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_list;

  // save all planes of key frame
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> history_plane_list;

  // save all binary descriptors of key frame
  std::vector<std::vector<BinaryDescriptor>> history_binary_list;

  // save all STD descriptors of key frame
  std::vector<std::vector<STD>> history_STD_list;

  // save all poses(translation, rotation) of all frame
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> normal_pose_list;

  // save all poses(translation only) of key frame
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> key_pose_list;

  // save all optimized poses
  std::vector<double *> opt_pose_list;

  MapOfPoses poses;
  VectorOfConstraints constraints;

  // hash table, save all descriptor
  std::unordered_map<STD_LOC, std::vector<STD>> STD_map;

  // calc mean time
  double mean_time = 0;

  // record mean position
  Eigen::Vector3d current_mean_position(0, 0, 0);

  long current_time = 0;

  // load lidar point cloud and start loop
  pcl::PointCloud<pcl::PointXYZI>::Ptr pose_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_key_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  bool is_build_descriptor = false;
  int key_frame_id = 0;
  std::fstream file_;
  file_.open(bag_file, std::ios::in);
  if (!file_) {
    std::cout << "File " << bag_file << " does not exit" << std::endl;
  }
  ROS_INFO("Start to load the rosbag %s", bag_file.c_str());
  rosbag::Bag bag;
  try {
    bag.open(bag_file, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
  }
  std::vector<std::string> types;
  types.push_back(std::string("sensor_msgs/PointCloud2"));
  rosbag::View view(bag, rosbag::TypeQuery(types));
  bool is_init_bag = false;
  bool is_skip_frame = false;
  Eigen::Vector3d init_translation;
  Eigen::Vector3d last_translation;
  Eigen::Quaterniond last_q;
  int count = 0;
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    sensor_msgs::PointCloud2::ConstPtr cloud_ptr =
        m.instantiate<sensor_msgs::PointCloud2>();
    if (cloud_ptr != NULL) {
      if (count == 0) {
        if (!is_skip_frame) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
              new pcl::PointCloud<pcl::PointXYZI>);
          key_cloud_list.push_back(temp_cloud);
          current_key_cloud->clear();
        }
      }
      long laser_time = cloud_ptr->header.stamp.toNSec();
      pcl::PCLPointCloud2 pcl_pc;
      pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
      pcl_conversions::toPCL(*cloud_ptr, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, pcl_cloud);
      int pose_index = findPoseIndexUsingTime(time_list, laser_time);
      if (pose_index < 0) {
        is_skip_frame = true;
        continue;
      }
      is_skip_frame = false;
      Eigen::Vector3d translation = pose_list[pose_index].first;
      Eigen::Matrix3d rotation = pose_list[pose_index].second;
      Eigen::Quaterniond q(rotation);

      if (!is_init_bag) {
        init_translation = translation;
        translation << 0, 0, 0;
        is_init_bag = true;
        last_translation = translation;
        last_q = q;
      } else {
        translation = translation - init_translation;
      }

      if (config_setting.stop_skip_enable_) {
        Eigen::Vector3d position_inc;
        position_inc = translation - last_translation;
        double rotation_inc = q.angularDistance(last_q);
        if (position_inc.norm() < 0.2 && rotation_inc < DEG2RAD(5)) {
          continue;
        }
        last_translation = translation;
        last_q = q;
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
      loop.sleep();

      pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      for (size_t i = 0; i < pcl_cloud.size(); i++) {
        Eigen::Vector3d pv(pcl_cloud.points[i].x, pcl_cloud.points[i].y,
                           pcl_cloud.points[i].z);
        pv = config_setting.rot_lidar_to_vehicle_ * pv +
             config_setting.t_lidar_to_vehicle_;
        pv = rotation * pv + translation;
        pcl::PointXYZI pi = pcl_cloud.points[i];
        pi.x = pv[0];
        pi.y = pv[1];
        pi.z = pv[2];
        register_cloud->push_back(pi);
      }
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
      single_pose.first = translation;
      single_pose.second = rotation;
      normal_pose_list.push_back(single_pose);
      Eigen::Quaterniond quaternion(rotation);

      if (count == 0) {
        if (!is_skip_frame) {
          key_pose_list.push_back(single_pose);
          double *pose = new double[7];
          pose[0] = translation[0];
          pose[1] = translation[1];
          pose[2] = translation[2];
          pose[3] = quaternion.x();
          pose[4] = quaternion.y();
          pose[5] = quaternion.z();
          pose[6] = quaternion.w();
          opt_pose_list.push_back(pose);
        }
      }

      time_t ds_start = clock();
      down_sampling_voxel(*register_cloud, config_setting.ds_size_);
      for (size_t i = 0; i < register_cloud->size(); i++) {
        current_key_cloud->points.push_back(register_cloud->points[i]);
      }
      if (count == config_setting.sub_frame_num_ / 2) {
        current_time = cloud_ptr->header.stamp.toNSec() / 1000;
        current_mean_position = translation;
        pcl::PointXYZI pi;
        pi.x = current_mean_position[0];
        pi.y = current_mean_position[1];
        pi.z = current_mean_position[2];
        // pose_cloud->points.push_back(pi);
      }
      if (count < config_setting.sub_frame_num_ - 1) {
        count++;
      } else {
        count = 0;
        is_build_descriptor = true;
      }
    }
    if (is_build_descriptor) {
      std::cout << std::endl;
      std::cout << "Key Frame:" << key_frame_id
                << ", cloud size:" << current_key_cloud->size() << std::endl;
      debug_file << std::endl;
      debug_file << "Key frame:" << key_frame_id
                 << ", cloud size:" << current_key_cloud->size() << std::endl;
      if (config_setting.is_kitti_) {
        result_file << key_frame_id << "," << current_mean_position[0] << ","
                    << current_mean_position[1] << ","
                    << current_mean_position[2] << ",";
      } else {
        result_file << key_frame_id << "," << current_time << ","
                    << current_mean_position[0] << ","
                    << current_mean_position[1] << ","
                    << current_mean_position[2] << ",";
      }

      auto t1 = std::chrono::high_resolution_clock::now();
      auto t_build_descriptor_begin = std::chrono::high_resolution_clock::now();
      std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
      init_voxel_map(config_setting, *current_key_cloud, voxel_map);
      auto t1_end = std::chrono::high_resolution_clock::now();
      std::cout << "init voxel map time:" << time_inc(t1_end, t1) << std::endl;
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr frame_plane_cloud(
          new pcl::PointCloud<pcl::PointXYZINormal>);
      get_plane(voxel_map, frame_plane_cloud);
      history_plane_list.push_back(frame_plane_cloud);
      std::vector<Plane *> proj_plane_list;
      std::vector<Plane *> merge_plane_list;
      get_project_plane(config_setting, voxel_map, proj_plane_list);
      sort(proj_plane_list.begin(), proj_plane_list.end(), plane_greater_sort);
      merge_plane(config_setting, proj_plane_list, merge_plane_list);
      sort(merge_plane_list.begin(), merge_plane_list.end(),
           plane_greater_sort);

      std::vector<BinaryDescriptor> binary_list;
      std::vector<BinaryDescriptor> binary_around_list;
      auto t2 = std::chrono::high_resolution_clock::now();
      binary_extractor(config_setting, merge_plane_list, current_key_cloud,
                       binary_list);
      auto t2_end = std::chrono::high_resolution_clock::now();
      std::vector<STD> STD_list;

      auto t_g_std = std::chrono::high_resolution_clock::now();
      generate_std(config_setting, binary_list, key_frame_id, STD_list);
      auto t_g_std_end = std::chrono::high_resolution_clock::now();
      auto t_build_descriptor_end = std::chrono::high_resolution_clock::now();

      std::cout << "extract binary time:" << time_inc(t2_end, t2) << std::endl;
      std::cout << "build std time:" << time_inc(t_g_std_end, t_g_std)
                << std::endl;
      // history_STD_list.push_back(STD_list);
      Eigen::Vector3d color2(0, 1, 0);
      publish_binary(binary_around_list, color2, "history", pubSTD);

      history_binary_list.push_back(binary_list);

      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(*current_key_cloud, pub_cloud);

      pub_cloud.header.frame_id = "camera_init";
      pubCureentCloud.publish(pub_cloud);
      loop.sleep();
      pcl::PointCloud<pcl::PointXYZ> key_points_cloud;
      for (auto var : binary_list) {
        pcl::PointXYZ pi;
        pi.x = var.location_[0];
        pi.y = var.location_[1];
        pi.z = var.location_[2];
        key_points_cloud.push_back(pi);
      }
      pcl::toROSMsg(key_points_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCurrentBinary.publish(pub_cloud);
      loop.sleep();
      Eigen::Vector3d color1(1, 0, 0);
      publish_binary(binary_list, color1, "current", pubSTD);
      pcl::PointCloud<pcl::PointXYZ> around_key_points_cloud;
      for (auto var : binary_around_list) {
        pcl::PointXYZ pi;
        pi.x = var.location_[0];
        pi.y = var.location_[1];
        pi.z = var.location_[2];
        around_key_points_cloud.push_back(pi);
      }
      pcl::toROSMsg(around_key_points_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubMatchedBinary.publish(pub_cloud);

      // debug for binary and std augument
      double dis_threshold = 1.0;
      int binary_augument_num = 0;
      if (key_frame_id >= 1) {
        for (size_t i = 0; i < history_binary_list[key_frame_id].size(); i++) {
          BinaryDescriptor binary1 = history_binary_list[key_frame_id][i];
          for (size_t j = 0; j < history_binary_list[key_frame_id - 1].size();
               j++) {
            BinaryDescriptor binary2 = history_binary_list[key_frame_id - 1][j];
            if ((binary1.location_ - binary2.location_).norm() <
                dis_threshold) {
              binary_augument_num++;
              break;
            }
          }
        }
      }

      debug_file << "[Corner] corner size:" << binary_list.size()
                 << "  descriptor size:" << STD_list.size() << std::endl;
      // candidate search
      auto t_candidate_search_begin = std::chrono::high_resolution_clock::now();
      std::vector<STDMatchList> alternative_match;
      candidate_searcher_old(config_setting, STD_map, STD_list,
                             alternative_match);
      auto t_candidate_search_end = std::chrono::high_resolution_clock::now();

      // geometrical verification
      auto t_fine_loop_begin = std::chrono::high_resolution_clock::now();
      bool triggle_loop = false;
      Eigen::Vector3d best_t;
      Eigen::Matrix3d best_rot;
      Eigen::Vector3d loop_translation;
      Eigen::Matrix3d loop_rotation;
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
      int match_frame = 0;
      double best_score = 0;
      double best_icp_score = 0;
      int best_frame = -1;
      for (int i = 0; i < alternative_match.size(); i++) {
        if (alternative_match[i].match_list_.size() >= 4) {
          bool fine_sucess = false;
          Eigen::Matrix3d std_rot;
          Eigen::Vector3d std_t;
#ifdef debug
          debug_file << "[Rough match] rough match frame:"
                     << alternative_match[i].match_frame_ << " match size:"
                     << alternative_match[i].match_list_.size() << std::endl;
#endif
          sucess_match_list.clear();
          fine_loop_detection_tbb(
              config_setting, alternative_match[i].match_list_, fine_sucess,
              std_rot, std_t, sucess_match_list, unsucess_match_list);
          // fine_loop_detection(config_setting,
          // alternative_match[i].match_list_,
          //                     fine_sucess, std_rot, std_t, sucess_match_list,
          //                     unsucess_match_list);
          if (fine_sucess) {
            double icp_score = geometric_verify(
                config_setting, frame_plane_cloud,
                history_plane_list[alternative_match[i].match_frame_], std_rot,
                std_t);
            double score = icp_score + sucess_match_list.size() * 1.0 / 1000;
#ifdef debug
            debug_file << "Fine sucess, Fine size:" << sucess_match_list.size()
                       << "  ,Icp score:" << icp_score << ", score:" << score
                       << std::endl;
#endif
            if (score > best_score) {
              if (std_t.norm() > 25) {
                continue;
              }
              unsucess_match_list_publish = unsucess_match_list;
              sucess_match_list_publish = sucess_match_list;
              best_frame = alternative_match[i].match_frame_;
              best_score = score;
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
      if (best_icp_score > icp_threshold) {
        loop_translation = best_t;
        loop_rotation = best_rot;
        match_frame = best_frame;
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

      if (triggle_loop) {
#ifdef debug
        debug_file << "[Loop Sucess] " << key_frame_id << "--" << match_frame
                   << ", candidate id:" << candidate_id
                   << ", icp:" << best_score << std::endl;
        debug_file << "[Loop Info] "
                   << "rough size:" << rough_size
                   << ", match size:" << match_size
                   << ", rough triangle dis:" << outlier_mean_triangle_dis
                   << ", fine triangle dis:" << mean_triangle_dis
                   << ", rough binary similarity:" << outlier_mean_triangle_dis
                   << ", fine binary similarity:" << mean_binary_similarity
                   << std::endl;
#endif
        result_file << 1 << "," << match_frame << "," << candidate_id << ","
                    << match_size << "," << rough_size << ","
                    << loop_translation[0] << "," << loop_translation[1] << ","
                    << loop_translation[2] << ",";

        pcl::toROSMsg(*key_cloud_list[match_frame], pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCloud.publish(pub_cloud);
        loop.sleep();
        pcl::PointCloud<pcl::PointXYZ> matched_key_points_cloud;
        for (auto var : history_binary_list[match_frame]) {
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
        publish_binary(history_binary_list[match_frame], color2, "history",
                       pubSTD);
        loop.sleep();
        publish_std(sucess_match_list_publish, pubSTD);
      } else {
        debug_file << "[Loop Fail] " << key_frame_id << ", icp:" << best_score
                   << std::endl;
      }
      auto t_add_descriptor_begin = std::chrono::high_resolution_clock::now();
      add_STD(STD_map, STD_list);
      auto t_add_descriptor_end = std::chrono::high_resolution_clock::now();
      mean_time += time_inc(t_build_descriptor_end, t_build_descriptor_begin) +
                   time_inc(t_candidate_search_end, t_candidate_search_begin) +
                   time_inc(t_fine_loop_end, t_fine_loop_begin) +
                   time_inc(t_add_descriptor_end, t_add_descriptor_begin);
      debug_file << "[Time] build_descriptor:"
                 << time_inc(t_build_descriptor_end, t_build_descriptor_begin)
                 << ", candidate search:"
                 << time_inc(t_candidate_search_end, t_candidate_search_begin)
                 << ", fine loop detect:"
                 << time_inc(t_fine_loop_end, t_fine_loop_begin)
                 << ", add descriptor:"
                 << time_inc(t_add_descriptor_end, t_add_descriptor_begin)
                 << ", average:" << mean_time / (key_frame_id + 1) << std::endl;
      if (triggle_loop) {
        // debug for binary and std augument
        double dis_threshold = 1.0;
        int binary_augument_num = 0;
        if (key_frame_id >= 1) {
          for (size_t i = 0; i < history_binary_list[key_frame_id].size();
               i++) {
            BinaryDescriptor binary1 = history_binary_list[key_frame_id][i];
            for (size_t j = 0; j < history_binary_list[match_frame].size();
                 j++) {
              BinaryDescriptor binary2 = history_binary_list[match_frame][j];
              if ((binary1.location_ - binary2.location_).norm() <
                  dis_threshold) {
                binary_augument_num++;
                break;
              }
            }
          }
        }

        std::cout << "Binary size:" << history_binary_list[key_frame_id].size()
                  << ", augument size:" << binary_augument_num
                  << ", augument rate:"
                  << binary_augument_num * 1.0 /
                         history_binary_list[key_frame_id].size()
                  << std::endl;
        result_file << history_binary_list[key_frame_id].size() << ","
                    << binary_augument_num << "," << best_score << std::endl;

        int std_augument_num = 0;
        double mean_dis = 0;
        double mean_similarity = 0;
        mean_dis = mean_dis / std_augument_num;
        mean_similarity = mean_similarity / std_augument_num;
        std::cout << "STD size:" << STD_list.size()
                  << ", augument size:" << std_augument_num
                  << ", augument rate:"
                  << std_augument_num * 1.0 / STD_list.size()
                  << ", mean dis:" << mean_dis
                  << ", mean similarity:" << mean_similarity << std::endl;

        // geometric_icp(frame_plane_cloud, history_plane_list[match_frame],
        //               loop_rotation, loop_translation);
        // add current_pose
        Pose3d current_pose;
        Eigen::Quaterniond quaternion1(key_pose_list.back().second);
        current_pose.p = key_pose_list.back().first;
        current_pose.q = quaternion1;
        (poses)[key_frame_id] = current_pose;
        // add loop constraint
        if (1) {
          // test
          // auto Q_a = key_pose_list[match_frame].second;
          // auto Q_b = key_pose_list[key_frame_id].second;
          // auto T_a = key_pose_list[match_frame].first;
          // auto T_b = key_pose_list[key_frame_id].first;
          // Eigen::Quaterniond ICP_q(loop_rotation);
          // auto ICP_t = loop_translation;
          // // ICP_t = (ICP_q.inverse() * (-ICP_t));
          // // ICP_q = ICP_q.inverse();

          // Constraint3d pose_constrain;
          // auto q_res = Q_b.inverse() * ICP_q.inverse() * Q_a;
          // // q_res = q_res.inverse();
          // auto t_res = Q_b.inverse() * (ICP_q.inverse() * (T_a - ICP_t) -
          // T_b); pose_constrain.id_begin = key_frame_id; pose_constrain.id_end
          // = match_frame; pose_constrain.t_be.p = t_res; pose_constrain.t_be.q
          // = q_res; constraints.push_back(pose_constrain);

          Constraint3d constraint;
          constraint.id_begin = match_frame;
          constraint.id_end = key_frame_id;

          Eigen::Quaterniond q_a(key_pose_list[constraint.id_begin].second);
          Eigen::Vector3d t_a = key_pose_list[constraint.id_begin].first;
          Eigen::Quaterniond q_b(key_pose_list[constraint.id_end].second);
          Eigen::Vector3d t_b = key_pose_list[constraint.id_end].first;

          // for debug
          // loop_rotation = Eigen::Matrix3d::Identity();
          // loop_translation << 0, 0, 0;
          Eigen::Quaterniond icp_q(loop_rotation);
          Eigen::Vector3d icp_t(loop_translation);
          // icp_t = (icp_q.inverse() * (-icp_t));
          // icp_q = icp_q.inverse();

          std::cout << "add loop constraint, icp_t:" << icp_t.transpose()
                    << std::endl
                    << ", icp_rot:" << loop_rotation << std::endl;

          auto q_res = q_a.inverse() * icp_q * q_b;
          auto t_res = q_a.inverse() * (icp_q * t_b + icp_t - t_a);
          // auto q_res = q_a.inverse() * icp_q * q_b;
          // auto t_res = q_a.inverse() * (icp_q * t_b + icp_t - t_a);
          Eigen::Quaterniond q_ab_estimated = q_a.inverse() * q_b;
          Eigen::Vector3d p_ab_estimated = q_a.inverse() * (t_b - t_a);
          std::cout << "p_ab_estimated: " << p_ab_estimated.transpose()
                    << std::endl;
          std::cout << "t_res: " << t_res.transpose() << std::endl;

          std::cout << "residual p: " << (p_ab_estimated - t_res).transpose()
                    << std::endl;
          Eigen::Quaterniond delta_q = q_res * q_ab_estimated.inverse();
          std::cout << "residual q:" << (2.0) * delta_q.vec().transpose()
                    << std::endl;
          // test cloud
          // pcl::PointCloud<pcl::PointXYZI> cloud_a;
          // pcl::PointCloud<pcl::PointXYZI> cloud_b;
          // std::cout << "pose a:"
          //           << key_pose_list[constraint.id_begin].first.transpose()
          //           << std::endl;
          // std::cout << key_pose_list[constraint.id_begin].second <<
          // std::endl; for (size_t i = 0; i <
          // key_cloud_list[constraint.id_begin]->size();
          //      i++) {
          //   pcl::PointXYZI pi =
          //   key_cloud_list[constraint.id_begin]->points[i]; Eigen::Vector3d
          //   pv(pi.x, pi.y, pi.z); pv = q_a.inverse() * (pv - t_a); pi.x =
          //   pv[0]; pi.y = pv[1]; pi.z = pv[2]; cloud_a.push_back(pi);
          // }
          // for (size_t i = 0; i < current_key_cloud->size(); i++) {
          //   pcl::PointXYZI pi = current_key_cloud->points[i];
          //   Eigen::Vector3d pv(pi.x, pi.y, pi.z);
          //   pv = q_b.inverse() * (pv - t_b);
          //   pv = q_ab_estimated * pv + p_ab_estimated;
          //   pi.x = pv[0];
          //   pi.y = pv[1];
          //   pi.z = pv[2];
          //   cloud_b.push_back(pi);
          // }
          // std::cout << "cloud a size:" << cloud_a.size()
          //           << ", cloud b size:" << cloud_b.size() << std::endl;
          // pcl::toROSMsg(cloud_a, pub_cloud);
          // pub_cloud.header.frame_id = "camera_init";
          // pubCureentCloud.publish(pub_cloud);
          // loop.sleep();
          // pcl::toROSMsg(cloud_b, pub_cloud);
          // pub_cloud.header.frame_id = "camera_init";
          // pubMatchedCloud.publish(pub_cloud);
          // loop.sleep();
          // getchar();

          constraint.t_be.p = t_res;
          constraint.t_be.q = q_res;
          constraint.information.setIdentity();
          // constraint.information =
          //     100 * Eigen::Matrix<double, 6, 6>::Identity();
          constraints.push_back(constraint);
        }

        // add near constraint
        if (1) {
          // add near constraint
          Constraint3d constraint_near;
          constraint_near.id_begin = key_frame_id - 1;
          constraint_near.id_end = key_frame_id;
          Eigen::Vector3d p_a = key_pose_list[constraint_near.id_begin].first;
          Eigen::Quaterniond q_a(
              key_pose_list[constraint_near.id_begin].second);
          Eigen::Vector3d p_b = key_pose_list[constraint_near.id_end].first;
          Eigen::Quaterniond q_b(key_pose_list[constraint_near.id_end].second);

          constraint_near.t_be.p = q_a.inverse() * (p_b - p_a);
          constraint_near.t_be.q = q_a.inverse() * q_b;
          constraint_near.information.setIdentity();
          // constraint_near.information = Eigen::Matrix<double, 6,
          // 6>::Identity();
          constraints.push_back(constraint_near);
        }

      } else {
        // add current_pose
        Eigen::Vector3d translation1 = key_pose_list.back().first;
        Eigen::Matrix3d rotation1 = key_pose_list.back().second;
        Eigen::Quaterniond quaternion1(rotation1);
        Pose3d current_pose;
        current_pose.p = translation1;
        current_pose.q = quaternion1;
        (poses)[key_frame_id] = current_pose;
        // add near connection
        if (opt_pose_list.size() >= 2) {
          // add near constraint
          if (1) {
            // add near constraint
            Constraint3d constraint_near;
            constraint_near.id_begin = key_frame_id - 1;
            constraint_near.id_end = key_frame_id;
            Eigen::Vector3d p_a = key_pose_list[constraint_near.id_begin].first;
            Eigen::Quaterniond q_a(
                key_pose_list[constraint_near.id_begin].second);
            Eigen::Vector3d p_b = key_pose_list[constraint_near.id_end].first;
            Eigen::Quaterniond q_b(
                key_pose_list[constraint_near.id_end].second);
            // The measurement for the position of B relative to A in the A
            // frame.
            constraint_near.t_be.p = q_a.inverse() * (p_b - p_a);
            constraint_near.t_be.q = q_a.inverse() * q_b;
            constraint_near.information.setIdentity();
            // constraint_near.information =
            //     Eigen::Matrix<double, 6, 6>::Identity();
            constraints.push_back(constraint_near);
          }
        }

        result_file << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << ","
                    << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << ","
                    << 0 << std::endl;
        // getchar();
      }
      // down_sampling_voxel(*current_key_cloud, 0.25);
      for (size_t i = 0; i < current_key_cloud->size(); i++) {
        key_cloud_list.back()->push_back(current_key_cloud->points[i]);
      }
      key_frame_id++;
      for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
        delete (iter->second);
      }
    }
  }

  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> origin_pose_list;
  for (size_t i = 0; i < opt_pose_list.size(); i++) {
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
    Eigen::Vector3d translation(opt_pose_list[i][0], opt_pose_list[i][1],
                                opt_pose_list[i][2]);
    Eigen::Quaterniond quaternion(opt_pose_list[i][6], opt_pose_list[i][3],
                                  opt_pose_list[i][4], opt_pose_list[i][5]);
    single_pose.first = translation;
    single_pose.second = quaternion.toRotationMatrix();
    origin_pose_list.push_back(single_pose);
  }
  auto t_ba_begin = std::chrono::high_resolution_clock::now();
  ceres::Problem problem;
  // BuildOptimizationProblem(constraints, &poses, &problem);
  BuildOptimizationProblemOwn(constraints, opt_pose_list, &problem);
  Eigen::Vector3d residual_p(0, 0, 0);
  Eigen::Vector3d residual_q(0, 0, 0);
  for (size_t i = 0; i < constraints.size(); i++) {
    // Eigen::Vector3d p_a = poses[constraints[i].id_begin].p;
    // Eigen::Quaterniond q_a = poses[constraints[i].id_begin].q;
    // Eigen::Vector3d p_b = poses[constraints[i].id_end].p;
    // Eigen::Quaterniond q_b = poses[constraints[i].id_end].q;
    Eigen::Vector3d p_a(opt_pose_list[constraints[i].id_begin][0],
                        opt_pose_list[constraints[i].id_begin][1],
                        opt_pose_list[constraints[i].id_begin][2]);
    Eigen::Quaterniond q_a(opt_pose_list[constraints[i].id_begin][6],
                           opt_pose_list[constraints[i].id_begin][3],
                           opt_pose_list[constraints[i].id_begin][4],
                           opt_pose_list[constraints[i].id_begin][5]);
    Eigen::Vector3d p_b(opt_pose_list[constraints[i].id_end][0],
                        opt_pose_list[constraints[i].id_end][1],
                        opt_pose_list[constraints[i].id_end][2]);
    Eigen::Quaterniond q_b(opt_pose_list[constraints[i].id_end][6],
                           opt_pose_list[constraints[i].id_end][3],
                           opt_pose_list[constraints[i].id_end][4],
                           opt_pose_list[constraints[i].id_end][5]);
    Eigen::Vector3d p_ab_estimated = q_a.conjugate() * (p_b - p_a);
    Eigen::Vector3d p_ref = constraints[i].t_be.p;
    residual_p[0] += fabs((p_ab_estimated - p_ref)[0]);
    residual_p[1] += fabs((p_ab_estimated - p_ref)[1]);
    residual_p[2] += fabs((p_ab_estimated - p_ref)[2]);
    Eigen::Quaterniond q_ab_estimated = q_a.conjugate() * q_b;
    Eigen::Quaterniond delta_q =
        constraints[i].t_be.q * q_ab_estimated.conjugate();
    residual_q[0] += fabs((2.0) * delta_q.vec()[0]);
    residual_q[1] += fabs((2.0) * delta_q.vec()[1]);
    residual_q[2] += fabs((2.0) * delta_q.vec()[2]);
  }
  std::cout << "Residual p before opt: " << residual_p.transpose() << std::endl;
  std::cout << "Residual q before opt: " << residual_q.transpose() << std::endl;
  SolveOptimizationProblem(&problem);
  residual_p << 0, 0, 0;
  residual_q << 0, 0, 0;
  for (size_t i = 0; i < constraints.size(); i++) {
    Eigen::Vector3d p_a(opt_pose_list[constraints[i].id_begin][0],
                        opt_pose_list[constraints[i].id_begin][1],
                        opt_pose_list[constraints[i].id_begin][2]);
    Eigen::Quaterniond q_a(opt_pose_list[constraints[i].id_begin][6],
                           opt_pose_list[constraints[i].id_begin][3],
                           opt_pose_list[constraints[i].id_begin][4],
                           opt_pose_list[constraints[i].id_begin][5]);
    Eigen::Vector3d p_b(opt_pose_list[constraints[i].id_end][0],
                        opt_pose_list[constraints[i].id_end][1],
                        opt_pose_list[constraints[i].id_end][2]);
    Eigen::Quaterniond q_b(opt_pose_list[constraints[i].id_end][6],
                           opt_pose_list[constraints[i].id_end][3],
                           opt_pose_list[constraints[i].id_end][4],
                           opt_pose_list[constraints[i].id_end][5]);
    Eigen::Vector3d p_ab_estimated = q_a.conjugate() * (p_b - p_a);
    Eigen::Vector3d p_ref = constraints[i].t_be.p;
    residual_p[0] += fabs((p_ab_estimated - p_ref)[0]);
    residual_p[1] += fabs((p_ab_estimated - p_ref)[1]);
    residual_p[2] += fabs((p_ab_estimated - p_ref)[2]);
    Eigen::Quaterniond q_ab_estimated = q_a.conjugate() * q_b;
    Eigen::Quaterniond delta_q =
        constraints[i].t_be.q * q_ab_estimated.conjugate();
    residual_q[0] += fabs((2.0) * delta_q.vec()[0]);
    residual_q[1] += fabs((2.0) * delta_q.vec()[1]);
    residual_q[2] += fabs((2.0) * delta_q.vec()[2]);
  }
  std::cout << "Residual p after opt: " << residual_p.transpose() << std::endl;
  std::cout << "Residual q after opt: " << residual_q.transpose() << std::endl;
  auto t_ba_end = std::chrono::high_resolution_clock::now();
  std::cout << "Final pga time cost: "
            << time_inc(t_ba_end, t_ba_begin) / 1000.0 << " s" << std::endl;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> correct_cloud_list;
  for (size_t i = 0; i < opt_pose_list.size(); i++) {
    pcl::PointCloud<pcl::PointXYZI> correct_cloud;

    Eigen::Vector3d opt_translation(opt_pose_list[i][0], opt_pose_list[i][1],
                                    opt_pose_list[i][2]);
    Eigen::Quaterniond opt_q(opt_pose_list[i][6], opt_pose_list[i][3],
                             opt_pose_list[i][4], opt_pose_list[i][5]);
    for (size_t j = 0; j < key_cloud_list[i]->size(); j++) {
      pcl::PointXYZI pi = key_cloud_list[i]->points[j];
      Eigen::Vector3d pv(pi.x, pi.y, pi.z);
      // back projection
      pv = origin_pose_list[i].second.transpose() * pv -
           origin_pose_list[i].second.transpose() * origin_pose_list[i].first;
      // re-projection
      pv = opt_q * pv + opt_translation;
      pi.x = pv[0];
      pi.y = pv[1];
      pi.z = pv[2];
      correct_cloud.push_back(pi);
    }
    pcl::PointXYZI pose_point;
    pose_point.x = opt_translation[0];
    pose_point.y = opt_translation[1];
    pose_point.z = opt_translation[2];
    pose_cloud->push_back(pose_point);

    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(correct_cloud, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubCorrectCloud.publish(pub_cloud);
    loop.sleep();
    down_sampling_voxel(correct_cloud, 0.25);
    correct_cloud_list.push_back(correct_cloud.makeShared());

    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.pose.pose.position.x = opt_translation[0];
    odom.pose.pose.position.y = opt_translation[1];
    odom.pose.pose.position.z = opt_translation[2];
    odom.pose.pose.orientation.w = opt_q.w();
    odom.pose.pose.orientation.x = opt_q.x();
    odom.pose.pose.orientation.y = opt_q.y();
    odom.pose.pose.orientation.z = opt_q.z();
    pubOdomCorreted.publish(odom);
    loop.sleep();
  }

  if (calc_gt_enable) {
    std::ofstream gt_file(loop_gt_file);
    std::cout << "calc gt for loop!" << std::endl;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_tree(
        new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kd_tree->setInputCloud(pose_cloud);
    std::vector<int> indices;
    std::vector<float> distances;
    double radius = 30;
    double overlap_threshold = 0.5;
    int gt_loop_num = 0;
    for (int i = 0; i < pose_cloud->size(); i++) {
      double max_overlap = 0;
      bool trigger_loop = false;
      int loop_id = 0;
      pcl::PointXYZI searchPoint = pose_cloud->points[i];
      int size = kd_tree->radiusSearch(searchPoint, radius, indices, distances);
      for (int j = 0; j < size; j++) {
        if (indices[j] >= i - 50) {
          continue;
        } else {
          pcl::PointCloud<pcl::PointXYZI> ds_cloud1 = *correct_cloud_list[i];
          pcl::PointCloud<pcl::PointXYZI> ds_cloud2 =
              *correct_cloud_list[indices[j]];
          double overlap = calc_overlap(ds_cloud1.makeShared(),
                                        ds_cloud2.makeShared(), 0.25);
          if (overlap > max_overlap) {
            max_overlap = overlap;
            loop_id = indices[j];
          }
        }
      }
      if (max_overlap > overlap_threshold) {
        trigger_loop = true;
        gt_loop_num++;
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(*correct_cloud_list[i], pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCureentCloud.publish(pub_cloud);
        loop.sleep();
        pcl::toROSMsg(*correct_cloud_list[loop_id], pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCloud.publish(pub_cloud);
        loop.sleep();
        gt_file << i << "," << searchPoint.x << "," << searchPoint.y << ","
                << searchPoint.z << "," << 1 << "," << loop_id << ","
                << max_overlap << std::endl;
        max_overlap = floor((max_overlap * pow(10, 3) + 0.5)) / pow(10, 3);
        std::cout << "loop trigger:" << i << "-" << loop_id
                  << ", overlap:" << max_overlap << std::endl;
        std::string max_overlap_str = std::to_string(max_overlap);
        max_overlap_str =
            max_overlap_str.substr(0, max_overlap_str.find(".") + 4);
        max_overlap_str = "Overlap: " + max_overlap_str;
        // publish_map(pubLaserCloudMap);
        cv::Mat max_overlap_pic = cv::Mat::zeros(200, 800, CV_8UC3);
        cv::Point siteNO;
        siteNO.x = 100;
        siteNO.y = 100;
        cv::putText(max_overlap_pic, max_overlap_str, siteNO, 4, 2,
                    cv::Scalar(255, 255, 255), 4);
        cv::imshow("", max_overlap_pic);
        cv::waitKey(500);
        // getchar();
      } else {
        gt_file << i << "," << searchPoint.x << "," << searchPoint.y << ","
                << searchPoint.z << "," << 0 << "," << 0 << "," << 0
                << std::endl;
      }
    }
  }
  for (size_t i = 0; i < opt_pose_list.size(); i++) {
    delete (opt_pose_list[i]);
  }
  return 0;
}