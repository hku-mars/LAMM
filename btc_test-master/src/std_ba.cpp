#include "include/std_ba.h"
#include "include/std.h"
#include "include/std_pgo.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

void geometric_icp(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    Eigen::Matrix3d &rot, Eigen::Vector3d &t) {
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < target_cloud->size(); i++) {
    pcl::PointXYZ pi;
    pi.x = target_cloud->points[i].x;
    pi.y = target_cloud->points[i].y;
    pi.z = target_cloud->points[i].z;
    input_cloud->push_back(pi);
  }
  kd_tree->setInputCloud(input_cloud);
  Eigen::Quaterniond q(rot.cast<double>());
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::Problem problem;
  ceres::LossFunction *loss_function = nullptr; // new ceres::HuberLoss(0.1);
  double para_q[4] = {q.x(), q.y(), q.z(), q.w()};
  double para_t[3] = {t(0), t(1), t(2)};
  problem.AddParameterBlock(para_q, 4, quaternion_manifold);
  problem.AddParameterBlock(para_t, 3);
  Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
  Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);
  // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  double useful_match = 0;
  double normal_threshold = 0.2;
  double dis_threshold = 0.5;
  for (size_t i = 0; i < cloud->size(); i++) {
    pcl::PointXYZINormal searchPoint = cloud->points[i];
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    pi = rot * pi + t;
    pcl::PointXYZ use_search_point;
    use_search_point.x = pi[0];
    use_search_point.y = pi[1];
    use_search_point.z = pi[2];
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                       searchPoint.normal_z);
    ni = rot * ni;
    if (kd_tree->nearestKSearch(use_search_point, 1, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      pcl::PointXYZINormal nearstPoint =
          target_cloud->points[pointIdxNKNSearch[0]];
      Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
      Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,
                          nearstPoint.normal_z);
      Eigen::Vector3d normal_inc = ni - tni;
      Eigen::Vector3d normal_add = ni + tni;
      double point_to_point_dis = (pi - tpi).norm();
      double point_to_plane = fabs(tni.transpose() * (pi - tpi));
      if ((normal_inc.norm() < normal_threshold ||
           normal_add.norm() < normal_threshold) &&
          point_to_plane < dis_threshold && point_to_point_dis < 3) {
        useful_match++;
        ceres::CostFunction *cost_function;
        Eigen::Vector3d curr_point(cloud->points[i].x, cloud->points[i].y,
                                   cloud->points[i].z);
        Eigen::Vector3d curr_normal(cloud->points[i].normal_x,
                                    cloud->points[i].normal_y,
                                    cloud->points[i].normal_z);

        cost_function = PlaneSolver::Create(curr_point, curr_normal, tpi, tni);
        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  Eigen::Quaterniond q_opt(para_q[3], para_q[0], para_q[1], para_q[2]);
  // std::cout << summary.BriefReport() << std::endl;
  rot = q_opt.toRotationMatrix();
  t << t_last_curr(0), t_last_curr(1), t_last_curr(2);

  std::cout << "useful match for icp:" << useful_match << std::endl;
}

void PoseOptimizer::addConnection(
    double *pose1, double *pose2,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &initial_guess,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud1,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud2,
    std::vector<PlanePair> &plane_match_list) {
  Eigen::Vector3d translation1(pose1[0], pose1[1], pose1[2]);
  Eigen::Quaterniond quaternion1(pose1[6], pose1[3], pose1[4], pose1[5]);
  Eigen::Matrix3d rotation1 = quaternion1.toRotationMatrix();
  Eigen::Vector3d translation2(pose2[0], pose2[1], pose2[2]);
  Eigen::Quaterniond quaternion2(pose2[6], pose2[3], pose2[4], pose2[5]);
  Eigen::Matrix3d rotation2 = quaternion2.toRotationMatrix();
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;
  problem_.AddParameterBlock(pose1, 3);
  problem_.AddParameterBlock(pose1 + 3, 4, quaternion_manifold);
  problem_.AddParameterBlock(pose2, 3);
  problem_.AddParameterBlock(pose2 + 3, 4, quaternion_manifold);
  int useful_num = 0;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < plane_cloud2->size(); i++) {
    pcl::PointXYZ pi;
    pi.x = plane_cloud2->points[i].x;
    pi.y = plane_cloud2->points[i].y;
    pi.z = plane_cloud2->points[i].z;
    input_cloud->push_back(pi);
  }
  kd_tree->setInputCloud(input_cloud);
  for (size_t i = 0; i < plane_cloud1->size(); i++) {
    pcl::PointXYZINormal searchPoint = plane_cloud1->points[i];
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                       searchPoint.normal_z);
    Eigen::Vector3d guess_pi = initial_guess.second * pi + initial_guess.first;
    Eigen::Vector3d guess_ni = initial_guess.second * ni;
    pcl::PointXYZ useSearchPoint;
    useSearchPoint.x = guess_pi[0];
    useSearchPoint.y = guess_pi[1];
    useSearchPoint.z = guess_pi[2];

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if (kd_tree->nearestKSearch(useSearchPoint, 1, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      pcl::PointXYZINormal nearstPoint =
          plane_cloud2->points[pointIdxNKNSearch[0]];
      Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
      Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,
                          nearstPoint.normal_z);
      Eigen::Vector3d normal_inc = guess_ni - tni;
      Eigen::Vector3d normal_add = guess_ni + tni;
      double point_to_point_dis = (guess_pi - tpi).norm();
      double point_to_plane = fabs(tni.transpose() * (guess_pi - tpi));
      if ((normal_inc.norm() < config_setting_.normal_threshold_ ||
           normal_add.norm() < config_setting_.normal_threshold_) &&
          point_to_plane < config_setting_.dis_threshold_ &&
          point_to_point_dis < 3) {
        useful_num++;
        ceres::CostFunction *cost_function;
        Eigen::Vector3d plane_normal1 = rotation1.transpose() * ni;
        Eigen::Vector3d plane_centriod1 =
            rotation1.transpose() * pi - rotation1.transpose() * translation1;
        Eigen::Vector3d plane_normal2 = rotation2.transpose() * tni;
        Eigen::Vector3d plane_centriod2 =
            rotation2.transpose() * tpi - rotation2.transpose() * translation2;
        cost_function = PlaneBaSolver::Create(plane_normal1, plane_centriod1,
                                              plane_normal2, plane_centriod2);
        problem_.AddResidualBlock(cost_function, loss_function_, pose1,
                                  pose1 + 3, pose2, pose2 + 3);
        PlanePair pp;
        pp.source_plane.x = plane_centriod1[0];
        pp.source_plane.y = plane_centriod1[1];
        pp.source_plane.z = plane_centriod1[2];
        pp.source_plane.normal_x = plane_normal1[0];
        pp.source_plane.normal_y = plane_normal1[1];
        pp.source_plane.normal_z = plane_normal1[2];

        pp.target_plane.x = plane_centriod2[0];
        pp.target_plane.y = plane_centriod2[1];
        pp.target_plane.z = plane_centriod2[2];
        pp.target_plane.normal_x = plane_normal2[0];
        pp.target_plane.normal_y = plane_normal2[1];
        pp.target_plane.normal_z = plane_normal2[2];
        plane_match_list.push_back(pp);
      }
    }
  }
  std::cout << "useful num:" << useful_num << std::endl;
}

void PoseOptimizer::addConnection(
    double *src_pose, double *tar_pose,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &src_plane_cloud,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &tar_plane_cloud,
    std::vector<PlanePair> &plane_pair_list) {
  Eigen::Vector3d translation1(src_pose[0], src_pose[1], src_pose[2]);
  Eigen::Quaterniond quaternion1(src_pose[6], src_pose[3], src_pose[4],
                                 src_pose[5]);
  Eigen::Matrix3d rotation1 = quaternion1.toRotationMatrix();
  Eigen::Vector3d translation2(tar_pose[0], tar_pose[1], tar_pose[2]);
  Eigen::Quaterniond quaternion2(tar_pose[6], tar_pose[3], tar_pose[4],
                                 tar_pose[5]);
  Eigen::Matrix3d rotation2 = quaternion2.toRotationMatrix();
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;
  problem_.AddParameterBlock(src_pose, 3);
  problem_.AddParameterBlock(src_pose + 3, 4, quaternion_manifold);
  problem_.AddParameterBlock(tar_pose, 3);
  problem_.AddParameterBlock(tar_pose + 3, 4, quaternion_manifold);
  int useful_num = 0;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < tar_plane_cloud->size(); i++) {
    Eigen::Vector3d pv = point2vec(tar_plane_cloud->points[i]);
    pv = rotation2 * pv + translation2;
    Eigen::Vector3d nv(tar_plane_cloud->points[i].normal_x,
                       tar_plane_cloud->points[i].normal_y,
                       tar_plane_cloud->points[i].normal_z);
    pcl::PointXYZ pi;
    pi.x = pv[0];
    pi.y = pv[1];
    pi.z = pv[2];
    input_cloud->push_back(pi);
  }
  kd_tree->setInputCloud(input_cloud);
  for (size_t i = 0; i < src_plane_cloud->size(); i++) {
    Eigen::Vector3d spi_body = point2vec(src_plane_cloud->points[i]);
    Eigen::Vector3d spi_world = rotation1 * spi_body + translation1;
    pcl::PointXYZINormal searchPoint = src_plane_cloud->points[i];
    // Eigen::Vector3d spi(searchPoint.x, searchPoint.y, searchPoint.z);
    Eigen::Vector3d sni_body(searchPoint.normal_x, searchPoint.normal_y,
                             searchPoint.normal_z);
    Eigen::Vector3d sni_world = rotation1 * sni_body;
    pcl::PointXYZ useSearchPoint;
    useSearchPoint.x = spi_world[0];
    useSearchPoint.y = spi_world[1];
    useSearchPoint.z = spi_world[2];

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if (kd_tree->nearestKSearch(useSearchPoint, 1, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      pcl::PointXYZINormal nearstPoint =
          tar_plane_cloud->points[pointIdxNKNSearch[0]];
      Eigen::Vector3d tpi_body(nearstPoint.x, nearstPoint.y, nearstPoint.z);
      Eigen::Vector3d tni_body(nearstPoint.normal_x, nearstPoint.normal_y,
                               nearstPoint.normal_z);
      Eigen::Vector3d tpi_world = rotation2 * tpi_body + translation2;
      Eigen::Vector3d tni_world = rotation2 * tni_body;
      Eigen::Vector3d normal_inc = sni_world - tni_world;
      Eigen::Vector3d normal_add = sni_world + tni_world;
      double point_to_point_dis = (spi_world - tpi_world).norm();
      double point_to_plane =
          fabs(tni_world.transpose() * (spi_world - tpi_world));
      if ((normal_inc.norm() < config_setting_.normal_threshold_ ||
           normal_add.norm() < config_setting_.normal_threshold_) &&
          point_to_plane < config_setting_.dis_threshold_ &&
          point_to_point_dis < 3) {
        useful_num++;
        ceres::CostFunction *cost_function;
        cost_function =
            PlaneBaSolver::Create(sni_body, spi_body, tni_body, tpi_body);
        problem_.AddResidualBlock(cost_function, loss_function_, src_pose,
                                  src_pose + 3, tar_pose, tar_pose + 3);
        PlanePair pp;
        pp.source_plane.x = spi_body[0];
        pp.source_plane.y = spi_body[1];
        pp.source_plane.z = spi_body[2];
        pp.source_plane.normal_x = sni_body[0];
        pp.source_plane.normal_y = sni_body[1];
        pp.source_plane.normal_z = sni_body[2];

        pp.target_plane.x = tpi_body[0];
        pp.target_plane.y = tpi_body[1];
        pp.target_plane.z = tpi_body[2];
        pp.target_plane.normal_x = tni_body[0];
        pp.target_plane.normal_y = tni_body[1];
        pp.target_plane.normal_z = tni_body[2];
        plane_pair_list.push_back(pp);
      }
    }
  }
  // std::cout << "useful num:" << useful_num << std::endl;
}

void PoseOptimizer::Solve() {
  // sparse surb
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options_.max_num_iterations = 100;
  options_.minimizer_progress_to_stdout = false;
  ceres::Solve(options_, &problem_, &summary_);
  std::cout << summary_.FullReport() << '\n';
}

void publish_std_surface(const std::vector<std::pair<STD, STD>> &match_std_list,
                         std::vector<Eigen::Vector3d> &color_list,
                         const ros::Publisher &std_publisher) {
  bool random_color = false;
  if (color_list.size() == 0) {
    random_color = true;
  }
  // publish descriptor
  // bool transform_enable = true;
  visualization_msgs::MarkerArray ma_triangle;
  visualization_msgs::Marker m_triangle;
  m_triangle.type = visualization_msgs::Marker::TRIANGLE_LIST;
  m_triangle.action = visualization_msgs::Marker::ADD;
  m_triangle.ns = "triangle";
  // Don't forget to set the alpha!
  m_triangle.scale.x = 1;
  m_triangle.scale.y = 1;
  m_triangle.scale.z = 1;
  // m_triangle.pose.orientation.w = 1.0;
  m_triangle.header.frame_id = "camera_init";
  m_triangle.id = 0;
  // line
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "triangle_line";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  // m_triangle.scale.y = 1;
  // m_triangle.scale.z = 1;
  // m_triangle.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  m_line.id = 0;
  int max_pub_cnt = 1;
  srand((unsigned)std::time(NULL));
  int std_id = 0;
  for (auto var : match_std_list) {

    if (max_pub_cnt > 100) {
      break;
    }
    max_pub_cnt++;
    m_triangle.color.a = 0.8;
    m_triangle.points.clear();
    if (random_color) {
      m_triangle.color.r = rand() % 256 * 1.0 / 256;
      m_triangle.color.g = rand() % 256 * 1.0 / 256;
      m_triangle.color.b = rand() % 256 * 1.0 / 256;
      color_list.push_back(Eigen::Vector3d(
          m_triangle.color.r, m_triangle.color.g, m_triangle.color.b));
    } else {
      m_triangle.color.r = color_list[std_id][0];
      m_triangle.color.g = color_list[std_id][1];
      m_triangle.color.b = color_list[std_id][2];
    }
    std_id++;

    m_line.color = m_triangle.color;
    Eigen::Vector3d t_p;
    geometry_msgs::Point p;

    // publish line
    p.x = var.second.binary_A_.location_[0];
    p.y = var.second.binary_A_.location_[1];
    p.z = var.second.binary_A_.location_[2];
    t_p << p.x, p.y, p.z;
    // if (transform_enable)
    //   t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.first.binary_A_.location_[0];
    p.y = var.first.binary_A_.location_[1];
    p.z = var.first.binary_A_.location_[2];
    t_p << p.x, p.y, p.z;
    // t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.second.binary_B_.location_[0];
    p.y = var.second.binary_B_.location_[1];
    p.z = var.second.binary_B_.location_[2];
    t_p << p.x, p.y, p.z;
    // if (transform_enable)
    //   t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.first.binary_B_.location_[0];
    p.y = var.first.binary_B_.location_[1];
    p.z = var.first.binary_B_.location_[2];
    t_p << p.x, p.y, p.z;
    // t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.second.binary_C_.location_[0];
    p.y = var.second.binary_C_.location_[1];
    p.z = var.second.binary_C_.location_[2];
    t_p << p.x, p.y, p.z;
    // if (transform_enable)
    //   t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.first.binary_C_.location_[0];
    p.y = var.first.binary_C_.location_[1];
    p.z = var.first.binary_C_.location_[2];
    t_p << p.x, p.y, p.z;
    // t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_triangle.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

    Eigen::Vector3d AB =
        var.second.binary_A_.location_ - var.second.binary_B_.location_;
    Eigen::Vector3d BC =
        var.second.binary_B_.location_ - var.second.binary_C_.location_;
    if (AB.cross(BC)[2] > 0) {
      p.x = var.second.binary_A_.location_[0];
      p.y = var.second.binary_A_.location_[1];
      p.z = var.second.binary_A_.location_[2];
      t_p << p.x, p.y, p.z;
      // if (transform_enable)
      //   t_p = rotation.inverse() * (t_p - translation);
      p.x = t_p[0];
      p.y = t_p[1];
      p.z = t_p[2];
      m_triangle.points.push_back(p);

      p.x = var.second.binary_B_.location_[0];
      p.y = var.second.binary_B_.location_[1];
      p.z = var.second.binary_B_.location_[2];
      t_p << p.x, p.y, p.z;
      // t_p = rotation.inverse() * (t_p - translation);
      p.x = t_p[0];
      p.y = t_p[1];
      p.z = t_p[2];
      m_triangle.points.push_back(p);

      p.x = var.second.binary_C_.location_[0];
      p.y = var.second.binary_C_.location_[1];
      p.z = var.second.binary_C_.location_[2];
      t_p << p.x, p.y, p.z;
      p.x = t_p[0];
      p.y = t_p[1];
      p.z = t_p[2];
      m_triangle.points.push_back(p);
      ma_triangle.markers.push_back(m_triangle);
      m_triangle.id++;
      m_triangle.points.clear();
    } else {

      p.x = var.second.binary_B_.location_[0];
      p.y = var.second.binary_B_.location_[1];
      p.z = var.second.binary_B_.location_[2];
      t_p << p.x, p.y, p.z;
      // t_p = rotation.inverse() * (t_p - translation);
      p.x = t_p[0];
      p.y = t_p[1];
      p.z = t_p[2];
      m_triangle.points.push_back(p);

      p.x = var.second.binary_A_.location_[0];
      p.y = var.second.binary_A_.location_[1];
      p.z = var.second.binary_A_.location_[2];
      t_p << p.x, p.y, p.z;
      // if (transform_enable)
      //   t_p = rotation.inverse() * (t_p - translation);
      p.x = t_p[0];
      p.y = t_p[1];
      p.z = t_p[2];
      m_triangle.points.push_back(p);

      p.x = var.second.binary_C_.location_[0];
      p.y = var.second.binary_C_.location_[1];
      p.z = var.second.binary_C_.location_[2];
      t_p << p.x, p.y, p.z;
      p.x = t_p[0];
      p.y = t_p[1];
      p.z = t_p[2];
      m_triangle.points.push_back(p);
      ma_triangle.markers.push_back(m_triangle);
      m_triangle.id++;
      m_triangle.points.clear();
    }

    // another
    // m_triangle.color.r = 1;
    // m_triangle.color.g = 0;
    // m_triangle.color.b = 0;
    AB = var.first.binary_A_.location_ - var.first.binary_B_.location_;
    BC = var.first.binary_B_.location_ - var.first.binary_C_.location_;
    if (AB.cross(BC)[2] > 0) {
      p.x = var.first.binary_A_.location_[0];
      p.y = var.first.binary_A_.location_[1];
      p.z = var.first.binary_A_.location_[2];
      m_triangle.points.push_back(p);
      p.x = var.first.binary_B_.location_[0];
      p.y = var.first.binary_B_.location_[1];
      p.z = var.first.binary_B_.location_[2];
      m_triangle.points.push_back(p);
      p.x = var.first.binary_C_.location_[0];
      p.y = var.first.binary_C_.location_[1];
      p.z = var.first.binary_C_.location_[2];
      m_triangle.points.push_back(p);
      ma_triangle.markers.push_back(m_triangle);
      m_triangle.id++;
      m_triangle.points.clear();
    } else {
      p.x = var.first.binary_B_.location_[0];
      p.y = var.first.binary_B_.location_[1];
      p.z = var.first.binary_B_.location_[2];
      m_triangle.points.push_back(p);
      p.x = var.first.binary_A_.location_[0];
      p.y = var.first.binary_A_.location_[1];
      p.z = var.first.binary_A_.location_[2];
      m_triangle.points.push_back(p);
      p.x = var.first.binary_C_.location_[0];
      p.y = var.first.binary_C_.location_[1];
      p.z = var.first.binary_C_.location_[2];
      m_triangle.points.push_back(p);
      ma_triangle.markers.push_back(m_triangle);
      m_triangle.id++;
      m_triangle.points.clear();
    }
  }
  for (int j = 0; j < 100 * 6; j++) {
    m_line.color.a = 0.00;
    m_triangle.color.a = 0.00;
    ma_triangle.markers.push_back(m_triangle);
    ma_triangle.markers.push_back(m_line);
    m_triangle.id++;
    m_line.id++;
  }
  std_publisher.publish(ma_triangle);
  m_triangle.id = 0;
  ma_triangle.markers.clear();
}

void publish_std_line(const std::vector<std::pair<STD, STD>> &match_std_list,
                      const ros::Publisher &std_publisher) {
  // publish descriptor
  // bool transform_enable = true;
  visualization_msgs::MarkerArray ma_triangle;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "triangle_line";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  // m_line.scale.y = 1;
  // m_line.scale.z = 1;
  // m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  m_line.id = 0;
  int max_pub_cnt = 1;
  srand((unsigned)std::time(NULL));
  for (auto var : match_std_list) {
    if (max_pub_cnt > 100) {
      break;
    }
    max_pub_cnt++;
    m_line.color.a = 1;
    m_line.points.clear();
    m_line.color.r = 255.0 / 256;
    m_line.color.g = 215.0 / 256;
    m_line.color.b = 0;
    Eigen::Vector3d t_p;
    geometry_msgs::Point p;

    p.x = var.second.binary_A_.location_[0];
    p.y = var.second.binary_A_.location_[1];
    p.z = var.second.binary_A_.location_[2];
    t_p << p.x, p.y, p.z;
    // if (transform_enable)
    //   t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.first.binary_A_.location_[0];
    p.y = var.first.binary_A_.location_[1];
    p.z = var.first.binary_A_.location_[2];
    t_p << p.x, p.y, p.z;
    // t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.second.binary_B_.location_[0];
    p.y = var.second.binary_B_.location_[1];
    p.z = var.second.binary_B_.location_[2];
    t_p << p.x, p.y, p.z;
    // if (transform_enable)
    //   t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.first.binary_B_.location_[0];
    p.y = var.first.binary_B_.location_[1];
    p.z = var.first.binary_B_.location_[2];
    t_p << p.x, p.y, p.z;
    // t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.second.binary_C_.location_[0];
    p.y = var.second.binary_C_.location_[1];
    p.z = var.second.binary_C_.location_[2];
    t_p << p.x, p.y, p.z;
    // if (transform_enable)
    //   t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);

    p.x = var.first.binary_C_.location_[0];
    p.y = var.first.binary_C_.location_[1];
    p.z = var.first.binary_C_.location_[2];
    t_p << p.x, p.y, p.z;
    // t_p = rotation.inverse() * (t_p - translation);
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_triangle.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
  }
  for (int j = 0; j < 100 * 6; j++) {
    m_line.color.a = 0.00;
    ma_triangle.markers.push_back(m_line);
    m_line.id++;
  }
  std_publisher.publish(ma_triangle);
  m_line.id = 0;
  ma_triangle.markers.clear();
}

double ICP_verify(const pcl::PointCloud<pcl::PointXYZI>::Ptr &key_cloud,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr &history_cloud,
                  const Eigen::Matrix3d &rot, const Eigen::Vector3d t) {
  int point_inc_num = 1;
  pcl::KdTreeFLANN<pcl::PointXYZI> kd_tree;
  kd_tree.setInputCloud(history_cloud);
  int K = 1;
  // old 0.5*0.5
  double dis_threshold = 0.5 * 0.5;
  // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  double match_num = 0;
  for (size_t i = 0; i < key_cloud->size(); i = i + point_inc_num) {
    pcl::PointXYZI searchPoint = key_cloud->points[i];
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    pi = rot * pi + t;
    searchPoint.x = pi[0];
    searchPoint.y = pi[1];
    searchPoint.z = pi[2];
    if (kd_tree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                               pointNKNSquaredDistance) > 0) {
      if (pointNKNSquaredDistance[0] < dis_threshold) {
        match_num++;
      }
    }
  }
  double match_degree = 2.0 * point_inc_num * match_num /
                        (key_cloud->size() + history_cloud->size());
  // std::cout << "[icp] match rate:" << match_degree << std::endl;
  return match_degree;
}