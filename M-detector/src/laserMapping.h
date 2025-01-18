#ifndef LASER_MAPPING_H
#define LASER_MAPPING_H

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <pcl/common/io.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include "common_lib.h"

#define time_debug
#define HASH_P 116101
#define MAX_N 10000000000

int plane_id = 0;
int max_layer = 1;
int max_points = 100;
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
// int layer_size[3] = {8, 5, 10};
// float eigen_value_array[3] = {0.0025, 0.0025, 0.0005};
typedef struct ptpl {
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  Eigen::Vector3d center;
  Eigen::Matrix<double, 6, 6> plane_var;
  double d;
  double eigen_value;
  bool   is_valid;
} ptpl;

typedef struct pointWithVar {
  Eigen::Vector3d point;
  Eigen::Matrix3d var;
} pointWithVar;

// typedef struct ndtcell {
//   Eigen::Vector3d point;
//   Eigen::Vector3d mean;
//   Eigen::Matrix3d sigma;

// } ndtcell;

const bool intensity_contrast(PointType &x, PointType &y) {
  return (x.intensity > y.intensity);
};

const bool var_contrast(pointWithVar &x, pointWithVar &y) {
  return (x.var.diagonal().norm() < y.var.diagonal().norm());
};

float calc_dist(PointType p1, PointType p2){
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

typedef struct Plane {
  pcl::PointXYZINormal p_center;
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  Eigen::Matrix3d covariance;
  Eigen::Matrix<double, 6, 6> plane_var;
  float radius = 0;
  float min_eigen_value = 1;
  float d = 0;
  int points_size = 0;
  bool is_plane = false;
  bool is_init = false;
  int id;
  bool is_update = false;
} Plane;

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <> struct hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
} // namespace std

struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

class OctoTree {
public:
  std::vector<pointWithVar> temp_points_;
  Plane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  Eigen::Vector3d layer_size_;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;
  OctoTree(int layer, int points_size_threshold, float planer_threshold)
      : layer_(layer), points_size_threshold_(points_size_threshold),
        planer_threshold_(planer_threshold) {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
  }

  void init_plane(const std::vector<pointWithVar> &points, Plane *plane) {
    plane->plane_var = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance = Eigen::Matrix3d::Zero();
    plane->center = Eigen::Vector3d::Zero();
    plane->normal = Eigen::Vector3d::Zero();
    plane->points_size = points.size();
    plane->radius = 0;
    for (auto pv : points) {
      plane->covariance += pv.point * pv.point.transpose();
      plane->center += pv.point;
    }
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->covariance / plane->points_size -
                        plane->center * plane->center.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = (evalsMin == evalsMax) ? evalsMin : 3 - evalsMin - evalsMax;
    // cout<<"min mid max: "<<evalsMin<<" "<<evalsMid<<" "<<evalsMax<<endl;
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0,
        1.0 / plane->points_size;
    // && evalsReal(evalsMid) > 0.05
    //
    if (evalsReal(evalsMin) < planer_threshold_ && evalsReal(evalsMid) > 0.01) {
      // std::cout << "min eigen value:" << evalsReal(evalsMin)
      //           << " mid eigen:" << evalsReal(evalsMid)
      //           << " max eigen:" << evalsReal(evalsMax) << std::endl;
      for (int i = 0; i < points.size(); i++) {
        Eigen::Matrix<double, 6, 3> J;
        Eigen::Matrix3d F;
        for (int m = 0; m < 3; m++) {
          if (m != (int)evalsMin) {
            Eigen::Matrix<double, 1, 3> F_m =
                (points[i].point - plane->center).transpose() /
                ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                 evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
            F.row(m) = F_m;
          } else {
            Eigen::Matrix<double, 1, 3> F_m;
            F_m << 0, 0, 0;
            F.row(m) = F_m;
          }
        }
        J.block<3, 3>(0, 0) = evecs.real() * F;
        // std::cout << "F:" << std::endl
        //           << J.block<3, 3>(0, 0) << std::endl;
        J.block<3, 3>(3, 0) = J_Q;
        plane->plane_var += J * points[i].var * J.transpose();
      }
      // std::cout << "plane var: " << std::endl << plane->plane_var <<
      // std::endl;
      // std::cout << "Eigen value: " << evalsReal(evalsMin) << std::endl;
      // std::cout << "Normal vector " << evecs.real().col(evalsMin)(0) << ","
      //           << evecs.real().col(evalsMin)(1) << ","
      //           << evecs.real().col(evalsMin)(2) << " Center"
      //           << plane->center(0) << "," << plane->center(1) << ","
      //           << plane->center(2) << std::endl;

      plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
          evecs.real()(2, evalsMin);
      // Eigen::Vector3d test_p = plane->center + 0.5 * plane->normal;
      // Eigen::Matrix<double, 1, 6> J_w;
      // J_w.block<1, 3>(0, 0) = test_p - plane->center;
      // J_w.block<1, 3>(0, 3) = -plane->normal;
      // double sigma_len = sqrt(J_w * plane->plane_var * J_w.transpose());
      // std::cout << "Sigma len: " << sigma_len << std::endl;

      plane->min_eigen_value = evalsReal(evalsMin);
      plane->radius = sqrt(evalsReal(evalsMax));
      plane->d = -(plane->normal(0) * plane->center(0) +
                   plane->normal(1) * plane->center(1) +
                   plane->normal(2) * plane->center(2));
      plane->p_center.x = plane->center(0);
      plane->p_center.y = plane->center(1);
      plane->p_center.z = plane->center(2);
      plane->p_center.normal_x = plane->normal(0);
      plane->p_center.normal_y = plane->normal(1);
      plane->p_center.normal_z = plane->normal(2);
      plane->is_plane = true;
      plane->is_update = true;
      if (!plane->is_init) {
        plane->id = plane_id;
        plane_id++;
        plane->is_init = true;
      }

      // Calc Normal and center covariance
    } else {
      plane->is_update = true;
      plane->is_plane = false;
    }
  }

  void init_octo_tree() {
    if (temp_points_.size() > points_size_threshold_) {
      init_plane(temp_points_, plane_ptr_);
      if (plane_ptr_->is_plane == true) {
        octo_state_ = 0;
      } else {
        octo_state_ = 1;
        cut_octo_tree();
      }
      init_octo_ = true;
      new_points_ = 0;
      //      temp_points_.clear();
    }
  }

  void cut_octo_tree() {
    if (layer_ >= max_layer) {
      octo_state_ = 0;
      return;
    }
    for (size_t i = 0; i < temp_points_.size(); i++) {
      int xyz[3] = {0, 0, 0};
      if (temp_points_[i].point[0] > voxel_center_[0]) {
        xyz[0] = 1;
      }
      if (temp_points_[i].point[1] > voxel_center_[1]) {
        xyz[1] = 1;
      }
      if (temp_points_[i].point[2] > voxel_center_[2]) {
        xyz[2] = 1;
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves_[leafnum] == nullptr) {
        leaves_[leafnum] = new OctoTree(layer_ + 1, layer_size_[layer_ + 1],
                                        planer_threshold_);
        leaves_[leafnum]->layer_size_ = layer_size_;
        leaves_[leafnum]->voxel_center_[0] =
            voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[1] =
            voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[2] =
            voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
        leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      }
      leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
      leaves_[leafnum]->new_points_++;
    }
    for (uint i = 0; i < 8; i++) {
      if (leaves_[i] != nullptr) {
        if (leaves_[i]->temp_points_.size() >
            leaves_[i]->points_size_threshold_) {
          init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
          if (leaves_[i]->plane_ptr_->is_plane) {
            leaves_[i]->octo_state_ = 0;
          } else {
            leaves_[i]->octo_state_ = 1;
            leaves_[i]->cut_octo_tree();
          }
          leaves_[i]->init_octo_ = true;
          leaves_[i]->new_points_ = 0;
          // leaves_[i]->temp_points_.clear();
        }
      }
    }
  }

  void UpdateOctoTree(const pointWithVar &pv) {
    if (!init_octo_) {
      new_points_++;
      temp_points_.push_back(pv);
      if (temp_points_.size() > points_size_threshold_) {
        init_octo_tree();
      }
    } else {
      if (plane_ptr_->is_plane) {
        if (update_enable_) {
          new_points_++;
          temp_points_.push_back(pv);
          if (new_points_ > update_size_threshold_) {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
          }
          if (temp_points_.size() >= max_points) {
            update_enable_ = false;
            temp_points_.clear();
          }
        }
      } else {
        if (layer_ < max_layer) {
          int xyz[3] = {0, 0, 0};
          if (pv.point[0] > voxel_center_[0]) {
            xyz[0] = 1;
          }
          if (pv.point[1] > voxel_center_[1]) {
            xyz[1] = 1;
          }
          if (pv.point[2] > voxel_center_[2]) {
            xyz[2] = 1;
          }
          int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
          if (leaves_[leafnum] != nullptr) {
            leaves_[leafnum]->UpdateOctoTree(pv);
          } else {
            leaves_[leafnum] = new OctoTree(layer_ + 1, layer_size_[layer_ + 1],
                                            planer_threshold_);
            leaves_[leafnum]->layer_size_ = layer_size_;
            leaves_[leafnum]->voxel_center_[0] =
                voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] =
                voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] =
                voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            leaves_[leafnum]->UpdateOctoTree(pv);
          }
        } else {
          if (update_enable_) {
            new_points_++;
            temp_points_.push_back(pv);
            if (new_points_ > update_size_threshold_) {
              init_plane(temp_points_, plane_ptr_);
              new_points_ = 0;
            }
            if (temp_points_.size() > max_points) {
              update_enable_ = false;
              temp_points_.clear();
            }
          }
        }
      }
    }
  }

  void updatePlane() {
    if (temp_points_.size() >= update_size_threshold_) {
      Eigen::Matrix3d old_covariance = plane_ptr_->covariance;
      Eigen::Vector3d old_center = plane_ptr_->center;
      Eigen::Matrix3d sum_ppt =
          (plane_ptr_->covariance +
           plane_ptr_->center * plane_ptr_->center.transpose()) *
          plane_ptr_->points_size;
      Eigen::Vector3d sum_p = plane_ptr_->center * plane_ptr_->points_size;
      for (size_t i = 0; i < temp_points_.size(); i++) {
        const pointWithVar &pv = temp_points_[i];
        sum_ppt += pv.point * pv.point.transpose();
        sum_p += pv.point;
      }
      plane_ptr_->points_size = plane_ptr_->points_size + temp_points_.size();
      plane_ptr_->center = sum_p / plane_ptr_->points_size;
      plane_ptr_->covariance =
          sum_ppt / plane_ptr_->points_size -
          plane_ptr_->center * plane_ptr_->center.transpose();
      Eigen::EigenSolver<Eigen::Matrix3d> es(plane_ptr_->covariance);
      Eigen::Matrix3cd evecs = es.eigenvectors();
      Eigen::Vector3cd evals = es.eigenvalues();
      Eigen::Vector3d evalsReal; //注意这里定义的MatrixXd里没有c
      evalsReal = evals.real();  //获取特征值实数部分
      Eigen::Matrix3f::Index evalsMin, evalsMax;
      evalsReal.rowwise().sum().minCoeff(&evalsMin);
      evalsReal.rowwise().sum().maxCoeff(&evalsMax);
      // std::cout << "min eigen value:" << evalsReal(evalsMin) <<
      // std::endl;
      if (evalsReal(evalsMin) < planer_threshold_) {
        plane_ptr_->normal << evecs.real()(0, evalsMin),
            evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane_ptr_->min_eigen_value = evalsReal(evalsMin);
        plane_ptr_->radius = sqrt(evalsReal(evalsMax));
        plane_ptr_->d = -(plane_ptr_->normal(0) * plane_ptr_->center(0) +
                          plane_ptr_->normal(1) * plane_ptr_->center(1) +
                          plane_ptr_->normal(2) * plane_ptr_->center(2));
        plane_ptr_->p_center.x = plane_ptr_->center(0);
        plane_ptr_->p_center.y = plane_ptr_->center(1);
        plane_ptr_->p_center.z = plane_ptr_->center(2);
        plane_ptr_->p_center.normal_x = plane_ptr_->normal(0);
        plane_ptr_->p_center.normal_y = plane_ptr_->normal(1);
        plane_ptr_->p_center.normal_z = plane_ptr_->normal(2);
        plane_ptr_->is_plane = true;
        // temp_points_.clear();
        new_points_ = 0;
        plane_ptr_->is_update = true;
      } else {
        // plane_ptr_->is_plane = false;
        plane_ptr_->is_update = true;
        plane_ptr_->covariance = old_covariance;
        plane_ptr_->center = old_center;
        plane_ptr_->points_size = plane_ptr_->points_size - temp_points_.size();
        // temp_points_.clear();
        new_points_ = 0;
      }
    }
  }
};

void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) {
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) {
    v = vmin;
  }

  if (v > vmax) {
    v = vmax;
  }

  double dr, dg, db;

  if (v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if (v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void buildUnorderMap(const std::vector<pointWithVar> &input_points,
                     const float voxel_size, const Eigen::Vector3d &layer_size,
                     const float planer_threshold,
                     std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map) {
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++) {
    const pointWithVar p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_v.point[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      feat_map[position]->temp_points_.push_back(p_v);
      feat_map[position]->new_points_++;
    } else {
      OctoTree *octo_tree = new OctoTree(0, layer_size[0], planer_threshold);
      feat_map[position] = octo_tree;
      feat_map[position]->quater_length_ = voxel_size / 4;
      feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      feat_map[position]->temp_points_.push_back(p_v);
      feat_map[position]->new_points_++;
      feat_map[position]->layer_size_ = layer_size;
    }
  }
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    iter->second->init_octo_tree();
  }
}

void updateUnorderMap(const std::vector<pointWithVar> &input_points,
                      const float voxel_size, const Eigen::Vector3d &layer_size,
                      const float planer_threshold,
                      std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map) {
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++) {
    const pointWithVar p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_v.point[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      feat_map[position]->UpdateOctoTree(p_v);
    } else {
      OctoTree *octo_tree = new OctoTree(0, layer_size[0], planer_threshold);
      feat_map[position] = octo_tree;
      feat_map[position]->layer_size_ = layer_size;
      feat_map[position]->quater_length_ = voxel_size / 4;
      feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      feat_map[position]->UpdateOctoTree(p_v);
    }
  }
}

void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                    const PointCloudXYZI::Ptr &input_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud) {
  trans_cloud->clear();
  for (size_t i = 0; i < input_cloud->size(); i++) {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = rot * p + t;
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

// void BuildNdtList(const unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
//                   const float match_eigen_value, const int layer,
//                   const float match_constraint, const Eigen::Matrix3d rot,
//                   const Eigen::Vector3d t,
//                   const PointCloudXYZI::Ptr input_cloud,
//                   std::vector<ndtcell> &ndt_list) {
//   // debug time
//   double t1, t2;
//   double t_sum = 0;
//   int match_plane_size = 0;
//   int match_sub_plane_size = 0;
//   for (size_t i = 0; i < input_cloud->points.size(); i++) {
//     pcl::PointXYZI p_c;
//     p_c.x = input_cloud->points[i].x;
//     p_c.y = input_cloud->points[i].y;
//     p_c.z = input_cloud->points[i].z;
//     p_c.intensity = input_cloud->points[i].intensity;
//     pcl::PointXYZI p_w;
//     Eigen::Vector3d p_wv(p_c.x, p_c.y, p_c.z);
//     p_wv = rot * (p_wv + Lidar_T_wrt_IMU) + t;
//     p_w.x = p_wv(0);
//     p_w.y = p_wv(1);
//     p_w.z = p_wv(2);
//     p_w.intensity = p_c.intensity;
//     float loc_xyz[3];
//     for (int j = 0; j < 3; j++) {
//       loc_xyz[j] = p_w.data[j] / 1.0;
//       if (loc_xyz[j] < 0) {
//         loc_xyz[j] -= 1.0;
//       }
//     }
//     VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
//                        (int64_t)loc_xyz[2]);
//     auto iter = feat_map.find(position);
//     bool match_flag = false;
//     float radius_k1 = 1.0;
//     float radius_k2 = 1.0;
//     if (iter != feat_map.end()) {
//       if (iter->second->octo_state_ == 0 &&
//           iter->second->plane_ptr_->is_plane &&
//           iter->second->plane_ptr_->min_eigen_value < match_eigen_value) {
//         // use new dis_threshold;
//         Plane *plane = iter->second->plane_ptr_;
//         float dis_to_plane =
//             (plane->normal(0) * p_w.x + plane->normal(1) * p_w.y +
//              plane->normal(2) * p_w.z + plane->d);
//         float dis_to_center =
//             (plane->center(0) - p_w.x) * (plane->center(0) - p_w.x) +
//             (plane->center(1) - p_w.y) * (plane->center(1) - p_w.y) +
//             (plane->center(2) - p_w.z) * (plane->center(2) - p_w.z);
//         float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);
//         float s =
//             1 - 0.9 * fabs(dis_to_plane) /
//                     sqrt(sqrt(p_c.x * p_c.x + p_c.y * p_c.y + p_c.z * p_c.z));
//         float dis_threshold =
//             3 * sqrt(iter->second->plane_ptr_->min_eigen_value);

//         // if (fabs(dis_to_plane) < dis_to_plane_threshold) {
//         if (s > match_constraint &&
//             range_dis < radius_k1 * iter->second->plane_ptr_->radius) {
//           ndtcell single_ndt;
//           single_ndt.point << p_c.x, p_c.y, p_c.z;
//           single_ndt.mean = iter->second->plane_ptr_->center;
//           single_ndt.sigma = iter->second->plane_ptr_->covariance;
//           ndt_list.push_back(single_ndt);
//           match_plane_size++;
//         }
//       } else {
//         if (layer == 1) {
//           float min_dis = 100;
//           ndtcell single_ndt;
//           for (int j = 0; j < 8; j++) {
//             if (iter->second->leaves_[j] != nullptr) {
//               if (iter->second->leaves_[j]->plane_ptr_->is_plane &&
//                   iter->second->leaves_[j]->plane_ptr_->min_eigen_value <
//                       match_eigen_value) {
//                 Plane *plane = iter->second->leaves_[j]->plane_ptr_;
//                 float dis_to_plane =
//                     (plane->normal(0) * p_w.x + plane->normal(1) * p_w.y +
//                      plane->normal(2) * p_w.z + plane->d);
//                 float dis_to_center = sqrt(
//                     (plane->center(0) - p_w.x) * (plane->center(0) - p_w.x) +
//                     (plane->center(1) - p_w.y) * (plane->center(1) - p_w.y) +
//                     (plane->center(2) - p_w.z) * (plane->center(2) - p_w.z));
//                 float range_dis = sqrt(dis_to_center * dis_to_center -
//                                        dis_to_plane * dis_to_plane);
//                 float dis_threshold =
//                     3 *
//                     sqrt(iter->second->leaves_[j]->plane_ptr_->min_eigen_value);
//                 float s = 1 - 0.9 * fabs(dis_to_plane) /
//                                   sqrt(sqrt(p_c.x * p_c.x + p_c.y * p_c.y +
//                                             p_c.z * p_c.z));
//                 if (s > match_constraint &&
//                     range_dis <
//                         radius_k2 *
//                             iter->second->leaves_[j]->plane_ptr_->radius) {
//                   // std::cout << "3_sigma:" << dis_threshold
//                   //           << " distance:" << dis_to_plane << std::endl;
//                   // if (min_dis > dis_to_plane) {
//                   // if (fabs(dis_to_plane > dis_threshold)) {
//                   min_dis = dis_to_plane;
//                   single_ndt.point << p_c.x, p_c.y, p_c.z;
//                   single_ndt.mean = iter->second->plane_ptr_->center;
//                   single_ndt.sigma = iter->second->plane_ptr_->covariance;
//                   // ptpl_list.push_back(single_ptpl);
//                   // match_sub_plane_size++;
//                   //}
//                   //}
//                 }
//               }
//             }
//           }
//           if (min_dis != 100) {
//             ndt_list.push_back(single_ndt);
//             match_sub_plane_size++;
//           }
//         }
//       }
//     }
//   }
//   std::cout << "[ Matching ]: match plane size: " << match_plane_size
//             << " , match sub plane size: " << match_sub_plane_size << std::endl;
// }

void BuildPtplList(const unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                   const float match_eigen_value, const int layer,
                   const float voxel_size, const float match_constraint,
                   const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                   const PointCloudXYZI::Ptr input_cloud,
                   std::vector<ptpl> &ptpl_list) {
  // debug time
  double t_sum = 0;
  int match_plane_size = 0;
  int match_sub_plane_size = 0;
  for (size_t i = 0; i < input_cloud->points.size(); i++) {
    pcl::PointXYZI p_c;
    p_c.x = input_cloud->points[i].x;
    p_c.y = input_cloud->points[i].y;
    p_c.z = input_cloud->points[i].z;
    p_c.intensity = input_cloud->points[i].intensity;

    pcl::PointXYZI p_w;
    Eigen::Vector3d p_cv(p_c.x, p_c.y, p_c.z);
    Eigen::Vector3d p_wv(p_c.x, p_c.y, p_c.z);
    p_wv = rot * (p_cv) + t;
    p_w.x = p_wv(0);
    p_w.y = p_wv(1);
    p_w.z = p_wv(2);
    p_w.intensity = p_c.intensity;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_w.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    bool match_flag = false;
    float radius_k1 = 1.25;
    float radius_k2 = 1.25;
    if (iter != feat_map.end()) {
      if (iter->second->octo_state_ == 0 &&
          iter->second->plane_ptr_->is_plane &&
          iter->second->plane_ptr_->min_eigen_value < match_eigen_value) {
        // use new dis_threshold;
        Plane *plane = iter->second->plane_ptr_;
        float dis_to_plane =
            (plane->normal(0) * p_w.x + plane->normal(1) * p_w.y +
             plane->normal(2) * p_w.z + plane->d);
        float dis_to_center =
            (plane->center(0) - p_w.x) * (plane->center(0) - p_w.x) +
            (plane->center(1) - p_w.y) * (plane->center(1) - p_w.y) +
            (plane->center(2) - p_w.z) * (plane->center(2) - p_w.z);
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);
        float s =
            1 - 0.9 * fabs(dis_to_plane) /
                    sqrt(sqrt(p_c.x * p_c.x + p_c.y * p_c.y + p_c.z * p_c.z));
        float dis_threshold =
            3 * sqrt(iter->second->plane_ptr_->min_eigen_value);

        // if (fabs(dis_to_plane) < dis_to_plane_threshold) {
        if (s > match_constraint &&
            range_dis < radius_k1 * iter->second->plane_ptr_->radius) {
          // std::cout << "range dis: " << range_dis
          //           << " plane radius: " << iter->second->plane_ptr_->radius
          //           << std::endl;
          // if (fabs(dis_to_plane) < dis_threshold) {
          ptpl single_ptpl;
          single_ptpl.point << p_c.x, p_c.y, p_c.z;
          single_ptpl.normal << plane->normal(0), plane->normal(1),
              plane->normal(2);
          single_ptpl.center = plane->center;
          single_ptpl.plane_var = plane->plane_var;
          single_ptpl.d = plane->d;
          single_ptpl.eigen_value = plane->min_eigen_value;
          ptpl_list.push_back(single_ptpl);
          match_plane_size++;
          //}
          // std::cout << "3_sigma:" << dis_threshold
          //           << " distance:" << dis_to_plane << std::endl;
        }
      } else {
        if (layer >= 1) {
          float min_dis = 100;
          ptpl single_ptpl;
          for (int j = 0; j < 8; j++) {
            if (iter->second->leaves_[j] != nullptr) {
              if (iter->second->leaves_[j]->plane_ptr_->is_plane &&
                  iter->second->leaves_[j]->plane_ptr_->min_eigen_value <
                      match_eigen_value) {
                Plane *plane = iter->second->leaves_[j]->plane_ptr_;
                float dis_to_plane =
                    (plane->normal(0) * p_w.x + plane->normal(1) * p_w.y +
                     plane->normal(2) * p_w.z + plane->d);
                float dis_to_center =
                    (plane->center(0) - p_w.x) * (plane->center(0) - p_w.x) +
                    (plane->center(1) - p_w.y) * (plane->center(1) - p_w.y) +
                    (plane->center(2) - p_w.z) * (plane->center(2) - p_w.z);
                float range_dis =
                    sqrt(dis_to_center - dis_to_plane * dis_to_plane);
                float dis_threshold =
                    3 *
                    sqrt(iter->second->leaves_[j]->plane_ptr_->min_eigen_value);
                float s = 1 - 0.9 * fabs(dis_to_plane) /
                                  sqrt(sqrt(p_c.x * p_c.x + p_c.y * p_c.y +
                                            p_c.z * p_c.z));
                if (s > match_constraint &&
                    range_dis <
                        radius_k2 *
                            iter->second->leaves_[j]->plane_ptr_->radius) {
                  // std::cout << "3_sigma:" << dis_threshold
                  //           << " distance:" << dis_to_plane << std::endl;
                  // if (min_dis > dis_to_plane) {
                  // if (fabs(dis_to_plane > dis_threshold)) {
                  min_dis = dis_to_plane;
                  single_ptpl.point << p_c.x, p_c.y, p_c.z;
                  single_ptpl.normal << plane->normal(0), plane->normal(1),
                      plane->normal(2);
                  single_ptpl.plane_var = plane->plane_var;
                  single_ptpl.center = plane->center;
                  single_ptpl.d = plane->d;
                  single_ptpl.eigen_value = plane->min_eigen_value;
                  // ptpl_list.push_back(single_ptpl);
                  // match_sub_plane_size++;
                  //}
                  //}
                }
              }
            }
          }
          if (min_dis != 100) {
            ptpl_list.push_back(single_ptpl);
            match_sub_plane_size++;
          }
        }
      }
    }
  }
  std::cout << "[ Matching ]: match plane size: " << match_plane_size
            << " , match sub plane size: " << match_sub_plane_size << std::endl;
}

void BuildResidualList(const unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                       const double voxel_size, const double sigma_num,
                       const pointWithVar &pv, const M3D &rot,
                       const V3D &t,  ptpl &ptpl_list) {
  float radius_k = 2;
  int match_index = -1;
  int match_plane = 0;
  int match_sub_plane = 0;
  int match_sub_sub_plane = 0;
  int check_plane = 0;
  int check_sub_plane = 0;
  int check_sub_sub_plane = 0;
  // ptpl_list.clear();

  ptpl_list.is_valid = false;

  // for (auto pv : pv_list) {
    V3D p_w = rot * pv.point + t;
    if (pv.point.norm() < 0) {
      return;
    }
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_w[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    double current_layer = 0;

    if (iter != feat_map.end()) {
      OctoTree *current_octo = iter->second;
      ptpl single_ptpl;
      double max_prob = 0;
      if (current_octo->plane_ptr_->is_plane) {
        Plane *plane = current_octo->plane_ptr_;
        // if (pv.point.norm() < 20 && fabs(plane->normal(2)) > 0.7) {
        //   continue;
        // }
        float dis_to_plane =
            fabs(plane->normal(0) * p_w(0) + plane->normal(1) * p_w(1) +
                 plane->normal(2) * p_w(2) + plane->d);
        float dis_to_center =
            (plane->center(0) - p_w(0)) * (plane->center(0) - p_w(0)) +
            (plane->center(1) - p_w(1)) * (plane->center(1) - p_w(1)) +
            (plane->center(2) - p_w(2)) * (plane->center(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);
        if (range_dis > radius_k * plane->radius) {
          return;
        }
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = p_w - plane->center;
        J_nq.block<1, 3>(0, 3) = -plane->normal;
        double sigma_l = J_nq * plane->plane_var * J_nq.transpose();
        // std::cout << "measure dis:" << dis_to_plane
        //           << " ,3sigmal:" << 3 * sqrt(sigma_l) << ", plane var:"
        //           << plane->plane_var.block<3, 3>(0, 0).diagonal().norm()
        //           << std::endl;
        sigma_l += plane->normal.transpose() * pv.var * plane->normal;
        // if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
        if (pv.point.norm() > sigma_num * dis_to_plane * dis_to_plane) {
          check_plane++;
          // match_plane++;
          float prob = 1 / (sqrt(2 * M_PI) * sqrt(sigma_l)) *
                       exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
          if (prob > max_prob) {
            max_prob = prob;
            single_ptpl.point = pv.point;
            single_ptpl.plane_var = plane->plane_var;
            single_ptpl.normal = plane->normal;
            single_ptpl.center = plane->center;
            single_ptpl.d = plane->d;
            single_ptpl.eigen_value = plane->min_eigen_value;
            match_index = 0;
          }
        }
      } else {
        for (size_t i = 0; i < 8; i++) {
          // continue;
          OctoTree *leaf_octo = current_octo->leaves_[i];
          if (leaf_octo != nullptr) {
            if (leaf_octo->plane_ptr_->is_plane) {
              Plane *plane = leaf_octo->plane_ptr_;
              // if (pv.point.norm() < 20 && fabs(plane->normal(2)) > 0.7) {
              //   continue;
              // }
              float dis_to_plane =
                  fabs(plane->normal(0) * p_w(0) + plane->normal(1) * p_w(1) +
                       plane->normal(2) * p_w(2) + plane->d);
              float dis_to_center =
                  (plane->center(0) - p_w(0)) * (plane->center(0) - p_w(0)) +
                  (plane->center(1) - p_w(1)) * (plane->center(1) - p_w(1)) +
                  (plane->center(2) - p_w(2)) * (plane->center(2) - p_w(2));
              float range_dis =
                  sqrt(dis_to_center - dis_to_plane * dis_to_plane);
              if (range_dis > radius_k * plane->radius) {
                return;
              }
              Eigen::Matrix<double, 1, 6> J_nq;
              J_nq.block<1, 3>(0, 0) = p_w - plane->center;
              J_nq.block<1, 3>(0, 3) = -plane->normal;
              double sigma_l = J_nq * plane->plane_var * J_nq.transpose();
              // std::cout << "sub measure dis:" << dis_to_plane
              //           << " ,3sigmal:" << 3 * sqrt(sigma_l) << ", plane var
              //           : "
              //           << plane->plane_var.block<3, 3>(0,
              //           0).diagonal().norm()
              //           << std::endl;
              sigma_l += plane->normal.transpose() * pv.var * plane->normal;
              if (pv.point.norm() > sigma_num * dis_to_plane * dis_to_plane) {
              // if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
                check_sub_plane++;
                float prob = 1 / (sqrt(2 * M_PI) * sqrt(sigma_l)) *
                             exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (prob > max_prob) {
                  max_prob = prob;
                  single_ptpl.point = pv.point;
                  single_ptpl.plane_var = plane->plane_var;
                  single_ptpl.normal = plane->normal;
                  single_ptpl.center = plane->center;
                  single_ptpl.d = plane->d;
                  single_ptpl.eigen_value = plane->min_eigen_value;
                  match_index = 1;
                }
              }
            } else {
              for (size_t j = 0; j < 8; j++) {
                OctoTree *leaf_leaf_octo = leaf_octo->leaves_[j];
                if (leaf_leaf_octo != nullptr) {
                  if (leaf_leaf_octo->plane_ptr_->is_plane) {
                    Plane *plane = leaf_leaf_octo->plane_ptr_;
                    // if (pv.point.norm() < 20 && fabs(plane->normal(2)) > 0.7)
                    // {
                    //   continue;
                    // }
                    float dis_to_plane = fabs(
                        plane->normal(0) * p_w(0) + plane->normal(1) * p_w(1) +
                        plane->normal(2) * p_w(2) + plane->d);
                    float dis_to_center = (plane->center(0) - p_w(0)) *
                                              (plane->center(0) - p_w(0)) +
                                          (plane->center(1) - p_w(1)) *
                                              (plane->center(1) - p_w(1)) +
                                          (plane->center(2) - p_w(2)) *
                                              (plane->center(2) - p_w(2));
                    float range_dis =
                        sqrt(dis_to_center - dis_to_plane * dis_to_plane);
                    if (range_dis > radius_k * plane->radius) {
                      return;
                    }
                    Eigen::Matrix<double, 1, 6> J_nq;
                    J_nq.block<1, 3>(0, 0) = p_w - plane->center;
                    J_nq.block<1, 3>(0, 3) = -plane->normal;
                    double sigma_l = J_nq * plane->plane_var * J_nq.transpose();
                    // std::cout
                    //     << "sub submeasure dis:" << dis_to_plane
                    //     << " ,3sigmal:" << 3 * sqrt(sigma_l) << ", plane var
                    //     : "
                    //     << plane->plane_var.block<3, 3>(0,
                    //     0).diagonal().norm()
                    //     << std::endl;
                    sigma_l +=
                        plane->normal.transpose() * pv.var * plane->normal;

                    if (pv.point.norm() > sigma_num * dis_to_plane * dis_to_plane) {
                    // if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
                      check_sub_sub_plane++;
                      float prob =
                          1 / (sqrt(2 * M_PI) * sqrt(sigma_l)) *
                          exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                      if (prob > max_prob) {
                        max_prob = prob;
                        single_ptpl.point = pv.point;
                        single_ptpl.plane_var = plane->plane_var;
                        single_ptpl.normal = plane->normal;
                        single_ptpl.center = plane->center;
                        single_ptpl.d = plane->d;
                        single_ptpl.eigen_value = plane->min_eigen_value;
                        match_index = 2;
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
      if (max_prob > 0) {
        ptpl_list = (single_ptpl);
        ptpl_list.is_valid = true;
        if (match_index == 0) {
          match_plane++;
        } else if (match_index == 1) {
          match_sub_plane++;
        } else if (match_index == 2) {
          match_sub_sub_plane++;
        }
      }
      else
      {
        ptpl_list.is_valid = false;
      }
    }
  // }
  
  // std::cout << "check_plane:" << check_plane << ", match_plane:" <<
  // match_plane
  //           << std::endl;
  // std::cout << "check_sub_plane:" << check_sub_plane
  //           << ", match_sub_plane:" << match_sub_plane << std::endl;
  // std::cout << "check_sub_sub_plane:" << check_sub_sub_plane
  //           << ", match_sub_sub_plane:" << match_sub_sub_plane << std::endl;
}

void BuildOptiList(const unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                   const float voxel_size, const Eigen::Matrix3d rot,
                   const Eigen::Vector3d t,
                   const PointCloudXYZI::Ptr input_cloud, const float sigma_num,
                   const std::vector<Eigen::Matrix3d> var_list,
                   std::vector<ptpl> &ptpl_list) {
  int match_plane = 0;
  int match_sub_plane = 0;
  int match_sub_sub_plane = 0;
  ptpl_list.clear();
  // old 1.25
  float radius_k1 = 1.5;
  float radius_k2 = 1.5;
  float num_sigma = sigma_num * sigma_num;
  pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < input_cloud->points.size(); i++) {
    Eigen::Vector3d p_c(input_cloud->points[i].x, input_cloud->points[i].y,
                        input_cloud->points[i].z);
    Eigen::Vector3d p_w = rot * (p_c + Lidar_T_wrt_IMU) + t;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_w[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      if (iter->second->octo_state_ == 0 &&
          iter->second->plane_ptr_->is_plane) {
        Plane *plane = iter->second->plane_ptr_;
        float dis_to_plane =
            fabs(plane->normal(0) * p_w(0) + plane->normal(1) * p_w(1) +
                 plane->normal(2) * p_w(2) + plane->d);
        float dis_to_center =
            (plane->center(0) - p_w(0)) * (plane->center(0) - p_w(0)) +
            (plane->center(1) - p_w(1)) * (plane->center(1) - p_w(1)) +
            (plane->center(2) - p_w(2)) * (plane->center(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);
        Eigen::Vector3d pq = (p_w - plane->center);
        pq = pq * dis_to_plane / pq.norm();
        if (pq[0] * pq[0] < num_sigma * var_list[i](0, 0) &&
            pq[1] * pq[1] < num_sigma * var_list[i](1, 1) &&
            pq[2] * pq[2] < num_sigma * var_list[i](2, 2) &&
            range_dis < radius_k1 * plane->radius) {
          ptpl single_ptpl;
          single_ptpl.point = p_c;
          single_ptpl.normal = plane->normal;
          single_ptpl.d = plane->d;
          single_ptpl.eigen_value = plane->min_eigen_value;
          single_ptpl.center = plane->center;
          single_ptpl.plane_var = plane->plane_var;
          ptpl_list.push_back(single_ptpl);
          match_plane++;
        }
      } else {
        for (int j = 0; j < 8; j++) {
          if (iter->second->leaves_[j] != nullptr) {
            if (iter->second->leaves_[j]->plane_ptr_->is_plane) {
              Plane *plane = iter->second->leaves_[j]->plane_ptr_;
              float dis_to_plane =
                  fabs(plane->normal(0) * p_w(0) + plane->normal(1) * p_w(1) +
                       plane->normal(2) * p_w(2) + plane->d);
              float dis_to_center =
                  (plane->center(0) - p_w(0)) * (plane->center(0) - p_w(0)) +
                  (plane->center(1) - p_w(1)) * (plane->center(1) - p_w(1)) +
                  (plane->center(2) - p_w(2)) * (plane->center(2) - p_w(2));
              float range_dis =
                  sqrt(dis_to_center - dis_to_plane * dis_to_plane);
              Eigen::Vector3d pq = (p_w - plane->center);
              // pq = pq * (dis_to_plane - sqrt(plane->min_eigen_value) / 2) /
              //      pq.norm();
              pq = pq * dis_to_plane / pq.norm();
              if (pq[0] * pq[0] < num_sigma * var_list[i](0, 0) &&
                  pq[1] * pq[1] < num_sigma * var_list[i](1, 1) &&
                  pq[2] * pq[2] < num_sigma * var_list[i](2, 2) &&
                  range_dis < radius_k2 * plane->radius) {
                ptpl single_ptpl;
                single_ptpl.point = p_c;
                single_ptpl.normal = plane->normal;
                single_ptpl.center = plane->center;
                single_ptpl.d = plane->d;
                single_ptpl.eigen_value = plane->min_eigen_value;
                single_ptpl.plane_var = plane->plane_var;
                ptpl_list.push_back(single_ptpl);
                match_sub_plane++;
                break;
              }
            } else {
              for (int k = 0; k < 8; k++) {
                if (iter->second->leaves_[j]->leaves_[k] != nullptr &&
                    iter->second->leaves_[j]
                        ->leaves_[k]
                        ->plane_ptr_->is_plane) {
                  Plane *plane =
                      iter->second->leaves_[j]->leaves_[k]->plane_ptr_;
                  float dis_to_plane = fabs(
                      plane->normal(0) * p_w(0) + plane->normal(1) * p_w(1) +
                      plane->normal(2) * p_w(2) + plane->d);
                  float dis_to_center =
                      (plane->center(0) - p_w(0)) *
                          (plane->center(0) - p_w(0)) +
                      (plane->center(1) - p_w(1)) *
                          (plane->center(1) - p_w(1)) +
                      (plane->center(2) - p_w(2)) * (plane->center(2) - p_w(2));
                  float range_dis =
                      sqrt(dis_to_center - dis_to_plane * dis_to_plane);
                  Eigen::Vector3d pq = (p_w - plane->center);
                  // pq = pq * (dis_to_plane - sqrt(plane->min_eigen_value) / 2)
                  // /
                  //      pq.norm();
                  pq = pq * dis_to_plane / pq.norm();
                  if (pq[0] * pq[0] < num_sigma * var_list[i](0, 0) &&
                      pq[1] * pq[1] < num_sigma * var_list[i](1, 1) &&
                      pq[2] * pq[2] < num_sigma * var_list[i](2, 2) &&
                      range_dis < radius_k2 * plane->radius) {
                    ptpl single_ptpl;
                    single_ptpl.point = p_c;
                    single_ptpl.normal = plane->normal;
                    single_ptpl.center = plane->center;
                    single_ptpl.d = plane->d;
                    single_ptpl.eigen_value = plane->min_eigen_value;
                    single_ptpl.plane_var = plane->plane_var;
                    ptpl_list.push_back(single_ptpl);
                    match_sub_sub_plane++;
                    break;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  std::cout << "[ Matching ]: match plane size: " << match_plane
            << " , match sub plane size: " << match_sub_plane
            << " ,match sub sub plane size:" << match_sub_sub_plane
            << std::endl;
}

void CalcQuation(const Eigen::Vector3d &vec, const int axis,
                 geometry_msgs::Quaternion &q) {
  Eigen::Vector3d x_body = vec;
  Eigen::Vector3d y_body(1, 1, 0);
  if (x_body(2) != 0) {
    y_body(2) = -(y_body(0) * x_body(0) + y_body(1) * x_body(1)) / x_body(2);
  } else {
    if (x_body(1) != 0) {
      y_body(1) = -(y_body(0) * x_body(0)) / x_body(1);
    } else {
      y_body(0) = 0;
    }
  }
  y_body.normalize();
  Eigen::Vector3d z_body = x_body.cross(y_body);
  Eigen::Matrix3d rot;

  rot << x_body(0), x_body(1), x_body(2), y_body(0), y_body(1), y_body(2),
      z_body(0), z_body(1), z_body(2);
  Eigen::Matrix3d rotation = rot.transpose();
  if (axis == 2) {
    Eigen::Matrix3d rot_inc;
    rot_inc << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    rotation = rotation * rot_inc;
  }
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void NormalToQuaternion(const Eigen::Vector3d &normal_vec,
                        geometry_msgs::Quaternion &q) {
  float CosY = normal_vec(2) / sqrt(normal_vec(0) * normal_vec(0) +
                                    normal_vec(1) * normal_vec(1));
  float CosYDiv2 = sqrt((CosY + 1) / 2);
  if (normal_vec(0) < 0)
    CosYDiv2 = -CosYDiv2;
  float SinYDiv2 = sqrt((1 - CosY) / 2);
  float CosX =
      sqrt((normal_vec(0) * normal_vec(0) + normal_vec(2) * normal_vec(2)) /
           (normal_vec(0) * normal_vec(0) + normal_vec(1) * normal_vec(1) +
            normal_vec(2) * normal_vec(2)));
  if (normal_vec(2) < 0)
    CosX = -CosX;
  float CosXDiv2 = sqrt((CosX + 1) / 2);
  if (normal_vec(1) < 0)
    CosXDiv2 = -CosXDiv2;
  float SinXDiv2 = sqrt((1 - CosX) / 2);
  q.w = CosXDiv2 * CosYDiv2;
  q.x = SinXDiv2 * CosYDiv2;
  q.y = CosXDiv2 * SinYDiv2;
  q.z = -SinXDiv2 * SinYDiv2;
}

void pubNormal(const ros::Publisher &normal_pub, const std::string normal_ns,
               const int normal_id, const pcl::PointXYZINormal normal_p,
               const float alpha, const Eigen::Vector3d rgb) {
  visualization_msgs::Marker normal;
  normal.header.frame_id = "camera_init";
  normal.header.stamp = ros::Time();
  normal.ns = normal_ns;
  normal.id = normal_id;
  normal.type = visualization_msgs::Marker::ARROW;
  normal.action = visualization_msgs::Marker::ADD;
  normal.pose.position.x = normal_p.x;
  normal.pose.position.y = normal_p.y;
  normal.pose.position.z = normal_p.z;
  geometry_msgs::Quaternion q;
  Eigen::Vector3d normal_vec(normal_p.normal_x, normal_p.normal_y,
                             normal_p.normal_z);
  CalcQuation(normal_vec, 0, q);
  normal.pose.orientation = q;
  normal.scale.x = 0.4;
  normal.scale.y = 0.05;
  normal.scale.z = 0.05;
  normal.color.a = alpha; // Don't forget to set the alpha!
  normal.color.r = rgb(0);
  normal.color.g = rgb(1);
  normal.color.b = rgb(2);
  normal.lifetime = ros::Duration();
  normal_pub.publish(normal);
}

void pubPlane(const ros::Publisher &plane_pub, const std::string plane_ns,
              const int plane_id, const pcl::PointXYZINormal normal_p,
              const float radius, const float min_eigen_value,
              const float alpha, const Eigen::Vector3d rgb) {
  visualization_msgs::Marker plane;
  plane.header.frame_id = "camera_init";
  plane.header.stamp = ros::Time();
  plane.ns = plane_ns;
  plane.id = plane_id;
  plane.type = visualization_msgs::Marker::CYLINDER;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = normal_p.x;
  plane.pose.position.y = normal_p.y;
  plane.pose.position.z = normal_p.z;
  geometry_msgs::Quaternion q;
  Eigen::Vector3d normal_vec(normal_p.normal_x, normal_p.normal_y,
                             normal_p.normal_z);
  CalcQuation(normal_vec, 2, q);
  plane.pose.orientation = q;
  plane.scale.x = 2 * radius;
  plane.scale.y = 2 * radius;
  plane.scale.z = sqrt(min_eigen_value);
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = ros::Duration();
  plane_pub.publish(plane);
}

void pubPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                 const ros::Publisher &plane_map_pub, const V3D &position) {
  int normal_id = 0;
  int plane_count = 0;
  int sub_plane_count = 0;
  int sub_sub_plane_count = 0;
  OctoTree *current_octo = nullptr;
  float plane_threshold = 0.0025;
  double dis_threshold = 100;
  ros::Rate loop(500);
  int update_count = 0;
  int id = 0;
  for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane) {
      V3D position_inc = position - iter->second->plane_ptr_->center;
      if (position_inc.norm() > dis_threshold) {
        continue;
      }
      Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);

      V3D plane_var =
          iter->second->plane_ptr_->plane_var.block<3, 3>(0, 0).diagonal();
      double trace = plane_var.sum();
      if (trace >= 0.05) {
        trace = 0.05;
      }
      trace = (0.05 - trace) / 0.05;
      uint8_t r, g, b;
      mapJet(trace, 0, 1, r, g, b);
      Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
      // if (plane_var.norm() < 0.001) {
      //   plane_rgb << 1, 0, 0;
      // } else if (plane_var.norm() < 0.005) {
      //   plane_rgb << 0, 1, 0;
      // } else {
      //   plane_rgb << 0, 0, 1;
      // }
      // plane_rgb << plane_var[0] / 0.0001, plane_var[1] / 0.0001,
      //     plane_var[2] / 0.0001;
      plane_rgb << fabs(iter->second->plane_ptr_->normal[0]),
          fabs(iter->second->plane_ptr_->normal[1]),
          fabs(iter->second->plane_ptr_->normal[2]);
      float alpha = 0.0;
      if (iter->second->plane_ptr_->is_plane) {
        alpha = 0.8;
      }
      pubNormal(plane_map_pub, "normal", iter->second->plane_ptr_->id,
                iter->second->plane_ptr_->p_center, alpha, normal_rgb);
      // loop.sleep();
      pubPlane(plane_map_pub, "plane", iter->second->plane_ptr_->id,
               iter->second->plane_ptr_->p_center,
               1.25 * iter->second->plane_ptr_->radius,
               iter->second->plane_ptr_->min_eigen_value, alpha, plane_rgb);
      loop.sleep();
      iter->second->plane_ptr_->is_update = false;
      update_count++;
      plane_count++;
    } else {
      for (uint i = 0; i < 8; i++) {
        if (iter->second->leaves_[i] != nullptr) {
          if (iter->second->leaves_[i]->plane_ptr_->is_plane) {
            // std::cout << "plane var:"
            //           << iter->second->leaves_[i]
            //                  ->plane_ptr_->plane_var.diagonal()
            //                  .transpose()
            //           << std::endl;
            V3D position_inc =
                position - iter->second->leaves_[i]->plane_ptr_->center;
            // if (position_inc.norm() > dis_threshold) {
            //   continue;
            // }
            Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);

            V3D plane_var = iter->second->leaves_[i]
                                ->plane_ptr_->plane_var.block<3, 3>(0, 0)
                                .diagonal();
            double trace = plane_var.sum();
            if (trace >= 0.05) {
              trace = 0.05;
            }
            trace = (0.05 - trace) / 0.05;
            uint8_t r, g, b;
            mapJet(trace, 0, 1, r, g, b);
            Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
            plane_rgb << fabs(iter->second->leaves_[i]->plane_ptr_->normal[0]),
                fabs(iter->second->leaves_[i]->plane_ptr_->normal[1]),
                fabs(iter->second->leaves_[i]->plane_ptr_->normal[2]);
            float alpha = 0.0;
            if (iter->second->leaves_[i]->plane_ptr_->is_plane) {
              alpha = 0.8;
            }
            pubNormal(plane_map_pub, "normal",
                      iter->second->leaves_[i]->plane_ptr_->id,
                      iter->second->leaves_[i]->plane_ptr_->p_center, alpha,
                      normal_rgb);
            // loop.sleep();
            pubPlane(plane_map_pub, "plane",
                     iter->second->leaves_[i]->plane_ptr_->id,
                     iter->second->leaves_[i]->plane_ptr_->p_center,
                     1.25 * iter->second->leaves_[i]->plane_ptr_->radius,
                     iter->second->leaves_[i]->plane_ptr_->min_eigen_value,
                     alpha, plane_rgb);
            loop.sleep();
            iter->second->leaves_[i]->plane_ptr_->is_update = false;
            update_count++;

            sub_plane_count++;
            normal_id++;
            // loop.sleep();
          } else {
            OctoTree *temp_octo_tree = iter->second->leaves_[i];
            for (uint j = 0; j < 8; j++) {
              if (temp_octo_tree->leaves_[j] != nullptr) {
                if (temp_octo_tree->leaves_[j]->octo_state_ == 0 &&
                    temp_octo_tree->leaves_[j]->plane_ptr_->is_plane) {
                  // std::cout << "plane var:"
                  //           << temp_octo_tree->leaves_[j]
                  //                  ->plane_ptr_->plane_var.diagonal()
                  //                  .transpose()
                  //           << std::endl;
                  V3D position_inc =
                      position - temp_octo_tree->leaves_[j]->plane_ptr_->center;
                  // if (position_inc.norm() > dis_threshold) {
                  //   continue;
                  // }
                  if (temp_octo_tree->leaves_[j]->plane_ptr_->is_plane) {
                    // std::cout << "subsubplane" << std::endl;
                    Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);
                    V3D plane_var =
                        temp_octo_tree->leaves_[j]
                            ->plane_ptr_->plane_var.block<3, 3>(0, 0)
                            .diagonal();
                    double trace = plane_var.sum();
                    if (trace >= 0.05) {
                      trace = 0.05;
                    }
                    trace = (0.05 - trace) / 0.05;
                    uint8_t r, g, b;
                    mapJet(trace, 0, 1, r, g, b);
                    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                    pubNormal(plane_map_pub, "normal",
                              iter->second->leaves_[i]->plane_ptr_->id,
                              temp_octo_tree->leaves_[j]->plane_ptr_->p_center,
                              0.8, normal_rgb);
                    pubPlane(
                        plane_map_pub, "plane",
                        temp_octo_tree->leaves_[j]->plane_ptr_->id,
                        temp_octo_tree->leaves_[j]->plane_ptr_->p_center,
                        temp_octo_tree->leaves_[j]->plane_ptr_->radius,
                        temp_octo_tree->leaves_[j]->plane_ptr_->min_eigen_value,
                        0.8, plane_rgb);
                    loop.sleep();
                    temp_octo_tree->leaves_[j]->plane_ptr_->is_update = false;
                    update_count++;
                  }
                  sub_sub_plane_count++;
                  // loop.sleep();
                }
              }
            }
          }
        }
      }
    }
  }
  cout << "[Map Info] Plane counts:" << plane_count
       << " Sub Plane counts:" << sub_plane_count
       << " Sub Sub Plane counts:" << sub_sub_plane_count << endl;
  cout << "[Map Info] Update plane counts:" << update_count << endl;
}

// Similar with PCL voxelgrid filter
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size) {
  int intensity = rand() % 255;
  if (voxel_size < 0.01) {
    return;
  }
  std::unordered_map<VOXEL_LOC, M_POINT> feat_map;
  uint plsize = pl_feat.size();

  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = pl_feat[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.intensity += p_c.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.intensity = p_c.intensity;
      anp.count = 1;
      feat_map[position] = anp;
    }
  }
  plsize = feat_map.size();
  pl_feat.clear();
  pl_feat.resize(plsize);

  uint i = 0;
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) 
  {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    pl_feat[i].intensity = iter->second.intensity / iter->second.count;
    i++;
  }
}

void calcBodyVar(const Eigen::Vector3d &pb, const float range_inc,
                 const float degree_inc, Eigen::Matrix3d &var) {
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
      pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0,
      -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1,
                               -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
      base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  var = direction * range_var * direction.transpose() +
        A * direction_var * A.transpose();
};

// struct LidarPlaneSolver {
//   LidarPlaneSolver(Eigen::Vector3d curr_point_,
//                    Eigen::Vector3d plane_unit_norm_,
//                    double negative_OA_dot_norm_)
//       : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
//         negative_OA_dot_norm(negative_OA_dot_norm_){};

//   template <typename T>
//   bool operator()(const T *q, const T *t, T *residual) const {
//     Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
//     Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
//     Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
//                               T(curr_point.z())};
//     Eigen::Matrix<T, 3, 1> point_w;
//     point_w = q_w_curr * cp + t_w_curr;
//     Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()),
//                                 T(plane_unit_norm.z()));
//     residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
//     return true;
//   }
//   static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
//                                      const Eigen::Vector3d plane_unit_norm_,
//                                      const double negative_OA_dot_norm_) {
//     return (new ceres::AutoDiffCostFunction<LidarPlaneSolver, 1, 4, 3>(
//         new LidarPlaneSolver(curr_point_, plane_unit_norm_,
//                              negative_OA_dot_norm_)));
//   }

//   Eigen::Vector4d plane_params;
//   Eigen::Vector3d curr_point;
//   Eigen::Vector3d plane_unit_norm;
//   double negative_OA_dot_norm;
// };

#endif