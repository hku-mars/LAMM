// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
// #include <common_lib.h>-
#include <algorithm>
#include <chrono>
#include <execution>
#include "IMU_Processing.hpp"
#include "laserMapping.h"
#include "DynObjFilter.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/CompressedImage.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <m_detector/States.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"

#ifdef USE_ikdforest
#include <ikd-Forest/ikd_Forest.h>
#else

#include <ikd-Tree/ikd_Tree.h>

#endif

#ifndef DEPLOY

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
#endif

#define INIT_TIME           (0.0)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)


float DET_RANGE = 300.0f;
#ifdef USE_ikdforest
const int laserCloudWidth  = 200;
const int laserCloudHeight = 200;
const int laserCloudDepth  = 200;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

#else
const float MOV_THRESHOLD = 1.5f;
#endif

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;
string m_detector_file = "";

// int iterCount, feats_down_size, NUM_MAX_ITERATIONS, laserCloudValidNum,\
//     effct_feat_num, time_log_counter, publish_count = 0;

int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, \
 effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0, pose_ind = 0;

int QUAD_LAYER_MAX = 1;
int dyn_windows_num = 3;
int occlude_windows = 3;
float VER_RESOLUTION_MAX = 0.01;
float HOR_RESOLUTION_MAX = 0.01;
float angle_noise = 0.001;
float angle_occlude = 0.02;
float dyn_windows_dur = 0.5;

double res_mean_last = 0.05;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double init_vel_x = 0.0, init_vel_y = 0.0, init_vel_z = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;

// Time Log Variables
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
int kdtree_delete_counter = 0, kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double search_time_rec[100000];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;

double max_voxel_size, min_eigen_value = 0.003, match_s = 0.90, sigma_num = 2.0, match_eigen_value = 0.0025;
int scanIdx = 0, layer = 0, lidar_type, pcd_save_interval = -1, pcd_index = 0;;
bool lidar_pushed, flg_reset, flg_exit = false, flg_EKF_inited;
bool use_new_map = true, is_pub_plane_map = false, imu_en = true, time_sync_en = false;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool runtime_pos_log = false, pcd_save_en = false, extrinsic_est_en = true, path_en = true, dyn_filter_en = true, dyn_filter_dbg_en = true, is_add = false;

vector<BoxPointType> cub_needrm;

// deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
visualization_msgs::MarkerArray clusters_gt;
deque<visualization_msgs::MarkerArray> cluster_buffer;
// deque<sensor_msgs::CompressedImage> image_buffer;
vector<vector<int>> pointSearchInd_surf;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
bool point_selected_surf[100000] = {0};
float res_last[100000] = {0.0};
double total_residual;
int point_index = 0;
string out_file = "";
string out_file_origin = "";
string label_folder = "";
string pose_log_file = "";
bool pose_log_flag = false;
int cur_frame = 0;
state_ikfom state_point;

//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
#ifndef USE_ikdforest
PointCloudXYZI::Ptr _featsArray;
#endif
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

#ifdef USE_ikdforest
KD_FOREST ikdforest;
#else
KD_TREE ikdtree;
#endif

M3D last_rot(M3D::Zero());
V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D last_odom(Zero3d);
// avia
V3D layer_size(6, 5, 5);
// velodyne
// V3D layer_size(10, 6, 6);

//estimator inputs and output;
MeasureGroup Measures;
#ifdef USE_IKFOM
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;
#else
StatesGroup state;
StatesGroup last_state;
#endif

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;


shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());

void SigHandle(int sig) {
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp) {
#ifdef USE_IKFOM
    //state_ikfom write_state = kf.get_x();
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
#else
    V3D rot_ang(Log(state.rot_end));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state.pos_end(0), state.pos_end(1), state.pos_end(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state.vel_end(0), state.vel_end(1), state.vel_end(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state.bias_g(0), state.bias_g(1), state.bias_g(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state.bias_a(0), state.bias_a(1), state.bias_a(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state.gravity(0), state.gravity(1), state.gravity(2)); // Bias_a  
    fprintf(fp, "\r\n");
    fflush(fp);
#endif
}

#ifdef USE_IKFOM
//project the lidar scan to world frame
void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->normal_x = pi->normal_x;
    po->normal_y = pi->normal_y;
    po->normal_z = pi->normal_z;
    po->intensity = pi->intensity;
}
#endif

void pointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
#ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
#else
    V3D p_global(state.rot_end * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + state.pos_end);
#endif

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->normal_x = pi->normal_x;
    po->normal_y = pi->normal_y;
    po->normal_z = pi->normal_z;
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
    V3D p_body(pi[0], pi[1], pi[2]);
#ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
#else
    V3D p_global(state.rot_end * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + state.pos_end);
#endif
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointTypeRGB *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
#ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
#else
    V3D p_global(state.rot_end * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + state.pos_end);
#endif
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    // po->r = pi->normal_x;
    // po->g = pi->normal_y;
    // po->b = pi->normal_z;

    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);

    int reflection_map = intensity * 10000;
}

int points_cache_size = 0;

#ifndef USE_ikdforest

void points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
    for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

#endif

#ifndef USE_ikdforest
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

void lasermap_fov_segment() {
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
#ifdef USE_IKFOM
    //state_ikfom fov_state = kf.get_x();
    //V3D pos_LiD = fov_state.pos + fov_state.rot * fov_state.offset_T_L_I;
    V3D pos_LiD = pos_lid;
#else
    V3D pos_LiD = state.pos_end;
#endif
    if (!Localmap_Initialized) {
        //if (cube_len <= 2.0 * MOV_THRESHOLD * DET_RANGE) throw std::invalid_argument("[Error]: Local Map Size is too small! Please change parameter \"cube_side_length\" to larger than %d in the launch file.\n");
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    // printf("Local Map is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", LocalMap_Points.vertex_min[0],LocalMap_Points.vertex_max[0],LocalMap_Points.vertex_min[1],LocalMap_Points.vertex_max[1],LocalMap_Points.vertex_min[2],LocalMap_Points.vertex_max[2]);
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);                     
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
    printf("Delete time: %0.6f, delete size: %d\n", kdtree_delete_time, kdtree_delete_counter);
    // printf("Delete Box: %d\n",int(cub_needrm.size()));
}

#endif

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);

    // std::cout << "Going to push time " << msg->header.stamp.toSec() << " into time_buffer." << std::endl;  

    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    if (time_sync_en && !timediff_set_flg && !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 1 && !imu_buffer.empty() &&
        !lidar_buffer.empty()) {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
               last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (time_sync_en) {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void GTBB_cbk(const visualization_msgs::MarkerArray::ConstPtr &msg_in) {
    mtx_buffer.lock();
    clusters_gt = *msg_in;
    cluster_buffer.push_back(clusters_gt);
    mtx_buffer.unlock();
}

// void image_cbk(const sensor_msgs::CompressedImage::ConstPtr &msg_in)
// {   
//     sensor_msgs::CompressedImage tmp = *msg_in;
//     image_buffer.push_back(tmp);
// }

bool sync_packages(MeasureGroup &meas) {
    if (!imu_en) {
        if (!lidar_buffer.empty()) {
            // cout<<"meas.lidar->points.size(): "<<meas.lidar->points.size()<<endl;
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            lidar_end_time = meas.lidar_beg_time;
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            return true;
        }

        return false;
    }

    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        if (meas.lidar->points.size() <= 1) {
            lidar_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        if (lidar_type == L515) {
            lidar_end_time = meas.lidar_beg_time;
        } else {
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        }
        std::cout << "lidar_end_time = " << meas.lidar_beg_time << " + "
                  << meas.lidar->points.back().curvature / double(1000) << std::endl;

        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;

void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++) {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
#ifdef USE_ikdforest
    ikdforest.Add_Points(PointToAdd, lidar_end_time);
#else
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
#endif
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB(size, 1));
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        int line_index = 1;
        double last_horizon_angle = 1.0;
        int line_index_change = 0;
        M3D delta_R = last_state.rot_end.transpose() * state.rot_end;
        double odom_theta = Log(delta_R)(2);
        double angle_correct = 0.001;
        bool jump_flag = false;
        for (int i = 0; i < size; i++) {
            if (lidar_type == L515) {
                V3D p_body(laserCloudFullRes->points[i].x, laserCloudFullRes->points[i].y,
                           laserCloudFullRes->points[i].z);
                double horizon_angle = atan2f(double(p_body(1)), double(p_body(0)));
                double vertical_angle = atan2f(double(p_body(2)),
                                               sqrt(pow(double(p_body(0)), 2) + pow(double(p_body(1)), 2)));
                if (horizon_angle >= 0.0 && horizon_angle < 1.0 && last_horizon_angle <= 0.0 &&
                    last_horizon_angle > -1.0 && line_index_change > 100) {
                    line_index++;
                    line_index_change = 0;
                    jump_flag = false;
                } else line_index_change++;
                if (last_horizon_angle < -2.5 && horizon_angle > 2.5) jump_flag = true;
                // cout << "point " << i << " " << "line: " << line_index << " horizon anlge: " << horizon_angle << " vertical_angle: " << vertical_angle << endl;
                bool undis_p_right = false;
                bool undis_p_left = false;
                if (odom_theta > 0) {
                    if (line_index < 33) {
                        switch (line_index % 12) {
                            case 0:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.058 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 1:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.020 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 2:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.012 - angle_correct))
                                    undis_p_right = true;
                                break;
                            case 3:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.048 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 4:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.084 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 5:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.081 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 6:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.047 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 7:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.006 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 8:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.024 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 9:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.060 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 10:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.096 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 11:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.0935 - angle_correct))
                                    undis_p_left = true;
                                break;
                        }
                    } else {
                        switch ((line_index - 32) % 12) {
                            case 0:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.058 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 1:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.020 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 2:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.012 - angle_correct))
                                    undis_p_right = true;
                                break;
                            case 3:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.048 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 4:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.084 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 5:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.081 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 6:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.047 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 7:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.006 - angle_correct))
                                    undis_p_left = true;
                                break;
                            case 8:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.024 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 9:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.060 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 10:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.096 - angle_correct))
                                    undis_p_right = true;;
                                break;
                            case 11:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.0935 - angle_correct))
                                    undis_p_left = true;
                                break;
                        }
                    }
                } else {
                    if (line_index < 33) {
                        switch (line_index % 12) {
                            case 0:
                                if (jump_flag && (horizon_angle <= -(PI - 0.5 * odom_theta - 0.058 - angle_correct) ||
                                                  horizon_angle >= PI + 0.5 * odom_theta))
                                    undis_p_left = true;
                                break;
                            case 1:
                                if (jump_flag && (horizon_angle <= -(PI - 0.5 * odom_theta - 0.020 - angle_correct) ||
                                                  horizon_angle >= PI + 0.5 * odom_theta))
                                    undis_p_left = true;
                                break;
                            case 2:
                                if (!jump_flag && (horizon_angle >= (PI - 0.5 * odom_theta - 0.012 - angle_correct) ||
                                                   horizon_angle <= -(PI + 0.5 * odom_theta)))
                                    undis_p_right = true;
                                break;
                            case 3:
                                if (!jump_flag && (horizon_angle >= (PI - 0.5 * odom_theta - 0.048 - angle_correct) ||
                                                   horizon_angle <= -(PI + 0.5 * odom_theta)))
                                    undis_p_right = true;;
                                break;
                            case 4:
                                if (!jump_flag && (horizon_angle >= (PI - 0.5 * odom_theta - 0.084 - angle_correct) ||
                                                   horizon_angle <= -(PI + 0.5 * odom_theta)))
                                    undis_p_right = true;;
                                break;
                            case 5:
                                if (jump_flag && (horizon_angle <= -(PI - 0.5 * odom_theta - 0.081 - angle_correct) ||
                                                  horizon_angle >= PI + 0.5 * odom_theta))
                                    undis_p_left = true;
                                break;
                            case 6:
                                if (jump_flag && (horizon_angle <= -(PI - 0.5 * odom_theta - 0.047 - angle_correct) ||
                                                  horizon_angle >= PI + 0.5 * odom_theta))
                                    undis_p_left = true;
                                break;
                            case 7:
                                if (jump_flag && (horizon_angle <= -(PI - 0.5 * odom_theta - 0.006 - angle_correct) ||
                                                  horizon_angle >= PI + 0.5 * odom_theta))
                                    undis_p_left = true;
                                break;
                            case 8:
                                if (!jump_flag && (horizon_angle >= (PI - 0.5 * odom_theta - 0.024 - angle_correct) ||
                                                   horizon_angle <= -(PI + 0.5 * odom_theta)))
                                    undis_p_right = true;;
                                break;
                            case 9:
                                if (!jump_flag && (horizon_angle >= (PI - 0.5 * odom_theta - 0.060 - angle_correct) ||
                                                   horizon_angle <= -(PI + 0.5 * odom_theta)))
                                    undis_p_right = true;;
                                break;
                            case 10:
                                if (!jump_flag && (horizon_angle >= (PI - 0.5 * odom_theta - 0.096 - angle_correct) ||
                                                   horizon_angle <= -(PI + 0.5 * odom_theta)))
                                    undis_p_right = true;;
                                break;
                            case 11:
                                if (jump_flag && (horizon_angle <= -(PI - 0.5 * odom_theta - 0.0935 - angle_correct) ||
                                                  horizon_angle >= PI + 0.5 * odom_theta))
                                    undis_p_left = true;
                                break;
                        }
                    } else {
                        switch ((line_index - 32) % 12) {
                            case 0:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.058 - angle_correct) ||
                                    horizon_angle >= PI + 0.5 * odom_theta)
                                    undis_p_left = true;
                                break;
                            case 1:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.020 - angle_correct) ||
                                    horizon_angle >= PI + 0.5 * odom_theta)
                                    undis_p_left = true;
                                break;
                            case 2:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.012 - angle_correct) ||
                                    horizon_angle <= -(PI + 0.5 * odom_theta))
                                    undis_p_right = true;
                                break;
                            case 3:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.048 - angle_correct) ||
                                    horizon_angle <= -(PI + 0.5 * odom_theta))
                                    undis_p_right = true;;
                                break;
                            case 4:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.084 - angle_correct) ||
                                    horizon_angle <= -(PI + 0.5 * odom_theta))
                                    undis_p_right = true;;
                                break;
                            case 5:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.081 - angle_correct) ||
                                    horizon_angle >= PI + 0.5 * odom_theta)
                                    undis_p_left = true;
                                break;
                            case 6:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.047 - angle_correct) ||
                                    horizon_angle >= PI + 0.5 * odom_theta)
                                    undis_p_left = true;
                                break;
                            case 7:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.006 - angle_correct) ||
                                    horizon_angle >= PI + 0.5 * odom_theta)
                                    undis_p_left = true;
                                break;
                            case 8:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.024 - angle_correct) ||
                                    horizon_angle <= -(PI + 0.5 * odom_theta))
                                    undis_p_right = true;;
                                break;
                            case 9:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.060 - angle_correct) ||
                                    horizon_angle <= -(PI + 0.5 * odom_theta))
                                    undis_p_right = true;;
                                break;
                            case 10:
                                if (horizon_angle >= (PI - 0.5 * odom_theta - 0.096 - angle_correct) ||
                                    horizon_angle <= -(PI + 0.5 * odom_theta))
                                    undis_p_right = true;;
                                break;
                            case 11:
                                if (horizon_angle <= -(PI - 0.5 * odom_theta - 0.0935 - angle_correct) ||
                                    horizon_angle >= PI + 0.5 * odom_theta)
                                    undis_p_left = true;
                                break;
                        }
                    }
                }
                if (undis_p_right) {
                    laserCloudWorldRGB->points[i].r = 255;
                    feats_undistort->points[i].intensity = 666;
                    double r = sqrt(p_body(0) * p_body(0) + p_body(1) * p_body(1));
                    horizon_angle += odom_theta;
                    laserCloudFullRes->points[i].x = r * cos(horizon_angle);
                    laserCloudFullRes->points[i].y = r * sin(horizon_angle);
                } else if (undis_p_left) {
                    laserCloudWorldRGB->points[i].b = 255;
                    feats_undistort->points[i].intensity = 666;
                    double r = sqrt(p_body(0) * p_body(0) + p_body(1) * p_body(1));
                    horizon_angle -= odom_theta;
                    laserCloudFullRes->points[i].x = r * cos(horizon_angle);
                    laserCloudFullRes->points[i].y = r * sin(horizon_angle);
                } else {
                    feats_undistort->points[i].intensity = 0;
                    laserCloudWorldRGB->points[i].g = 255;
                }
                // laserCloudWorldRGB->points[i].r = 100 * (line_index % 3);
                // laserCloudWorldRGB->points[i].g = 100 * ((line_index + 1) % 3);
                // laserCloudWorldRGB->points[i].b = 100 * ((line_index + 2) % 3);
                last_horizon_angle = horizon_angle;

            }
            if (lidar_type == L515)
                RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorldRGB->points[i]);
            else
                pointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }


        sensor_msgs::PointCloud2 laserCloudmsg;
        if (lidar_type == L515)
            pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
        else
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }


    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++) {
            pointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(m_detector_file + "/" + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher &pubLaserCloudFullRes_body) {
    PointCloudXYZI::Ptr laserCloudFullRes(feats_undistort);
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_undistort, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes_body.publish(laserCloudmsg);
}

void publish_cluster_world(const ros::Publisher &cluster_gt_pub, ofstream &fout_pos) {
    Eigen::Quaterniond q(state.rot_end);
    pose_ind++;
    if (!cluster_buffer.empty()) {
        // visualization_msgs::MarkerArray clusters_front = cluster_buffer.back();
        visualization_msgs::MarkerArray clusters_front = clusters_gt;
        for (int i = 0; i < clusters_front.markers.size(); i++) {
            for (int j = 0; j < clusters_front.markers[i].points.size(); j++) {
                V3D point(clusters_front.markers[i].points[j].x, clusters_front.markers[i].points[j].y,
                          clusters_front.markers[i].points[j].z);
                V3D point_gl(state.rot_end * (Lidar_R_wrt_IMU * point + Lidar_T_wrt_IMU) + state.pos_end);
                clusters_front.markers[i].points[j].x = point_gl(0);
                clusters_front.markers[i].points[j].y = point_gl(1);
                clusters_front.markers[i].points[j].z = point_gl(2);
            }
        }
        cluster_gt_pub.publish(clusters_front);
        cluster_buffer.pop_back();
    }
    fout_pos << pose_ind << " " << 0.00 << " " << state.pos_end(0) << " " << state.pos_end(1) << " " << state.pos_end(2)
             << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

void publish_effect_world(const ros::Publisher &pubLaserCloudEffect) {
    PointCloudXYZI::Ptr laserCloudWorld(\
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++) {
        pointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    // laserCloudFullRes3.header.stamp = ros::Time::now();
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher &pubLaserCloudMap) {
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = ros::Time::now();
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T &out) {
#ifdef USE_IKFOM
    //state_ikfom stamp_state = kf.get_x();
    out.position.x = state_point.pos(0);
    out.position.y = state_point.pos(1);
    out.position.z = state_point.pos(2);
#else
    out.position.x = state.pos_end(0);
    out.position.y = state.pos_end(1);
    out.position.z = state.pos_end(2);
#endif
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    // odomAftMapped.header.stamp = ros::Time::now();//ros::Time().fromSec(lidar_end_time);
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose.pose);

    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped"));
}

void publish_mavros(const ros::Publisher &mavros_pose_publisher) {
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);

    msg_body_pose.header.frame_id = "camera_odom_frame";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void publish_path(const ros::Publisher pubPath) {
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";
    static int jjj = 0;
    jjj++;
    if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

#ifdef USE_IKFOM
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];
        //double search_start = omp_get_wtime();
        /* transform to world frame */
        //pointBodyToWorld_ikfom(&point_body, &point_world, s);
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
#ifdef USE_ikdforest
                uint8_t search_flag = 0;
                search_flag = ikdforest.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, first_lidar_time, 5);
#else
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
#endif

            if (points_near.size() < NUM_MATCH_POINTS)
            {
                point_selected_surf[i] = false;
            }
            else
            {
                point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            }

#ifdef USE_ikdforest
                point_selected_surf[i] = point_selected_surf[i] && (search_flag == 0);
#endif
        }

        //kdtree_search_time += omp_get_wtime() - search_start;

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) //(planeValid)
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    // cout<<"pca time test: "<<pca_time1<<" "<<pca_time2<<endl;
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    res_mean_last = total_residual / effct_feat_num;
    // cout << "[ mapping ]: Effective feature num: "<<effct_feat_num<<" res_mean_last "<<res_mean_last<<endl;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    //MatrixXd H(effct_feat_num, 23);
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num); // = VectorXd::Zero(effct_feat_num);
    //VectorXd meas_vec(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C); // s.rot.conjugate() * norm_vec);
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
        //H.row(i) = Eigen::Matrix<double, 1, 23>::Zero();
        ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        //ekfom_data.h_x.block<1, 3>(i, 6) << VEC_FROM_ARRAY(A);
        //ekfom_data.h_x.block<1, 6>(i, 17) << VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);

        /*** Measuremnt: distance to the closest surface/corner ***/
        //meas_vec(i) = - norm_p.intensity;
        ekfom_data.h(i) = -norm_p.intensity;
    }
    //ekfom_data.h_x =H;
    solve_time += omp_get_wtime() - solve_start_;
    //return meas_vec;
}
#endif

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    // pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

    nh.param<string>("dyn_obj/out_file", out_file, "");
    nh.param<string>("dyn_obj/out_file_origin", out_file_origin, "");

    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<string>("map_file_path", map_file_path, "");
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh.param<double>("mapping/fov_degree", fov_deg, 180);
    nh.param<bool>("mapping/imu_en", imu_en, true);
    nh.param<double>("mapping/init_vel_x", init_vel_x, 0.0);
    nh.param<double>("mapping/init_vel_y", init_vel_y, 0.0);
    nh.param<double>("mapping/init_vel_z", init_vel_z, 0.0);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 1.0);
    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<bool>("preprocess/feature_extract_en", p_pre->feature_enabled, 0);
    nh.param<bool>("voxel/voxel_map_en", use_new_map, 0);
    nh.param<bool>("voxel/pub_plane_en", is_pub_plane_map, 0);
    nh.param<double>("voxel/match_eigen_value", match_eigen_value, 0.0025);
    nh.param<int>("voxel/layer", layer, 1);
    nh.param<double>("voxel/match_s", match_s, 0.90);
    nh.param<double>("voxel/voxel_size", max_voxel_size, 1.0);
    nh.param<double>("voxel/min_eigen_value", min_eigen_value, 0.01);
    nh.param<double>("voxel/sigma_num", sigma_num, 3);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, 1);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, 1);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, 1);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, true);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<string>("dyn_obj/m_detector_file", m_detector_file, "");
    cout << "lidar_type: " << lidar_type << endl;
    ros::Publisher pub_pcl_dyn_extend = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl_dyn_extend", 10000);
    ros::Publisher cluster_vis_high = nh.advertise<visualization_msgs::MarkerArray>("/cluster_vis_high", 10000);
    ros::Publisher pub_ground_points = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10000);
    ros::Publisher cluster_gt_pub = nh.advertise<visualization_msgs::MarkerArray>
            ("cluster_vis_gt_gl", 100000);
    ros::Publisher demo_pcl_display = nh.advertise<sensor_msgs::PointCloud2>
            ("/demo_display", 100000);

    // path.header.stamp    = ros::Time::now();
    path.header.stamp = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = "camera_init";

    /***-------------- cout for multi-map ---------------***/

    string m_detector_file_all = m_detector_file + "odom.txt";
    std::cout << "m_detector file:" << m_detector_file_all << std::endl;
    std::ofstream m_file(m_detector_file_all);

    /*** variables definition ***/
#ifndef USE_IKFOM
    VD(DIM_STATE) solution;
    MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
    V3D rot_add, t_add;
    StatesGroup state_propagat;
    PointType pointOri, pointSel, coeff;
#endif
    //PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

#ifndef USE_ikdforest
    _featsArray.reset(new PointCloudXYZI());
#endif

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    DynObjFilt->SetParam(nh);
    DynObjFilt->Cluster.Init(pub_pcl_dyn_extend, cluster_vis_high, pub_ground_points);
#ifdef USE_ikdforest
    ikdforest.Set_balance_criterion_param(0.6);
    ikdforest.Set_delete_criterion_param(0.5);
    ikdforest.Set_environment(laserCloudDepth,laserCloudWidth,laserCloudHeight,cube_len);
    ikdforest.Set_downsample_param(filter_size_map_min);
#endif

    shared_ptr<ImuProcess> p_imu(new ImuProcess());
    // p_imu->set_extrinsic(V3D(0.04165, 0.02326, -0.0284));   //avia
    // p_imu->set_extrinsic(V3D(0.05512, 0.02226, -0.0297));   //horizon
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    state.vel_end << init_vel_x, init_vel_y, init_vel_z;
#ifndef USE_IKFOM
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
#endif

#ifdef USE_IKFOM
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
#endif
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_pre, fout_out, fout_dbg, fout_pos;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
    if (pose_log_flag) {
        fout_pos.open(pose_log_file, ios::out);
        fout_pos << 0 << " " << 0.000000001 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "
                 << 0.0 << " " << 1.0 << std::endl;
    }

    if (fout_pre && fout_out)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

#ifdef USE_ikdforest
    ikdforest.Set_balance_criterion_param(0.6);
    ikdforest.Set_delete_criterion_param(0.5);
#endif

    /*** For New Map ***/
    int pub_map = 0;
    int last_match_num = 0;
    bool init_map = false;
    int min_points_size = 30;
    std::time_t startTime, endTime;
    std::unordered_map<VOXEL_LOC, OctoTree *> feat_map;
    last_rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_bb = nh.subscribe("/cluster_vis_gt", 100, GTBB_cbk);
    // ros::Subscriber sub_image = nh.subscribe("/camera/image_color/compressed", 10000, image_cbk);
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    // ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/cloud_dyn_obj", 100000);
    ros::Publisher pubLaserCloudEffect_depth = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_dyn_obj_removed", 100000);
    ros::Publisher pubLaserCloudhist_depth = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected_hist", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
            ("/aft_mapped_to_init", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>
            ("/path", 100000);
    ros::Publisher plane_pub = nh.advertise<visualization_msgs::Marker>
            ("/planner_normal", 1);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::CompressedImage>
            ("/demo_image/compressed", 100000);
#ifdef DEPLOY
    ros::Publisher mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
#endif
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status) {
        if (flg_exit) break;
        ros::spinOnce();
        if (sync_packages(Measures)) {
            if (flg_reset) {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();
#ifdef USE_IKFOM
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
#else
            last_state = state;
            p_imu->Process(Measures, state, feats_undistort);
            state_propagat = state;
#endif

            if (feats_undistort->empty() || (feats_undistort == NULL)) {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                cout << "FAST-LIO not ready" << endl;
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;

            if (use_new_map && !init_map) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
                        new pcl::PointCloud<pcl::PointXYZI>);
                Eigen::Quaterniond q(state.rot_end);

                // std::cout << "Begin build unorder map" << std::endl;
                transformLidar(state.rot_end, state.pos_end, feats_undistort,
                               world_lidar);
                std::vector<pointWithVar> pv_list;
                pv_list.reserve(world_lidar->size());
                for (size_t i = 0; i < world_lidar->size(); i++) {
                    pointWithVar pv;
                    pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
                            world_lidar->points[i].z;
                    V3D point_this(feats_undistort->points[i].x,
                                   feats_undistort->points[i].y,
                                   feats_undistort->points[i].z);
                    M3D var;
                    if (point_this.norm() > 5) {
                        calcBodyVar(point_this, 0.05, 0.05, var);
                    } else {
                        calcBodyVar(point_this, 0.05, 0.05, var);
                    }
                    // point_this += Lidar_offset_to_IMU;
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);
                    var = state.rot_end * var * state.rot_end.transpose() +
                          (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
                          (-point_crossmat).transpose() +
                          state.cov.block<3, 3>(3, 3);
                    pv.var = var;
                    pv_list.push_back(pv);
                }
                startTime = clock();
                // std::cout << " init rot var:" << std::endl
                //           << state.cov.block<3, 3>(0, 0) << std::endl;
                buildUnorderMap(pv_list, max_voxel_size, layer_size, min_eigen_value,
                                feat_map);
                scanIdx++;
                endTime = clock();
                // std::cout << "Build unorderMap,init points size:" <<
                // world_lidar->size()
                //           << " Time cost:"
                //           << (double)(endTime - startTime) / CLOCKS_PER_SEC * 1000
                //           << "ms" << std::endl;
                dump_lio_state_to_log(fp);
                // kitti_log(fp_kitti);
                if (is_pub_plane_map) pubPlaneMap(feat_map, plane_pub, state.pos_end);
                init_map = true;
                string file_name = out_file;
                stringstream ss;
                ss << setw(6) << setfill('0') << cur_frame;
                file_name += ss.str();
                file_name.append(".label");
                string file_name_origin = out_file_origin;
                stringstream sss;
                sss << setw(6) << setfill('0') << cur_frame;
                file_name_origin += sss.str();
                file_name_origin.append(".label");

                if (file_name.length() > 15 || file_name_origin.length() > 15)
                    DynObjFilt->set_path(file_name, file_name_origin);
                // DynObjFilt->publish_img(image_pub, image_buffer, lidar_end_time);
#ifdef USE_IKFOM
                m_file << std::fixed << std::setprecision(6) << "1odomAftMapped.header.stamp"
                         << std::setprecision(7) << " " << state.pos_end[0]
                         << " " << state.pos_end[1] << " "
                         << state.pos_end[2] << " " << state.rot_end(0, 0) << " "
                         << state.rot_end(1, 1) << " " << state.rot_end(2, 2) << " "
                         << std::endl;
                DynObjFilt->filter(feats_undistort, state_point.rot.toRotationMatrix(), state_point.pos, lidar_end_time);
#else
                // DynObjFilt->publish_img(image_pub, image_buffer, lidar_end_time);
                //**************************************
                m_file << std::fixed << std::setprecision(6) << "2odomAftMapped.header.stamp"
                       << std::setprecision(7) << " " << state.pos_end[0]
                       << " " << state.pos_end[1] << " "
                       << state.pos_end[2] << " " << state.rot_end(0, 0) << " "
                       << state.rot_end(1, 1) << " " << state.rot_end(2, 2) << " "
                       << std::endl;
                DynObjFilt->filter(feats_undistort, state.rot_end, state.pos_end,
                                   lidar_end_time); //state.offset_R_L_I , state.offset_T_L_I,
#endif
                DynObjFilt->publish_hist(pubLaserCloudhist_depth, lidar_end_time);
                // DynObjFilt->ReadFromLabel(pubLaserCloudEffect, feats_undistort, state.rot_end, state.pos_end, label_folder, cur_frame, lidar_end_time);
                DynObjFilt->publish_dyn(pubLaserCloudEffect, pubLaserCloudEffect_depth, lidar_end_time);

                publish_frame_body(pubLaserCloudFullRes_body);
                cur_frame += 1;
                continue;
            }

            /*** Segment the map in lidar FOV ***/
#ifndef USE_ikdforest
            if (!use_new_map) lasermap_fov_segment();
            // fout_out << "Before seg- tree size: " << fov_rec_before[0] << " " << fov_rec_before[1] << " " << fov_rec_before[2] << endl;
            // fout_out << "FoV seg - size : " << fov_size[0] << " " << fov_size[1] << " " << fov_size[2] << endl;
            // fout_out << "After seg - tree size: " << fov_rec_after[0] << " " << fov_rec_after[1] << " " << fov_rec_after[2] << endl;
            // cout << "Max Queue Size is : " << ikdtree.max_queue_size << endl;
            // fout_out << "Point Cache Size: " << points_cache_size << endl;
#endif
            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
#ifdef USE_ikdforest
            if (!ikdforest.initialized){
                if(feats_down_size > 5){
                    ikdforest.Build(feats_down_body->points, true, lidar_end_time);
                }
                continue;                
            }
            int featsFromMapNum = ikdforest.total_size;
#else
            if (!use_new_map) {
                if (ikdtree.Root_Node == nullptr) {
                    if (feats_down_size > 5) {
                        ikdtree.set_downsample_param(filter_size_map_min);
                        feats_down_world->resize(feats_down_size);
                        for (int i = 0; i < feats_down_size; i++) {
                            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                        }
                        ikdtree.Build(feats_down_world->points);
                    }
                    string file_name = out_file;
                    stringstream ss;
                    ss << setw(6) << setfill('0') << cur_frame;
                    file_name += ss.str();
                    file_name.append(".label");

                    string file_name_origin = out_file_origin;
                    stringstream sss;
                    sss << setw(6) << setfill('0') << cur_frame;
                    file_name_origin += sss.str();
                    file_name_origin.append(".label");
                    if (out_file.length() > 0 || out_file_origin.length() > 0)
                        DynObjFilt->set_path(file_name, file_name_origin);
                    // cout<<file_name<<endl;
                    // DynObjFilt->publish_img(image_pub, image_buffer, lidar_end_time);
#ifdef USE_IKFOM
                    m_file << std::fixed << std::setprecision(6) << "3odomAftMapped.header.stamp"
                         << std::setprecision(7) << " " << state.pos_end[0]
                         << " " << state.pos_end[1] << " "
                         << state.pos_end[2] << " " << state.rot_end(0, 0) << " "
                         << state.rot_end(1, 1) << " " << state.rot_end(2, 2) << " "
                         << std::endl;
                    DynObjFilt->filter(feats_undistort, state_point.rot.toRotationMatrix(), state_point.pos, lidar_end_time);
#else
//                    m_file << std::fixed << std::setprecision(6) << "4odomAftMapped.header.stamp"
//                           << std::setprecision(7) << " " << state.pos_end[0]
//                           << " " << state.pos_end[1] << " "
//                           << state.pos_end[2] << " " << state.rot_end(0, 0) << " "
//                           << state.rot_end(1, 1) << " " << state.rot_end(2, 2) << " "
//                           << std::endl;
                    Eigen::Quaterniond current_q(state.rot_end);
                    m_file << lidar_end_time << " " << state.pos_end[0]
                           << " " << state.pos_end[1] << " "
                           << state.pos_end[2] << " " << current_q.x() << " "
                           << current_q.y() << " " << current_q.z() << " "
                           << current_q.w() << std::endl;
                    DynObjFilt->filter(feats_undistort, state.rot_end, state.pos_end,
                                       lidar_end_time); //state.offset_R_L_I , state.offset_T_L_I,
#endif
                    DynObjFilt->publish_hist(pubLaserCloudhist_depth, lidar_end_time);
                    // DynObjFilt->ReadFromLabel(pubLaserCloudEffect, feats_undistort, state.rot_end, state.pos_end, label_folder, cur_frame, lidar_end_time);
                    DynObjFilt->publish_dyn(pubLaserCloudEffect, pubLaserCloudEffect_depth, lidar_end_time);
                    publish_frame_body(pubLaserCloudFullRes_body);
                    cur_frame += 1;


                    continue;
                }
                int featsFromMapNum = ikdtree.validnum();
                kdtree_size_st = ikdtree.size();
            }
#endif

            // cout<<"[ mapping ]: Raw num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            // VD(DIM_STATE) P_diag = state.cov.diagonal();
            // cout<<"P_pre: "<<P_diag.transpose()<<endl;

#ifdef USE_IKFOM
            //state_ikfom fout_state = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;
#else
            euler_cur = RotMtoEuler(state.rot_end);
            fout_pre << setprecision(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() * 57.3 << " " << state.pos_end.transpose() << " "
                     << state.vel_end.transpose() \
 << " " << state.bias_g.transpose() << " " << state.bias_a.transpose() << " " << state.gravity.transpose() << endl;
#endif

#ifdef USE_ikdforest
            // if (1){
            //     int tmp_index;
            //     vector<PointCubeIndexType> tmp_points;
            //     for (int i=0;i< ikdforest.root_index.size();i++){
            //         tmp_index = ikdforest.root_index[i];
            //         ikdforest.flatten(ikdforest.roots[tmp_index],tmp_points);
            //     }
            //     featsFromMap->clear();
            //     for (int i=0;i<tmp_points.size();i++){
            //         featsFromMap->points.push_back(tmp_points[i].point);
            //     }
            // }
#else
            if (0) {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }
#endif

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();


            /*** iterated state estimation ***/
            std::vector<M3D> body_var;
            std::vector<M3D> crossmat_list;
            body_var.reserve(feats_down_size);
            crossmat_list.reserve(feats_down_size);
            if (use_new_map) {

                for (size_t i = 0; i < feats_down_body->size(); i++) {
                    V3D point_this(feats_down_body->points[i].x,
                                   feats_down_body->points[i].y,
                                   feats_down_body->points[i].z);
                    // M3D var;
                    // if (point_this.norm() > 5) {
                    // calcBodyVar(point_this, 0.05, 0.05, var);
                    // } else {
                    // calcBodyVar(point_this, 0.05, 0.05, var);
                    // }
                    // //
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);
                    crossmat_list.push_back(point_crossmat);
                    // body_var.push_back(var);
                }
            }
#ifdef MP_EN
            // cout<<"Using multi-processor, used core number: "<<MP_PROC_NUM<<endl;
#endif
            double t_update_start = omp_get_wtime();
            std::vector<ptpl> ptpl_list_raw, ptpl_list;
            vector<pointWithVar> pv_list;
            std::vector<M3D> var_list;


            ptpl_list_raw.resize(feats_down_size);
            pv_list.resize(feats_down_size);
            var_list.resize(feats_down_size);


            for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) {
                match_start = omp_get_wtime();
                laserCloudOri->clear();
                corr_normvect->clear();
                total_residual = 0.0;


                std::vector<double> r_list;
                ptpl_list.clear();
                ptpl_list.reserve(feats_down_size);

                /** new residual computation **/
                if (use_new_map) {
                    startTime = clock();

                    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
                    for (size_t i = 0; i < feats_down_body->size(); i++) {

                        pointWithVar pv;
                        pv.point << feats_down_body->points[i].x,
                                feats_down_body->points[i].y, feats_down_body->points[i].z;

                        // M3D var = body_var[i];
                        M3D var = Eigen::Matrix3d::Zero();
                        M3D point_crossmat = crossmat_list[i];
                        M3D rot_var = state.cov.block<3, 3>(0, 0);
                        M3D t_var = state.cov.block<3, 3>(3, 3);
                        var =
                                state.rot_end * var * state.rot_end.transpose() +
                                (-point_crossmat) * rot_var * (-point_crossmat.transpose()) +
                                t_var;
                        pv.var = var;
                        pv_list[i] = (pv);
                        var_list[i] = (var);
                        BuildResidualList(feat_map, max_voxel_size, sigma_num, pv_list[i],
                                          state.rot_end, state.pos_end, ptpl_list_raw[i]);
                    }

                    if (last_match_num == 0) last_match_num = ptpl_list.size();

                    // BuildResidualList(feat_map, max_voxel_size, sigma_num, pv_list,
                    //                 state.rot_end, state.pos_end, ptpl_list);
                    double mean_dis = 0;
                    effct_feat_num = 0;
                    for (auto ptpl: ptpl_list_raw) {
                        if (ptpl.is_valid) {
                            V3D p_w = state.rot_end * ptpl.point + state.pos_end;
                            double dis_to_plane =
                                    fabs(ptpl.normal(0) * p_w(0) + ptpl.normal(1) * p_w(1) +
                                         ptpl.normal(2) * p_w(2) + ptpl.d);
                            mean_dis += dis_to_plane;
                            ptpl_list.push_back(ptpl);
                            PointType pi_body;
                            PointType pi_world;
                            PointType pl;
                            pi_body.x = ptpl.point(0);
                            pi_body.y = ptpl.point(1);
                            pi_body.z = ptpl.point(2);
                            pointBodyToWorld(&pi_body, &pi_world);
                            pl.x = ptpl.normal(0);
                            pl.y = ptpl.normal(1);
                            pl.z = ptpl.normal(2);
                            // if (ptpl_list[i].point.norm() < 10 &&
                            //     fabs(ptpl_list[i].normal[2] > 0.7)) {
                            //   continue;
                            // }
                            effct_feat_num++;
                            float dis = (pi_world.x * pl.x + pi_world.y * pl.y + pi_world.z * pl.z + ptpl.d);
                            pl.intensity = dis;
                            laserCloudOri->push_back(pi_body);
                            corr_normvect->push_back(pl);
                            total_residual += fabs(dis);
                            r_list.push_back(ptpl.eigen_value);
                        }

                    }
                    mean_dis = mean_dis / ptpl_list.size();

                    // publish_effect_world(pubLaserCloudEffect, pubVarPoints,
                    // ptpl_list); if (is_pub_plane_map)
                    //   pubPlaneMap(feat_map, plane_pub, state.pos_end);
                    res_mean_last = total_residual / effct_feat_num;
                    endTime = clock();
                    match_time += (double) (endTime - startTime) / CLOCKS_PER_SEC;
                    // cout << "[ Matching ]: Time:"
                    //      << (double)(endTime - startTime) / CLOCKS_PER_SEC * 1000
                    //      << " ms  Effective feature num: " << effct_feat_num
                    //      << " Last num:" << last_match_num << "  res_mean_last "
                    //      << res_mean_last << endl;
                    last_match_num = ptpl_list.size();
                } else {
                    /** closest surface search and residual computation **/
                    // #ifdef MP_EN
                    //     omp_set_num_threads(MP_PROC_NUM);
                    //     #pragma omp parallel for
                    // #endif
                    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
                    for (int i = 0; i < feats_down_size; i++) {
                        PointType &point_body = feats_down_body->points[i];
                        PointType &point_world = feats_down_world->points[i];
                        V3D p_body(point_body.x, point_body.y, point_body.z);
                        /* transform to world frame */
                        pointBodyToWorld(&point_body, &point_world);
                        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                        // point_world_rec[i] = point_world;
                        auto &points_near = Nearest_Points[i];
                        uint8_t search_flag = 0;
                        double search_start = omp_get_wtime();
                        if (nearest_search_en) {
                            /** Find the closest surfaces in the map **/
#ifdef USE_ikdforest
                            search_flag = ikdforest.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, first_lidar_time, 5);
#else
                            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, 5);
#endif

                            // if(i==0)     cout<<points_near[0].x<<endl;

                            if (points_near.size() < NUM_MATCH_POINTS) {
                                point_selected_surf[i] = false;
                            } else {
                                point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
                            }

#ifdef USE_ikdforest
                            point_selected_surf[i] = point_selected_surf[i] && (search_flag == 0);
#endif
                            search_time_rec[i] = omp_get_wtime() - search_start;
                        }

                        res_last[i] = -1000.0f;

                        if (!point_selected_surf[i] || points_near.size() < NUM_MATCH_POINTS) {
                            point_selected_surf[i] = false;
                            continue;
                        }

                        point_selected_surf[i] = false;
                        VF(4) pabcd;
                        pabcd.setZero();
                        if (esti_plane(pabcd, points_near, 0.1f)) //(planeValid)
                        {
                            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z +
                                        pabcd(3);
                            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                            if (s > match_s) {
                                point_selected_surf[i] = true;
                                normvec->points[i].x = pabcd(0);
                                normvec->points[i].y = pabcd(1);
                                normvec->points[i].z = pabcd(2);
                                normvec->points[i].intensity = pd2;
                                res_last[i] = abs(pd2);
                            }
                        }
                    }
                    effct_feat_num = 0;
                    // omp_set_num_threads(MP_PROC_NUM);
                    // #pragma omp parallel for
                    for (int i = 0; i < feats_down_size; i++) {
                        if (point_selected_surf[i]) {
                            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                            corr_normvect->points[effct_feat_num] = normvec->points[i];
                            effct_feat_num++;
                        }
                    }

                    res_mean_last = total_residual / effct_feat_num;

                    /*** Debug residual computation ***/
                    // if(Measures.lidar_beg_time-first_lidar_time < 60.0)
                    // {
                    //     fout_dbg<<setprecision(20)<<"time "<<Measures.lidar_beg_time-first_lidar_time<<" "<<total_residual<<" "<<effct_feat_num<<endl;
                    //     for (int i = 0; i < feats_down_size; i++)
                    //     {
                    //         fout_dbg<<setprecision(20)<<i<<" "<<res_last[i]<<" "<<point_selected_surf[i]<<" "<<normvec->points[i].intensity<<endl;
                    //     }
                    // }
                    // fout_out<<" res "<<Measures.lidar_beg_time-first_lidar_time<<" "<<total_residual<<" "<<effct_feat_num<<endl;
                    // cout << "[ mapping ]: Effective feature num: "<<effct_feat_num<<" res_mean_last "<<res_mean_last<<endl;
                    match_time = omp_get_wtime() - match_start;
                }

                solve_start = omp_get_wtime();

                /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                MatrixXd Hsub(effct_feat_num, 6);
                MatrixXd Hsub_T_R_inv(6, effct_feat_num);
                VectorXd R_inv(effct_feat_num);
                VectorXd meas_vec(effct_feat_num);

                for (int i = 0; i < effct_feat_num; i++) {
                    const PointType &laser_p = laserCloudOri->points[i];
                    V3D point_this(laser_p.x, laser_p.y, laser_p.z);
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    M3D var;
                    if (point_this.norm() > 5) {
                        calcBodyVar(point_this, 0.05, 0.05, var);
                    } else {
                        calcBodyVar(point_this, 0.05, 0.05, var);
                    }
                    var = state.rot_end * var * state.rot_end.transpose();
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);

                    /*** get the normal vector of closest surface/corner ***/
                    const PointType &norm_p = corr_normvect->points[i];
                    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

                    if (use_new_map) {
                        V3D point_world = state.rot_end * point_this + state.pos_end;
                        // /*** get the normal vector of closest surface/corner ***/

                        // bug exist, to be fixed
                        Eigen::Matrix<double, 1, 6> J_nq;
                        J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
                        J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
                        double sigma_l = J_nq * ptpl_list[i].plane_var * J_nq.transpose();
                        R_inv(i) = 1000;
                        // 1.0 / (sigma_l + norm_vec.transpose() * var * norm_vec);
                        // R_inv(i) = 1.0 / (sigma_l);
                        // R_inv(i) = 1.0 / (sigma_l);
                        // cout << "R_inv:" << R_inv(i) << endl;
                        // if (R_inv(i) > 40000) {
                        //   R_inv(i) = 40000;
                        //   // R_inv(i) = 1;
                        // }
                        // R_inv(i) = 1000;
                    } else {
                        R_inv(i) = 1000;
                    }
                    laserCloudOri->points[i].intensity = sqrt(R_inv(i));
                    //            R_inv(i) = 1.0;

                    /*** calculate the Measuremnt Jacobian matrix H ***/
                    V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
                    Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
                    Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i),
                            A[2] * R_inv(i), norm_p.x * R_inv(i), norm_p.y * R_inv(i),
                            norm_p.z * R_inv(i);
                    /*** Measuremnt: distance to the closest surface/corner ***/
                    meas_vec(i) = -norm_p.intensity;
                }
                solve_const_H_time += omp_get_wtime() - solve_start;

                // MatrixXd K(DIM_STATE, effct_feat_num);

                EKF_stop_flg = false;
                flg_EKF_converged = false;

                /*** Iterative Kalman Filter Update ***/
                if (!flg_EKF_inited) {
                    cout << "||||||||||Initiallizing LiDar||||||||||" << endl;
                    /*** only run in initialization period ***/
                    MatrixXd H_init(MD(9, DIM_STATE)::Zero());
                    MatrixXd z_init(VD(9)::Zero());
                    H_init.block<3, 3>(0, 0) = M3D::Identity();
                    H_init.block<3, 3>(3, 3) = M3D::Identity();
                    H_init.block<3, 3>(6, 15) = M3D::Identity();
                    z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
                    z_init.block<3, 1>(0, 0) = -state.pos_end;

                    auto H_init_T = H_init.transpose();
                    auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T + \
                                    0.0001 * MD(9, 9)::Identity()).inverse();
                    solution = K_init * z_init;

                    // solution.block<9,1>(0,0).setZero();
                    // state += solution;
                    // state.cov = (MatrixXd::Identity(DIM_STATE, DIM_STATE) - K_init * H_init) * state.cov;

                    state.resetpose();
                    EKF_stop_flg = true;
                } else {
                    // auto &&Hsub_T = Hsub.transpose();
                    // H_T_H.block<6,6>(0,0) = Hsub_T_R_inv * Hsub;
                    // MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + state.cov.inverse()).inverse();
                    // K = K_1.block<DIM_STATE,6>(0,0) * Hsub_T_R_inv;
                    // auto vec = state_propagat - state;
                    // solution = K * meas_vec + vec - K * Hsub * vec.block<6,1>(0,0);

                    // auto &&Hsub_T = Hsub.transpose();
                    auto &&HTz = Hsub_T_R_inv * meas_vec;
                    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
                    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
                    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + state.cov.inverse()).inverse();
                    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
                    auto vec = state_propagat - state;
                    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec -
                               G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);

                    // fout_out<<Measures.lidar_beg_time-first_lidar_time<<" crc_nom "<<crc_nom<<endl;
                    // fout_out<<Measures.lidar_beg_time-first_lidar_time<<" H_T_H "<<H_T_H.block<6,6>(0,0)<<endl;
                    // fout_out<<Measures.lidar_beg_time-first_lidar_time<<" solution "<<solution.transpose()<<endl;

                    int minRow, minCol;
                    if (0) // if(V.minCoeff(&minRow, &minCol) < 1.0f)
                    {
                        VD(6) V = H_T_H.block<6, 6>(0, 0).eigenvalues().real();
                        cout << "!!!!!! Degeneration Happend, eigen values: " << V.transpose() << endl;
                        EKF_stop_flg = true;
                        solution.block<6, 1>(9, 0).setZero();
                    }

                    state += solution;

                    rot_add = solution.block<3, 1>(0, 0);
                    t_add = solution.block<3, 1>(3, 0);

                    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) {
                        flg_EKF_converged = true;
                    }

                    deltaR = rot_add.norm() * 57.3;
                    deltaT = t_add.norm() * 100;
                }

                euler_cur = RotMtoEuler(state.rot_end);

#ifdef DEBUG_PRINT
                cout<<"update: R"<<euler_cur.transpose()*57.3<<" p "<<state.pos_end.transpose()<<" v "<<state.vel_end.transpose()<<" bg"<<state.bias_g.transpose()<<" ba"<<state.bias_a.transpose()<<endl;
                cout<<"dR & dT: "<<deltaR<<" "<<deltaT<<" res norm:"<<res_mean_last<<endl;
#endif

                /*** Rematch Judgement ***/
                nearest_search_en = false;
                if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2)))) {
                    nearest_search_en = true;
                    rematch_num++;
                }

                /*** Convergence Judgements and Covariance Update ***/
                if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))) {
                    if (flg_EKF_inited) {
                        /*** Covariance Update ***/
                        state.cov = (I_STATE - G) * state.cov;
                        total_distance += (state.pos_end - position_last).norm();
                        position_last = state.pos_end;
                        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                (euler_cur(0), euler_cur(1), euler_cur(2));

                        // VD(DIM_STATE) K_sum  = K.rowwise().sum();
                        // VD(DIM_STATE) P_diag = state.cov.diagonal();
                        // cout<<"K: "<<K_sum.transpose()<<endl;
                        // cout<<"P_out: "<<P_diag.transpose()<<endl;
                        // cout<< setprecision(6)<< "position: "<<state.pos_end.transpose()<<" total distance: "<<total_distance<<endl;
                    }
                    EKF_stop_flg = true;
                }
                solve_time += omp_get_wtime() - solve_start;

                if (EKF_stop_flg) break;
            }
            // cout<<"[ mapping ]: iteration count: "<<iterCount+1<<endl;

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            if (!use_new_map) {
                for (int i = 0; i < feats_down_size; i++) kdtree_search_time += search_time_rec[i];
                map_incremental();
            } else {
                startTime = clock();
                Eigen::Quaterniond q(state.rot_end);
                pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
                        new pcl::PointCloud<pcl::PointXYZI>);
                transformLidar(state.rot_end, state.pos_end, feats_down_body,
                               world_lidar);
                std::vector<pointWithVar> pv_list;
                for (size_t i = 0; i < world_lidar->size(); i++) {
                    pointWithVar pv;
                    pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
                            world_lidar->points[i].z;
                    M3D point_crossmat = crossmat_list[i];
                    M3D var = var_list[i];
                    var = state.rot_end * var * state.rot_end.transpose() +
                          (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
                          (-point_crossmat).transpose() +
                          state.cov.block<3, 3>(3, 3);
                    pv.var = var;
                    pv_list.push_back(pv);
                }
                std::sort(pv_list.begin(), pv_list.end(), var_contrast);
                updateUnorderMap(pv_list, max_voxel_size, layer_size, min_eigen_value,
                                 feat_map);
                scanIdx++;
                endTime = clock();
                std::cout << "[Update map]: time:"
                          << (double) (endTime - startTime) / CLOCKS_PER_SEC * 1000
                          << "ms" << std::endl;
                kdtree_incremental_time = omp_get_wtime() - t_update_end;
            }

            t5 = omp_get_wtime();
#ifndef USE_ikdforest
            kdtree_size_end = ikdtree.size();
#endif
            /******* Publish points *******/
            if (scan_pub_en || pcd_save_en) publish_cluster_world(cluster_gt_pub, fout_pos);
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFullRes);
            // if (scan_pub_en && scan_body_pub_en) 
            publish_frame_body(pubLaserCloudFullRes_body);
            // DynObjFilt->filter(feats_undistort, state_point, lidar_end_time);
            std::cout << "lidar_end_time: " << lidar_end_time << std::endl;

            string file_name = out_file;
            stringstream ss;
            ss << setw(6) << setfill('0') << cur_frame;
            file_name += ss.str();
            file_name.append(".label");

            string file_name_origin = out_file_origin;
            stringstream sss;
            sss << setw(6) << setfill('0') << cur_frame;
            file_name_origin += sss.str();
            file_name_origin.append(".label");
            if (out_file.length() > 0 || out_file_origin.length() > 0)
                DynObjFilt->set_path(file_name, file_name_origin);
            // cout<<file_name<<endl;
            // DynObjFilt->publish_img(image_pub, image_buffer, lidar_end_time);
#ifdef USE_IKFOM
            m_file << std::fixed << std::setprecision(6) << "6odomAftMapped.header.stamp"
                         << std::setprecision(7) << " " << state.pos_end[0]
                         << " " << state.pos_end[1] << " "
                         << state.pos_end[2] << " " << state.rot_end(0, 0) << " "
                         << state.rot_end(1, 1) << " " << state.rot_end(2, 2) << " "
                         << std::endl;
            DynObjFilt->filter(feats_undistort, state_point.rot.toRotationMatrix(), state_point.pos, lidar_end_time);
#else
//            m_file << std::fixed << std::setprecision(6) << "7odomAftMapped.header.stamp"
//                   << std::setprecision(7) << " " << state.pos_end[0]
//                   << " " << state.pos_end[1] << " "
//                   << state.pos_end[2] << " " << state.rot_end(0, 0) << " "
//                   << state.rot_end(1, 1) << " " << state.rot_end(2, 2) << " "
//                   << std::endl;
            Eigen::Quaterniond current_q(state.rot_end);
            m_file << lidar_end_time << " " << state.pos_end[0]
                   << " " << state.pos_end[1] << " "
                   << state.pos_end[2] << " " << current_q.x() << " "
                   << current_q.y() << " " << current_q.z() << " "
                   << current_q.w() << std::endl;
            DynObjFilt->filter(feats_undistort, state.rot_end, state.pos_end,
                               lidar_end_time); //state.offset_R_L_I , state.offset_T_L_I,
#endif
            DynObjFilt->publish_hist(pubLaserCloudhist_depth, lidar_end_time);
            // DynObjFilt->ReadFromLabel(pubLaserCloudEffect, feats_undistort, state.rot_end, state.pos_end, label_folder, cur_frame, lidar_end_time);
            DynObjFilt->publish_dyn(pubLaserCloudEffect, pubLaserCloudEffect_depth, lidar_end_time);
            cur_frame += 1;
            // DynObjFilt->time_file << t5-t0 << endl;

            last_odom = state.pos_end;
            last_rot = state.rot_end;
            if (is_pub_plane_map) pubPlaneMap(feat_map, plane_pub, state.pos_end);
            endTime = clock();
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);
            if (path_en) publish_path(pubPath);
#ifdef DEPLOY
            publish_mavros(mavros_pose_publisher);
#endif

            /*** Debug variables Logging ***/
            frame_num++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
            aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
            aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
#ifdef USE_IKFOM
            aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
            aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
#else
            aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
            aver_time_const_H_time =
                    aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
            //cout << "construct H:" << aver_time_const_H_time << std::endl;
#endif
            // aver_time_consu = aver_time_consu * 0.9 + (t5 - t0) * 0.1;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = t5 - t0;
            s_plot2[time_log_counter] = feats_undistort->points.size();
            s_plot3[time_log_counter] = kdtree_incremental_time;
            s_plot4[time_log_counter] = kdtree_search_time;
            s_plot5[time_log_counter] = kdtree_delete_counter;
            s_plot6[time_log_counter] = kdtree_delete_time;
            s_plot7[time_log_counter] = kdtree_size_st;
            s_plot8[time_log_counter] = kdtree_size_end;
            s_plot9[time_log_counter] = aver_time_consu;
            s_plot10[time_log_counter] = add_point_size;
            time_log_counter++;
            printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",
                   t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                   aver_time_const_H_time);
            printf("---------------------------------------");
#ifdef USE_IKFOM
            ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
#else
            fout_out << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
                     << state.pos_end.transpose() << " " << state.vel_end.transpose() \
 << " " << state.bias_g.transpose() << " " << state.bias_a.transpose() << " " << state.gravity.transpose() << " "
                     << feats_undistort->points.size() << endl;
#endif
            dump_lio_state_to_log(fp);
        }
        status = ros::ok();
        rate.sleep();
    }
    //--------------------------save map---------------
    // string surf_filename(map_file_path + "/surf.pcd");
    // string corner_filename(map_file_path + "/corner.pcd");
    // string all_points_filename(map_file_path + "/all_points.pcd");

    // PointCloudXYZI surf_points, corner_points;
    // surf_points = *featsFromMap;
    // fout_out.close();
    // fout_pre.close();
    // if (surf_points.size() > 0 && corner_points.size() > 0) 
    // {
    // pcl::PCDWriter pcd_writer;
    // cout << "saving...";
    // pcd_writer.writeBinary(surf_filename, surf_points);
    // pcd_writer.writeBinary(corner_filename, corner_points);
    // }

    // #ifndef DEPLOY
    // vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
    // FILE *fp2;
    // string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
    // fp2 = fopen(log_dir.c_str(),"w");
    // fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
    // for (int i = 0;i<time_log_counter; i++){
    //     fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
    //     t.push_back(T1[i]);
    //     s_vec.push_back(s_plot9[i]);
    //     s_vec2.push_back(s_plot3[i] + s_plot6[i]);
    //     s_vec3.push_back(s_plot4[i]);
    //     s_vec5.push_back(s_plot[i]);
    // }
    // fclose(fp2);
    // if (!t.empty())
    // {
    //     plt::named_plot("incremental time",t,s_vec2);
    //     plt::named_plot("search_time",t,s_vec3);
    //     plt::named_plot("total time",t,s_vec5);
    //     plt::named_plot("average time",t,s_vec);
    //     plt::legend();
    //     plt::show();
    //     plt::pause(0.5);
    //     plt::close();
    // }
    // cout << "no points saved" << endl;
    // #endif

    return 0;
}
