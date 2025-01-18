#include <iostream>
#include <vector>
#include <random>
#include "DynObjFilter.h"
// #include <algorithm>
// #include <chrono>
// #include <execution>

#define PI_MATH  (3.14159f)

struct Pos
{
  int x;
  int y;

  Pos() { }
  Pos(int x, int y)
  {
    this->x = x;
    this->y = y;
  }

  bool operator==(const Pos& otherPos) const
  {
    if (this->x == otherPos.x && this->y == otherPos.y) return true;
    else return false;
  }

  struct HashFunction
  {
    size_t operator()(const Pos& pos) const
    {
      return   pos.x * HASH_P % MAX_N + pos.y;
    }
  };
};

void  DynObjFilter::SetParam(ros::NodeHandle& nh)
{
    nh.param<double>("dyn_obj/buffer_delay", buffer_delay, 0.1);
    nh.param<int>("dyn_obj/buffer_size",buffer_size, 300000);
    nh.param<int>("dyn_obj/points_num_perframe",points_num_perframe, 150000);
    nh.param<double>("dyn_obj/depth_map_dur", depth_map_dur, 0.2);
    nh.param<int>("dyn_obj/max_depth_map_num",max_depth_map_num, 5);
    nh.param<int>("dyn_obj/max_pixel_points",max_pixel_points, 50);
    nh.param<double>("dyn_obj/frame_dur", frame_dur, 0.1);
    nh.param<int>("dyn_obj/dataset", dataset, 0);
    nh.param<float>("dyn_obj/self_x_f", self_x_f, 0.15f);
    nh.param<float>("dyn_obj/self_x_b", self_x_b, 0.15f);
    nh.param<float>("dyn_obj/self_y_l", self_y_l, 0.15f);
    nh.param<float>("dyn_obj/self_y_r", self_y_r, 0.5f);
    nh.param<float>("dyn_obj/blind_dis", blind_dis, 0.15f);
    nh.param<float>("dyn_obj/fov_up", fov_up, 0.15f);
    nh.param<float>("dyn_obj/fov_down", fov_down, 0.15f);
    nh.param<float>("dyn_obj/fov_cut", fov_cut, 0.15f);
    nh.param<float>("dyn_obj/fov_left", fov_left, 180.0f);
    nh.param<float>("dyn_obj/fov_right", fov_right, -180.0f);
    nh.param<int>("dyn_obj/checkneighbor_range", checkneighbor_range, 1);
    nh.param<bool>("dyn_obj/stop_object_detect", stop_object_detect, false);
    

    
    nh.param<float>("dyn_obj/depth_thr1", depth_thr1, 0.15f);
    nh.param<float>("dyn_obj/enter_min_thr1", enter_min_thr1, 0.15f);
    nh.param<float>("dyn_obj/enter_max_thr1", enter_max_thr1, 0.15f);
    nh.param<float>("dyn_obj/map_cons_depth_thr1", map_cons_depth_thr1, 0.5f);
    nh.param<float>("dyn_obj/map_cons_hor_thr1", map_cons_hor_thr1, 0.01f);
    nh.param<float>("dyn_obj/map_cons_ver_thr1", map_cons_ver_thr1, 0.01f);
    nh.param<float>("dyn_obj/map_cons_hor_dis1", map_cons_hor_dis1, 0.2f);
    nh.param<float>("dyn_obj/map_cons_ver_dis1", map_cons_ver_dis1, 0.1f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr1", depth_cons_depth_thr1, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr1", depth_cons_depth_max_thr1, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr1", depth_cons_hor_thr1, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr1", depth_cons_ver_thr1, 0.01f);
    nh.param<float>("dyn_obj/enlarge_z_thr1", enlarge_z_thr1, 0.05f);
    nh.param<float>("dyn_obj/enlarge_angle", enlarge_angle, 2.0f);
    nh.param<float>("dyn_obj/enlarge_depth", enlarge_depth, 3.0f);
    nh.param<int>("dyn_obj/occluded_map_thr1", occluded_map_thr1, 3);
    nh.param<bool>("dyn_obj/case1_interp_en", case1_interp_en, false);


    nh.param<float>("dyn_obj/v_min_thr2", v_min_thr2, 0.5f);
    nh.param<float>("dyn_obj/acc_thr2", acc_thr2, 1.0f);
    nh.param<float>("dyn_obj/map_cons_depth_thr2", map_cons_depth_thr2, 0.15f);
    nh.param<float>("dyn_obj/map_cons_hor_thr2", map_cons_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/map_cons_ver_thr2", map_cons_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/occ_depth_thr2", occ_depth_thr2, 0.15f);
    nh.param<float>("dyn_obj/occ_hor_thr2", occ_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/occ_ver_thr2", occ_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr2", depth_cons_depth_thr2, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr2", depth_cons_depth_max_thr2, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr2", depth_cons_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr2", depth_cons_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/k_depth2", k_depth2, 0.005f);
    nh.param<int>("dyn_obj/occluded_times_thr2", occluded_times_thr2, 3);
    nh.param<bool>("dyn_obj/case2_interp_en", case2_interp_en, false);

    nh.param<float>("dyn_obj/v_min_thr3", v_min_thr3, 0.5f);
    nh.param<float>("dyn_obj/acc_thr3", acc_thr3, 1.0f);
    nh.param<float>("dyn_obj/map_cons_depth_thr3", map_cons_depth_thr3, 0.15f);
    nh.param<float>("dyn_obj/map_cons_hor_thr3", map_cons_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/map_cons_ver_thr3", map_cons_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/occ_depth_thr3", occ_depth_thr3, 0.15f);
    nh.param<float>("dyn_obj/occ_hor_thr3", occ_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/occ_ver_thr3", occ_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr3", depth_cons_depth_thr3, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr3", depth_cons_depth_max_thr3, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr3", depth_cons_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr3", depth_cons_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/k_depth3", k_depth3, 0.005f);
    nh.param<int>("dyn_obj/occluded_times_thr3", occluding_times_thr3, 3);
    nh.param<bool>("dyn_obj/case3_interp_en", case3_interp_en, false);


    nh.param<float>("dyn_obj/interp_hor_thr", interp_hor_thr, 0.01f);
    nh.param<float>("dyn_obj/interp_ver_thr", interp_ver_thr, 0.01f);
    nh.param<float>("dyn_obj/interp_thr1", interp_thr1, 1.0f);
    nh.param<float>("dyn_obj/interp_static_max", interp_static_max, 10.0f);
    nh.param<float>("dyn_obj/interp_start_depth1", interp_start_depth1, 20.0f);
    nh.param<float>("dyn_obj/interp_kp1", interp_kp1, 0.1f);
    nh.param<float>("dyn_obj/interp_kd1", interp_kd1, 1.0f);
    nh.param<float>("dyn_obj/interp_thr2", interp_thr2, 0.15f);
    nh.param<float>("dyn_obj/interp_thr3", interp_thr3, 0.15f);
    nh.param<float>("dyn_obj/interp_bg", interp_bg, 0.5f);

    nh.param<float>("dyn_obj/debug_x", debug_x, 0.15f);
    nh.param<float>("dyn_obj/debug_y", debug_y, 0.15f);
    nh.param<float>("dyn_obj/debug_z", debug_z, 0.15f);


    
    nh.param<bool>("dyn_obj/dyn_filter_en", dyn_filter_en,true);
    nh.param<bool>("dyn_obj/debug_publish", debug_en,true);
    nh.param<int>("dyn_obj/laserCloudSteadObj_accu_limit", laserCloudSteadObj_accu_limit,5);
    nh.param<float>("dyn_obj/voxel_filter_size", voxel_filter_size,0.1f);
    nh.param<bool>("dyn_obj/cluster_coupled", cluster_coupled, false);
    nh.param<bool>("dyn_obj/cluster_future", cluster_future, false);
    nh.param<int>("dyn_obj/cluster_extend_pixel", Cluster.cluster_extend_pixel, 2);
    nh.param<int>("dyn_obj/cluster_min_pixel_number", Cluster.cluster_min_pixel_number, 4);
    nh.param<float>("dyn_obj/cluster_thrustable_thresold", Cluster.thrustable_thresold, 0.3f);
    nh.param<float>("dyn_obj/cluster_Voxel_revolusion", Cluster.Voxel_revolusion, 0.3f);
    nh.param<bool>("dyn_obj/cluster_debug_en", Cluster.debug_en, false);
    nh.param<int>("dyn_obj/dyn_windows_num", dyn_windows_num,3);
    nh.param<int>("dyn_obj/occlude_windows", occlude_windows,3);
    nh.param<float>("dyn_obj/ver_resolution_max", ver_resolution_max,0.0025f);
    nh.param<float>("dyn_obj/hor_resolution_max", hor_resolution_max,0.0025f);
    nh.param<float>("dyn_obj/dyn_windows_dur", dyn_windows_dur,0.5f);
    nh.param<float>("dyn_obj/buffer_dur", buffer_dur,0.1f);
    nh.param<float>("dyn_obj/angle_noise", angle_noise, 0.001);
    nh.param<float>("dyn_obj/angle_occlude", angle_occlude, 0.02);
    nh.param<float>("dyn_obj/depth_thr", depth_thr, 0.15f);
    nh.param<float>("dyn_obj/depth_thr_fpc_max", depth_thr_fpc_max, 2.0f);
    nh.param<float>("dyn_obj/depth_thr_fpc_min", depth_thr_fpc_min, 0.1f);
    nh.param<float>("dyn_obj/min_dis_roll1", min_dis_roll1, 0.2f);
    nh.param<float>("dyn_obj/min_dis_pitch1", min_dis_pitch1, 0.2f);
    nh.param<float>("dyn_obj/min_roll1", min_roll1, 0.02f);
    nh.param<float>("dyn_obj/min_pitch1", min_pitch1, 0.01f);
    nh.param<float>("dyn_obj/depth_diff", depth_diff, 0.1f);
    nh.param<float>("dyn_obj/depth_cut", depth_cut, 1.0f);
    nh.param<float>("dyn_obj/roll_noise", roll_noise, 0.02f);
    nh.param<float>("dyn_obj/pitch_noise", pitch_noise, 0.01f);
    nh.param<float>("dyn_obj/min_dis_roll2", min_dis_roll2, 0.2f);
    nh.param<float>("dyn_obj/min_dis_pitch2", min_dis_pitch2, 0.2f);
    nh.param<float>("dyn_obj/min_roll2", min_roll2, 0.02f);
    nh.param<float>("dyn_obj/min_pitch2", min_pitch2, 0.01f);
    nh.param<float>("dyn_obj/depth2", depth2, 1.0f);
    nh.param<float>("dyn_obj/v_min_case2", v_min_case2, 0.5f);
    nh.param<float>("dyn_obj/depth_thr_case2", depth_thr_case2, 0.15f);
    nh.param<float>("dyn_obj/depth_minus", depth_minus, 0.5f);
    nh.param<float>("dyn_obj/occu_minus_th", occu_minus_th, 2.0f);
    nh.param<int>("dyn_obj/point_index", point_index, 0);
    nh.param<int>("dyn_obj/mode", mode, 0);
    nh.param<bool>("dyn_obj/interp_en", interp_en, false);
    nh.param<string>("dyn_obj/time_log_file", time_log_file,"");
    nh.param<string>("dyn_obj/pbp_log_file", pbp_log_file,"");
    nh.param<string>("common/frame_id", frame_id, "camera_init");
    nh.param<string>("dyn_obj/time_file", time_file, "");
    nh.param<string>("dyn_obj/save_file", save_file, "");

    max_ind   = floor(3.1415926 * 2 / hor_resolution_max);
    if (pcl_his_list.size() == 0)
    {   
        PointCloudXYZI::Ptr first_frame(new PointCloudXYZI());
        first_frame->reserve(400000);
        pcl_his_list.push_back(first_frame);
        // project_R.push_back(Eye3d);
        // project_T.push_back(Zero3d);
        laserCloudSteadObj_hist = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudSteadObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudDynObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudDynObj_world = PointCloudXYZI::Ptr(new PointCloudXYZI());
        
        int xy_ind[3] = {-1, 1};
        for (int ind_hor = 0; ind_hor < 2*hor_num + 1; ind_hor ++)
        {
            for (int ind_ver = 0; ind_ver < 2*ver_num + 1; ind_ver ++)
            {
                pos_offset.push_back(((ind_hor)/2 + ind_hor%2)*xy_ind[ind_hor%2] * MAX_1D_HALF + ((ind_ver)/2 + ind_ver%2)*xy_ind[ind_ver%2]);
            }
            
        }
    }

    map_cons_hor_num1 = ceil(map_cons_hor_thr1/hor_resolution_max);
    map_cons_ver_num1 = ceil(map_cons_ver_thr1/ver_resolution_max);
    interp_hor_num = ceil(interp_hor_thr/hor_resolution_max);
    interp_ver_num = ceil(interp_ver_thr/ver_resolution_max);
    map_cons_hor_num2 = ceil(map_cons_hor_thr2/hor_resolution_max);
    map_cons_ver_num2 = ceil(map_cons_ver_thr2/ver_resolution_max);
    occ_hor_num2 = ceil(occ_hor_thr2/hor_resolution_max);
    occ_ver_num2 = ceil(occ_ver_thr2/ver_resolution_max);
    depth_cons_hor_num2 = ceil(depth_cons_hor_thr2/hor_resolution_max);
    depth_cons_ver_num2 = ceil(depth_cons_ver_thr2/ver_resolution_max);
    map_cons_hor_num3 = ceil(map_cons_hor_thr3/hor_resolution_max);
    map_cons_ver_num3 = ceil(map_cons_ver_thr3/ver_resolution_max);
    occ_hor_num3 = ceil(occ_hor_thr3/hor_resolution_max);
    occ_ver_num3 = ceil(occ_ver_thr3/ver_resolution_max);
    depth_cons_hor_num3 = ceil(depth_cons_hor_thr3/hor_resolution_max);
    depth_cons_ver_num3 = ceil(depth_cons_ver_thr3/ver_resolution_max);
    buffer.init(buffer_size);

    pixel_fov_up = floor((fov_up/180.0*PI_MATH + 0.5 * PI_MATH)/ver_resolution_max);
    pixel_fov_down = floor((fov_down/180.0*PI_MATH + 0.5 * PI_MATH)/ver_resolution_max);
    // pixel_fov_left = floor((-fov_hor/180.0*PI_MATH + PI_MATH)/hor_resolution_max);
    // pixel_fov_right = floor((fov_hor/180.0*PI_MATH + PI_MATH)/hor_resolution_max);
    pixel_fov_cut = floor((fov_cut/180.0*PI_MATH +  0.5 * PI_MATH)/ver_resolution_max);
    pixel_fov_left = floor((fov_left/180.0*PI_MATH +  PI_MATH)/hor_resolution_max);
    pixel_fov_right = floor((fov_right/180.0*PI_MATH +  PI_MATH)/hor_resolution_max);

    max_pointers_num = round((dyn_windows_num * dyn_windows_dur + buffer_delay)/frame_dur) + 1;
    std::cout << "max_pointer_num: " << max_pointers_num << " ---------- " << points_num_perframe << std::endl;
    point_soph_pointers.reserve(max_pointers_num);
    for (int i = 0; i < max_pointers_num; i++)
    {
        point_soph* p = new point_soph[points_num_perframe];
        point_soph_pointers.push_back(p);
    }
    // time_file.open(time_log_file,ios::out);
    pbp_file.open(pbp_log_file,ios::out);

    // string time_file = "/home/huajie/event_detection/time/points.txt"; //rec computation time
    if(time_file != "")
    {
        time_out.open(time_file, ios::out); //rec computation time
    }
        
    // points_out.open("/home/huajie/event_detection/demo/cover/2022-12-07-15-36-31.txt", ios::out);
}

void  DynObjFilter::init(const int & windows_num, const float & windows_dur, const int & windows_occlude, const float & hor_resolution, const float & ver_resolution, const float & ang_noise, const float & ang_occlude, const bool & dbg_en, const int & pt_index)
{
    dyn_windows_num = windows_num;
    dyn_windows_dur = windows_dur; 
    occlude_windows = windows_occlude;
    hor_resolution_max  = hor_resolution;
    ver_resolution_max  = ver_resolution;
    angle_noise     = ang_noise;
    angle_occlude   = ang_occlude;
    debug_en        = dbg_en;
    point_index     = pt_index;

    std::cout << dyn_windows_num << " " << dyn_windows_dur << hor_resolution_max << ver_resolution_max << angle_noise << angle_occlude << point_index;
    max_ind   = floor(3.1415926 * 2 / hor_resolution);
    if (pcl_his_list.size() == 0)
    {   
        PointCloudXYZI::Ptr first_frame(new PointCloudXYZI());
        first_frame->reserve(400000);
        pcl_his_list.push_back(first_frame);
        // project_R.push_back(Eye3d);
        // project_T.push_back(Zero3d);
        laserCloudSteadObj_hist = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudSteadObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudDynObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudDynObj_world = PointCloudXYZI::Ptr(new PointCloudXYZI());
        
        int xy_ind[3] = {-1, 1};
        for (int ind_hor = 0; ind_hor < 2*hor_num + 1; ind_hor ++)
        {
            for (int ind_ver = 0; ind_ver < 2*ver_num + 1; ind_ver ++)
            {
                pos_offset.push_back(((ind_hor)/2 + ind_hor%2)*xy_ind[ind_hor%2] * MAX_1D_HALF + ((ind_ver)/2 + ind_ver%2)*xy_ind[ind_ver%2]);
            }
            
        }
    }

    map_cons_hor_num1 = ceil(map_cons_hor_thr1/hor_resolution_max);
    map_cons_ver_num1 = ceil(map_cons_ver_thr1/ver_resolution_max);
    interp_hor_num = ceil(interp_hor_thr/hor_resolution_max);
    interp_ver_num = ceil(interp_ver_thr/ver_resolution_max);
    map_cons_hor_num2 = ceil(map_cons_hor_thr2/hor_resolution_max);
    map_cons_ver_num2 = ceil(map_cons_ver_thr2/ver_resolution_max);
    occ_hor_num2 = ceil(occ_hor_thr2/hor_resolution_max);
    occ_ver_num2 = ceil(occ_ver_thr2/ver_resolution_max);
    depth_cons_hor_num2 = ceil(depth_cons_hor_thr2/hor_resolution_max);
    depth_cons_ver_num2 = ceil(depth_cons_ver_thr2/ver_resolution_max);
    map_cons_hor_num3 = ceil(map_cons_hor_thr3/hor_resolution_max);
    map_cons_ver_num3 = ceil(map_cons_ver_thr3/ver_resolution_max);
    occ_hor_num3 = ceil(occ_hor_thr3/hor_resolution_max);
    occ_ver_num3 = ceil(occ_ver_thr3/ver_resolution_max);
    depth_cons_hor_num3 = ceil(depth_cons_hor_thr3/hor_resolution_max);
    depth_cons_ver_num3 = ceil(depth_cons_ver_thr3/ver_resolution_max);
    buffer.init(buffer_size);

    pixel_fov_up = floor((fov_up/180.0*PI_MATH + 0.5 * PI_MATH)/ver_resolution_max);
    pixel_fov_down = floor((fov_down/180.0*PI_MATH + 0.5 * PI_MATH)/ver_resolution_max);
    // pixel_fov_left = floor((-fov_hor/180.0*PI_MATH + PI_MATH)/hor_resolution_max);
    // pixel_fov_right = floor((fov_hor/180.0*PI_MATH + PI_MATH)/hor_resolution_max);
    pixel_fov_cut = floor((fov_cut/180.0*PI_MATH +  0.5 * PI_MATH)/ver_resolution_max);
    pixel_fov_left = floor((fov_left/180.0*PI_MATH +  PI_MATH)/hor_resolution_max);
    pixel_fov_right = floor((fov_right/180.0*PI_MATH +  PI_MATH)/hor_resolution_max);

    max_pointers_num = round((dyn_windows_num * dyn_windows_dur + buffer_delay)/frame_dur) + 1;
    std::cout << "max_pointer_num: " << max_pointers_num << " ---------- " << points_num_perframe << std::endl;
    point_soph_pointers.reserve(max_pointers_num);
    for (int i = 0; i < max_pointers_num; i++)
    {
        point_soph* p = new point_soph[points_num_perframe];
        point_soph_pointers.push_back(p);
    }
    // time_file.open(time_log_file,ios::out);
    pbp_file.open(pbp_log_file,ios::out);

    // string time_file = "/home/huajie/event_detection/time/points.txt"; //rec computation time
    if(time_file != "")
    {
        time_out.open(time_file, ios::out); //rec computation time
    }
        
    // points_out.open("/home/huajie/event_detection/demo/cover/2022-12-07-15-36-31.txt", ios::out);
}

void  DynObjFilter::set_add(bool is_add)
{
    // add_map = is_add;
}

void  DynObjFilter::filter(bool if_rgb, PointCloudXYZI::Ptr &feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time)
{
    std::cout << "feats 0: " << feats_undistort->points[0].normal_x << " "
            << feats_undistort->points[0].normal_y << " "
            << feats_undistort->points[0].normal_z << std::endl;
    std::cout << "==============================" << std::endl;
    // return;
    // cout<<"-----in1------"<<endl;
    double t00 = omp_get_wtime();
    time_search = time_research = time_search_0 = time_build = time_total = time_other0 = 0.0;
    time_interp1 = time_interp2 =0;

    int num_build = 0, num_search_0 = 0, num_research = 0;


    if (feats_undistort == NULL) return;
    int size = feats_undistort->points.size();

//    /***--------------***/
//    cout << "!!!!!save---origin!!!!!" << endl;
//    string all_points_dir1("/home/weihairuo/bag/pcd_correct/origin_pointcloud/" + to_string(time_ind) + string(".pcd"));
//    cout << "-----------" << all_points_dir1 << "-----------" << feats_undistort->size() << endl;
//    pcl::PCDWriter pcd_writer;
//    pcd_writer.writeBinary(all_points_dir1, *feats_undistort);
//    cout << "----finish---origin-----" << endl;
//    /***--------------***/

    std::cout << "feats_undistort1 size: " << size << " time: "<< fixed << scan_end_time << std::endl;
    // cout<<"buffer: size: "<<buffer.size()<<endl;
    if (debug_en)
    {
        laserCloudSteadObj_hist.reset(new PointCloudXYZI());
        laserCloudSteadObj_hist->reserve(20*size);
    }
    dyn_tag_origin.clear();
    dyn_tag_origin.reserve(size);
    dyn_tag_origin.resize(size);
    dyn_tag_cluster.clear();
    dyn_tag_cluster.reserve(size);
    dyn_tag_cluster.resize(size);
    laserCloudDynObj.reset(new PointCloudXYZI());
    laserCloudDynObj->reserve(size);
    laserCloudDynObj_world.reset(new PointCloudXYZI());
    laserCloudDynObj_world->reserve(size);
    laserCloudSteadObj.reset(new PointCloudXYZI());
    laserCloudSteadObj->reserve(size);
    laserCloudDynObj_clus.reset(new PointCloudXYZI());
    laserCloudDynObj_clus->reserve(size);
    laserCloudSteadObj_clus.reset(new PointCloudXYZI());
    laserCloudSteadObj_clus->reserve(size);

    ofstream out;
    ofstream out_origin;
    bool is_rec = false;
    bool is_rec_origin = false;
    if(is_set_path)
    {
        out.open(out_file, ios::out  | ios::binary);
        out_origin.open(out_file_origin, ios::out | ios::binary);
        if (out.is_open())
        {
            is_rec = true;
        }
        if (out_origin.is_open())
        {
            is_rec_origin = true;
        }
    }


    int case2_num = 0;
    double t0 = omp_get_wtime();
    std::cout << "t0 - t00: " << t0- t00 << std::endl;
    double time_case1 = 0, time_case2 = 0, time_case3 = 0;

    pcl::PointCloud<PointType> raw_points_world;
    raw_points_world.reserve(size);
    raw_points_world.resize(size);

    std::vector<int> index(size);
    for (int i = 0; i < size; i++) {
        index[i] = i;
    }

    vector<point_soph*> points;
    points.reserve(size);
    points.resize(size);
    point_soph* p = point_soph_pointers[cur_point_soph_pointers];
    if(time_file != "") time_out << size << " "; //rec computation time
    std::for_each(std::execution::par, index.begin(), index.end(), [&](const int &i)
    // std::for_each(std::execution::seq, index.begin(), index.end(), [&](const int &i)
    {
        p[i].reset();
        should_print = (i == point_index);

        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
        int intensity = feats_undistort->points[i].intensity;
        p[i].r = feats_undistort->points[i].normal_x;
        p[i].g = feats_undistort->points[i].normal_y;
        p[i].b = feats_undistort->points[i].normal_z;
        ///debug
        if(i == 0){
            std::cout << "feats: " << feats_undistort->points[i].normal_x << " "
            << feats_undistort->points[i].normal_y << " "
            << feats_undistort->points[i].normal_z << std::endl;
            std::cout << "p: " << p[i].r << " " << p[i].g << " " << p[i].b << std::endl;
            std::cout << "======================" << std::endl;
        }

        V3D p_glob(rot_end * (p_body) + pos_end);
        p[i].glob = p_glob;

        p[i].dyn = STATIC;
        p[i].rot = rot_end.transpose();
        p[i].transl = pos_end;
        p[i].time = scan_end_time;
        p[i].local = p_body;

        if(dataset == 0 && fabs(intensity-666) < 10E-4)
        {
            p[i].is_distort = true;
        }
        // double t = omp_get_wtime();
        if (InvalidPointCheck(p_body, intensity))
        {
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
            dyn_tag_cluster[i] = -1;
            // laserCloudSteadObj->push_back(po);
        }
        else if(SelfPointCheck(p_body, p[i].dyn))
        {
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
            // po.normal_x = -1;
            // laserCloudSteadObj->push_back(po);
        }
        else if (Case1(p[i]))
        {
            p[i].dyn = CASE1;
            // po.normal_x = 1;
            // laserCloudDynObj->push_back(po);
            dyn_tag_origin[i] = 1;
        }
        else if (Case2(p[i]))
        {
            p[i].dyn = CASE2;
            // po.normal_y = p[i].occu_times;
            // laserCloudDynObj->push_back(po);
            dyn_tag_origin[i] = 1;
        }
        else if(Case3(p[i]))
        {
            p[i].dyn = CASE3;
            // po.normal_z = p[i].is_occu_times;
            // po.normal_x = p[i].vec(0);
            // po.normal_y = p[i].vec(1);
            // laserCloudDynObj->push_back(po);
            dyn_tag_origin[i] = 1;
        }
        else
        {
            // laserCloudSteadObj->push_back(po);
            dyn_tag_origin[i] = 0;
        }
        points[i] = &p[i];
        //  points[i] = p;
    });
    std::cout << "Case1/2/3 time: " << omp_get_wtime()-t0 << std::endl;
    if(time_file != "") time_out << omp_get_wtime()-t0 << " "; //rec computation time

    for(int i = 0; i < size; i++)
    {
        PointType po;
        po.x = points[i]->local[0];
        po.y = points[i]->local[1];
        po.z = points[i]->local[2];
        po.normal_x = points[i]->r;
        po.normal_y = points[i]->g;
        po.normal_z = points[i]->b;
//        std::cout << "=================================" << std::endl;
//        std::cout << "po rgb: " << po.normal_x << " " << po.normal_y << " " << po.normal_z << std::endl;
//        std::cout << "=================================" << std::endl;

        // V3D p_body(po.x, po.y, po.z);
        // double horizon_angle = atan2f(double(p_body(1)), double(p_body(0)));
        // double vertical_angle = atan2f(double(p_body(2)), sqrt(pow(double(p_body(0)), 2) + pow(double(p_body(1)), 2)));
        // if(horizon_angle > fov_right || horizon_angle < fov_left || vertical_angle < -fov_down || vertical_angle > -fov_up) continue;
        PointType po_w;
        po_w.x = points[i]->glob[0];
        po_w.y = points[i]->glob[1];
        po_w.z = points[i]->glob[2];
        po_w.normal_x = points[i]->r;
        po_w.normal_y = points[i]->g;
        po_w.normal_z = points[i]->b;
        raw_points_world[i] = po;
        switch(points[i]->dyn) // == CASE1 ||  || points[i]->dyn == CASE3)
        {
            case CASE1:
//                po.normal_x = 1;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case CASE2:
//                po.normal_y = points[i]->occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case CASE3:
//                po.normal_z = points[i]->is_occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            default:
                laserCloudSteadObj->push_back(po_w);
        }
    }

    // int map_nums = 0, pixel_num = 0, min_num = 10000, max_num = 0;
    // if(depth_map_list.size() > 0)
    // {
    //     for(int i = 0; i < depth_map_list.back()->depth_map.size(); i++)
    //     {
    //         map_nums += depth_map_list.back()->depth_map[i].size();
    //         if(depth_map_list.back()->depth_map[i].size() > 0 ) pixel_num ++;
    //         if(depth_map_list.back()->depth_map[i].size() > 0 && depth_map_list.back()->depth_map[i].size() < min_num ) min_num = depth_map_list.back()->depth_map[i].size();
    //         if(depth_map_list.back()->depth_map[i].size() > max_num ) max_num = depth_map_list.back()->depth_map[i].size();
    //         // if(depth_map_list.back()->depth_map[i].size() > 10 ) cout<< depth_map_list.back()->depth_map[i].size() << " , " << i/MAX_1D_HALF << " , " << i%MAX_1D_HALF<< " "<< depth_map_list.back()->depth_map[i][0]->glob.transpose() <<endl;
    //     }
    // }
    // cout<<" !!!!!!!!!!!!!!!!!!!!!!!! search times: "<< times_search << " in neighbor: "<< neighbor_in << " points in neighbor: "<< neighbor_num <<" points in map: " << map_nums <<" pixel with pointy: "\
    // << endl;//<< pixel_num <<" min: "<< min_num << " max "<< max_num << " !!!!!!!!!!!!!!!! "<< " search time in case 2 :" << time_search <<"  "
	int num_1 = 0, num_2 = 0, num_3 = 0, num_inval = 0, num_neag = 0;

    double clus_before = omp_get_wtime(); //rec computation time
    std_msgs::Header header_clus;
    header_clus.stamp = ros::Time().fromSec(scan_end_time);
    header_clus.frame_id = frame_id;
    if (cluster_coupled || cluster_future)
    {
        Cluster.Clusterprocess(dyn_tag_cluster, *laserCloudDynObj, raw_points_world, header_clus, rot_end, pos_end);
        for(int i = 0; i < size; i++)
        {
            PointType po;
            po.x = points[i]->glob(0);
            po.y = points[i]->glob(1);
            po.z = points[i]->glob(2);
            po.normal_x = points[i]->r;
            po.normal_y = points[i]->g;
            po.normal_z = points[i]->b;

//            std::cout << "po rgb: " << po.normal_x << " " << po.normal_y << " " << po.normal_z << std::endl;

            // PointType po_b;
            // po_b.x = points[i]->local(0);
            // po_b.y = points[i]->local(1);
            // po_b.z = points[i]->local(2);
            // V3D p_body(po.x, po.y, po.z);
            // double horizon_angle = atan2f(double(p_body(1)), double(p_body(0)));
            // double vertical_angle = atan2f(double(p_body(2)), sqrt(pow(double(p_body(0)), 2) + pow(double(p_body(1)), 2)));
            // if(horizon_angle > fov_right || horizon_angle < fov_left || vertical_angle < -fov_down || vertical_angle > -fov_up) continue;
            po.curvature = i;
            switch (points[i]->dyn)
            {
                case CASE1:
                    if (dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
//                        po.normal_x = 0;
//                        po.normal_y = points[i]->is_occu_times;
//                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else // case1
                    {
//                        po.normal_x = 1;
//                        po.normal_y = points[i]->is_occu_times;
//                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                            // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                            // laserCloudSteadObj_clus->push_back(po);
                        }
                        num_1 += 1;
                    }
                    break;
                case CASE2:
                    if(dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
//                        po.normal_x = 0;
//                        po.normal_y = points[i]->is_occu_times;
//                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else
                    {
//                        po.normal_x = 0;
//                        po.normal_y = points[i]->is_occu_times;
//                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                            // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                            // laserCloudSteadObj_clus->push_back(po);
                        }
                        num_2 += 1;
                    }
                    break;
                case CASE3:
                    if(dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
//                        po.normal_x = 0;
//                        po.normal_y = points[i]->is_occu_times;
//                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else
                    {
//                        po.normal_x = 0;
//                        po.normal_y = points[i]->is_occu_times;
//                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                            // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                            // laserCloudSteadObj_clus->push_back(po);
                        }
                        num_3 += 1;
                    }
                    break;
                case STATIC:
                    if(dyn_tag_cluster[i] == 1)
                    {
                        points[i]->dyn = CASE1;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
//                        po.normal_x = 0;
//                        po.normal_y = points[i]->is_occu_times;
//                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                            // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                            // laserCloudSteadObj_clus->push_back(po);
                        }
                        num_1 += 1;
                    }
                    else
                    {
//                        po.normal_x = 0;
//                        po.normal_y = points[i]->is_occu_times;
//                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    break;
                default: //invalid
                    num_inval += 1;
                    break;
            }
        }
    }

    if(time_file != "") time_out << omp_get_wtime()-clus_before << " "; //rec computation time




    double t3 = omp_get_wtime();
    Points2Buffer(points, index);
    double t4 = omp_get_wtime();
    if(time_file != "") time_out << omp_get_wtime()-t3 << " "; //rec computation time
    Buffer2DepthMap(scan_end_time);
    std::cout << "time_handle buffer: " << omp_get_wtime()-t4 << std::endl;
    if(time_file != "") time_out << omp_get_wtime()-t3 << endl; //rec computation time
    if (cluster_coupled)
    {
        // laserCloudDynObj = laserCloudDynObj_clus;
        // laserCloudSteadObj = laserCloudSteadObj_clus;
        for(int i = 0; i < size; i++)
        {
            if (dyn_tag_cluster[i] == 1)
            {
                if(is_rec)
                {
                    int tmp = 251;
                    out.write((char*)&tmp, sizeof(int));
                }
            }
            else
            {
                if(is_rec)
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                }
            }

            if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
            {
                if(is_rec_origin)
                {
                    int tmp = 251;
                    out_origin.write((char*)&tmp, sizeof(int));
                }
            }
            else
            {
                if(is_rec_origin)
                {
                    int tmp = 9;
                    out_origin.write((char*)&tmp, sizeof(int));
                }
            }
        }
    }
    else
    {
        for(int i = 0; i < size; i++)
        {
            if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
            {
                if(is_rec)
                {
                    int tmp = 251;
                    out.write((char*)&tmp, sizeof(int));
                }
            }
            else
            {
                if(is_rec)
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                }
            }
        }
    }

    if(draw_depth) Draw_depthmap(feats_undistort, dyn_tag_cluster);
    for (int i = 0; i < size; i++)
    {
        if(dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
        {
            pbp_file << feats_undistort->at(i).curvature / 1000 + frame_num_for_rec * frame_dur << " " << feats_undistort->at(i).x << " " << feats_undistort->at(i).y << " " << feats_undistort->at(i).z  << " " << 1 << endl;
        }
        else
        {
            pbp_file << feats_undistort->at(i).curvature / 1000 + frame_num_for_rec * frame_dur << " " << feats_undistort->at(i).x << " " << feats_undistort->at(i).y << " " << feats_undistort->at(i).z << " " << 0 << endl;
        }
    }

    frame_num_for_rec ++;
    // for (int i = 0; i < size; i++)
    // {
    //     switch (points[i].dyn)
    //     {
    //         case CASE1:
    //             if(is_rec) out << 1 <<  "\n";
    //             num_1 += 1;
    //             break;
    //         case CASE2:
    //             if(is_rec) out << 1 <<  "\n";
    //             num_2 += 1;
    //             break;
    //         case CASE3:
    //             if(is_rec) out << 1 <<  "\n";
    //             num_3 += 1;
    //             break;
    //         case STATIC:
    //             if(is_rec) out << 0 <<  "\n";
    //             num_neag += 1;
    //             break;
    //         default:
    //             if(is_rec) out << 0 <<  "\n";
    //             num_inval += 1;
    //             break;
    //     }
    // }
    cur_point_soph_pointers = (cur_point_soph_pointers + 1)%max_pointers_num;
    if(is_rec) out.close();
    std::cout << "num_1 points: " << num_1 << " num_2 points: " << num_2 << " num_3 points: " << num_3 << " num_invalid points: " << num_inval << " num_steady: " << num_neag <<  std::endl;
    time_total = omp_get_wtime() - t00;
    time_ind ++;
    time_total_avr = time_total_avr * (time_ind - 1) / time_ind + time_total / time_ind;

    // rec_i ++;
    // std::cout << "time_all: " << time_all << std::endl;
}


void  DynObjFilter::filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time)
{
    // return;
    // cout<<"-----in1------"<<endl;
    double t00 = omp_get_wtime();
    time_search = time_research = time_search_0 = time_build = time_total = time_other0 = 0.0;
    time_interp1 = time_interp2 =0;
    
    int num_build = 0, num_search_0 = 0, num_research = 0;

    
    if (feats_undistort == NULL) return;
    int size = feats_undistort->points.size();

    /***--------------***/
    cout << "!!!!!save---origin!!!!!" << endl;
    string all_points_dir1("/home/weihairuo/bag/pcd_correct/origin_pointcloud/" + to_string(time_ind) + string(".pcd"));
    cout << "-----------" << all_points_dir1 << "-----------" << feats_undistort->size() << endl;
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir1, *feats_undistort);
    cout << "----finish---origin-----" << endl;
    /***--------------***/
    
    std::cout << "feats_undistort1 size: " << size << " time: "<< fixed << scan_end_time << std::endl;
    // cout<<"buffer: size: "<<buffer.size()<<endl;
    if (debug_en)
    {
        laserCloudSteadObj_hist.reset(new PointCloudXYZI());
        laserCloudSteadObj_hist->reserve(20*size);
    }
    dyn_tag_origin.clear();
    dyn_tag_origin.reserve(size);
    dyn_tag_origin.resize(size);
    dyn_tag_cluster.clear();
    dyn_tag_cluster.reserve(size);
    dyn_tag_cluster.resize(size);
    laserCloudDynObj.reset(new PointCloudXYZI());
    laserCloudDynObj->reserve(size);
    laserCloudDynObj_world.reset(new PointCloudXYZI());
    laserCloudDynObj_world->reserve(size);
    laserCloudSteadObj.reset(new PointCloudXYZI());
    laserCloudSteadObj->reserve(size);
    laserCloudDynObj_clus.reset(new PointCloudXYZI());
    laserCloudDynObj_clus->reserve(size);
    laserCloudSteadObj_clus.reset(new PointCloudXYZI());
    laserCloudSteadObj_clus->reserve(size);
    
    ofstream out;
    ofstream out_origin;
    bool is_rec = false;
    bool is_rec_origin = false;
    if(is_set_path)
    {
        out.open(out_file, ios::out  | ios::binary);
        out_origin.open(out_file_origin, ios::out | ios::binary);
        if (out.is_open()) 
        {
            is_rec = true;
        }
        if (out_origin.is_open()) 
        {
            is_rec_origin = true;
        }
    }


    int case2_num = 0;
    double t0 = omp_get_wtime();
    std::cout << "t0 - t00: " << t0- t00 << std::endl;
    double time_case1 = 0, time_case2 = 0, time_case3 = 0;
    
    pcl::PointCloud<PointType> raw_points_world;
    raw_points_world.reserve(size);
    raw_points_world.resize(size);

    std::vector<int> index(size);
    for (int i = 0; i < size; i++) {
        index[i] = i;
    }

    vector<point_soph*> points;
    points.reserve(size);
    points.resize(size);
    point_soph* p = point_soph_pointers[cur_point_soph_pointers];
    if(time_file != "") time_out << size << " "; //rec computation time
    std::for_each(std::execution::par, index.begin(), index.end(), [&](const int &i)
    // std::for_each(std::execution::seq, index.begin(), index.end(), [&](const int &i)
    {   
        p[i].reset();     
        should_print = (i == point_index);

        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
        int intensity = feats_undistort->points[i].intensity;

        V3D p_glob(rot_end * (p_body) + pos_end);
        p[i].glob = p_glob;
        
        p[i].dyn = STATIC;
        p[i].rot = rot_end.transpose();
        p[i].transl = pos_end;
        p[i].time = scan_end_time;
        p[i].local = p_body;

        if(dataset == 0 && fabs(intensity-666) < 10E-4)
        {
            p[i].is_distort = true;
        }
        // double t = omp_get_wtime();
        if (InvalidPointCheck(p_body, intensity))
        {   
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
            dyn_tag_cluster[i] = -1;
            // laserCloudSteadObj->push_back(po);
        }
        else if(SelfPointCheck(p_body, p[i].dyn))
        {
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
            // po.normal_x = -1;
            // laserCloudSteadObj->push_back(po);
        }
        else if (Case1(p[i]))
        {
            p[i].dyn = CASE1;
            // po.normal_x = 1;
            // laserCloudDynObj->push_back(po);
            dyn_tag_origin[i] = 1;
        }
        else if (Case2(p[i]))
        {
            p[i].dyn = CASE2;
            // po.normal_y = p[i].occu_times;
            // laserCloudDynObj->push_back(po);
            dyn_tag_origin[i] = 1;
        } 
        else if(Case3(p[i]))
        {
            p[i].dyn = CASE3;
            // po.normal_z = p[i].is_occu_times;
            // po.normal_x = p[i].vec(0);
            // po.normal_y = p[i].vec(1);
            // laserCloudDynObj->push_back(po);
            dyn_tag_origin[i] = 1;
        }
        else
        {
            // laserCloudSteadObj->push_back(po);
            dyn_tag_origin[i] = 0;
        }
        points[i] = &p[i];
        //  points[i] = p;
    });
    std::cout << "Case1/2/3 time: " << omp_get_wtime()-t0 << std::endl;
    if(time_file != "") time_out << omp_get_wtime()-t0 << " "; //rec computation time
    
    for(int i = 0; i < size; i++)
    {
        PointType po;
        po.x = points[i]->local[0];
        po.y = points[i]->local[1];
        po.z = points[i]->local[2];
        // V3D p_body(po.x, po.y, po.z);
        // double horizon_angle = atan2f(double(p_body(1)), double(p_body(0)));
        // double vertical_angle = atan2f(double(p_body(2)), sqrt(pow(double(p_body(0)), 2) + pow(double(p_body(1)), 2)));
        // if(horizon_angle > fov_right || horizon_angle < fov_left || vertical_angle < -fov_down || vertical_angle > -fov_up) continue;
        PointType po_w;
        po_w.x = points[i]->glob[0];
        po_w.y = points[i]->glob[1];
        po_w.z = points[i]->glob[2];
        raw_points_world[i] = po;
        switch(points[i]->dyn) // == CASE1 ||  || points[i]->dyn == CASE3)
        {
            case CASE1:
                po.normal_x = 1;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case CASE2:
                po.normal_y = points[i]->occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case CASE3:
                po.normal_z = points[i]->is_occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            default:
                laserCloudSteadObj->push_back(po_w);
        }
    }
    
    // int map_nums = 0, pixel_num = 0, min_num = 10000, max_num = 0;
    // if(depth_map_list.size() > 0)
    // {
    //     for(int i = 0; i < depth_map_list.back()->depth_map.size(); i++)
    //     {
    //         map_nums += depth_map_list.back()->depth_map[i].size();
    //         if(depth_map_list.back()->depth_map[i].size() > 0 ) pixel_num ++;
    //         if(depth_map_list.back()->depth_map[i].size() > 0 && depth_map_list.back()->depth_map[i].size() < min_num ) min_num = depth_map_list.back()->depth_map[i].size();
    //         if(depth_map_list.back()->depth_map[i].size() > max_num ) max_num = depth_map_list.back()->depth_map[i].size();
    //         // if(depth_map_list.back()->depth_map[i].size() > 10 ) cout<< depth_map_list.back()->depth_map[i].size() << " , " << i/MAX_1D_HALF << " , " << i%MAX_1D_HALF<< " "<< depth_map_list.back()->depth_map[i][0]->glob.transpose() <<endl; 
    //     }
    // }
    // cout<<" !!!!!!!!!!!!!!!!!!!!!!!! search times: "<< times_search << " in neighbor: "<< neighbor_in << " points in neighbor: "<< neighbor_num <<" points in map: " << map_nums <<" pixel with pointy: "\
    // << endl;//<< pixel_num <<" min: "<< min_num << " max "<< max_num << " !!!!!!!!!!!!!!!! "<< " search time in case 2 :" << time_search <<"  " 
	int num_1 = 0, num_2 = 0, num_3 = 0, num_inval = 0, num_neag = 0; 

    double clus_before = omp_get_wtime(); //rec computation time
    std_msgs::Header header_clus;
    header_clus.stamp = ros::Time().fromSec(scan_end_time);
    header_clus.frame_id = frame_id;
    if (cluster_coupled || cluster_future)
    {
        Cluster.Clusterprocess(dyn_tag_cluster, *laserCloudDynObj, raw_points_world, header_clus, rot_end, pos_end);
        for(int i = 0; i < size; i++)
        {   
            PointType po;
            po.x = points[i]->glob(0);
            po.y = points[i]->glob(1);
            po.z = points[i]->glob(2);

            // PointType po_b;
            // po_b.x = points[i]->local(0);
            // po_b.y = points[i]->local(1);
            // po_b.z = points[i]->local(2);
            // V3D p_body(po.x, po.y, po.z);
            // double horizon_angle = atan2f(double(p_body(1)), double(p_body(0)));
            // double vertical_angle = atan2f(double(p_body(2)), sqrt(pow(double(p_body(0)), 2) + pow(double(p_body(1)), 2)));
            // if(horizon_angle > fov_right || horizon_angle < fov_left || vertical_angle < -fov_down || vertical_angle > -fov_up) continue;
            po.curvature = i;
            switch (points[i]->dyn)
            {   
                case CASE1:               
                    if (dyn_tag_cluster[i] == 0) 
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else // case1
                    {
                        po.normal_x = 1;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                            // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                            // laserCloudSteadObj_clus->push_back(po);
                        }
                        num_1 += 1;
                    }
                    break;               
                case CASE2:
                    if(dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                            // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                            // laserCloudSteadObj_clus->push_back(po);
                        }
                        num_2 += 1;
                    }
                    break;
                case CASE3:
                    if(dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                            // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                            // laserCloudSteadObj_clus->push_back(po);
                        }
                        num_3 += 1;
                    }
                    break;       
                case STATIC:
                    if(dyn_tag_cluster[i] == 1)
                    {
                        points[i]->dyn = CASE1;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if(!dyn_filter_en)
                        {
                            po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                            // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                            // laserCloudSteadObj_clus->push_back(po);
                        }
                        num_1 += 1;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10;
                        // if(points[i]->local.norm() > 1.5) laserCloudSteadObj_clus->push_back(po);
                        laserCloudSteadObj_clus->push_back(po);
                        num_neag += 1;
                    }
                    break;               
                default: //invalid
                    num_inval += 1;
                    break;
            }
        }
    }

    if(time_file != "") time_out << omp_get_wtime()-clus_before << " "; //rec computation time
    
    
    
    
    double t3 = omp_get_wtime();
    Points2Buffer(points, index);
    double t4 = omp_get_wtime();
    if(time_file != "") time_out << omp_get_wtime()-t3 << " "; //rec computation time
    Buffer2DepthMap(scan_end_time);
    std::cout << "time_handle buffer: " << omp_get_wtime()-t4 << std::endl;
    if(time_file != "") time_out << omp_get_wtime()-t3 << endl; //rec computation time
    if (cluster_coupled)
    {   
        // laserCloudDynObj = laserCloudDynObj_clus;
        // laserCloudSteadObj = laserCloudSteadObj_clus;
        for(int i = 0; i < size; i++)
        {
            if (dyn_tag_cluster[i] == 1)
            {
                if(is_rec) 
                {
                    int tmp = 251;
                    out.write((char*)&tmp, sizeof(int));
                }
            } 
            else
            {
                if(is_rec)
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                } 
            } 

            if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
            {
                if(is_rec_origin) 
                {
                    int tmp = 251;
                    out_origin.write((char*)&tmp, sizeof(int));
                }
            } 
            else
            {
                if(is_rec_origin)
                {
                    int tmp = 9;
                    out_origin.write((char*)&tmp, sizeof(int));
                }
            } 
        }
    }
    else
    {
        for(int i = 0; i < size; i++)
        {
            if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
            {
                if(is_rec) 
                {
                    int tmp = 251;
                    out.write((char*)&tmp, sizeof(int));
                }
            } 
            else
            {
                if(is_rec)
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                }
            } 
        }
    }

    if(draw_depth) Draw_depthmap(feats_undistort, dyn_tag_cluster);
    for (int i = 0; i < size; i++)
    {
        if(dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
        {
            pbp_file << feats_undistort->at(i).curvature / 1000 + frame_num_for_rec * frame_dur << " " << feats_undistort->at(i).x << " " << feats_undistort->at(i).y << " " << feats_undistort->at(i).z  << " " << 1 << endl;
        }
        else
        {
            pbp_file << feats_undistort->at(i).curvature / 1000 + frame_num_for_rec * frame_dur << " " << feats_undistort->at(i).x << " " << feats_undistort->at(i).y << " " << feats_undistort->at(i).z << " " << 0 << endl;
        }
    }
    
    frame_num_for_rec ++;
    // for (int i = 0; i < size; i++)
    // {
    //     switch (points[i].dyn)
    //     {   
    //         case CASE1:                    
    //             if(is_rec) out << 1 <<  "\n";
    //             num_1 += 1;
    //             break;
    //         case CASE2:
    //             if(is_rec) out << 1 <<  "\n";
    //             num_2 += 1;
    //             break;
    //         case CASE3:
    //             if(is_rec) out << 1 <<  "\n";
    //             num_3 += 1;
    //             break;              
    //         case STATIC:               
    //             if(is_rec) out << 0 <<  "\n";
    //             num_neag += 1;
    //             break;                 
    //         default:
    //             if(is_rec) out << 0 <<  "\n";
    //             num_inval += 1;
    //             break;
    //     }
    // }
    cur_point_soph_pointers = (cur_point_soph_pointers + 1)%max_pointers_num;
    if(is_rec) out.close();
    std::cout << "num_1 points: " << num_1 << " num_2 points: " << num_2 << " num_3 points: " << num_3 << " num_invalid points: " << num_inval << " num_steady: " << num_neag <<  std::endl;
    time_total = omp_get_wtime() - t00;
    time_ind ++;
    time_total_avr = time_total_avr * (time_ind - 1) / time_ind + time_total / time_ind;

    // rec_i ++;
    // std::cout << "time_all: " << time_all << std::endl;
}

void  DynObjFilter::Points2Buffer(vector<point_soph*> &points, std::vector<int> &index_vector)
{
    int cur_tail = buffer.tail;
    // for(int k = 0; k < points.size(); k++)
    // {
    //     buffer.push_pos(points[k], cur_tail+k);
    // }
    // cout<<"buffer size: "<<buffer.size() << " , "<< points.size()<<" , "<<cur_tail<<endl;
    buffer.push_parallel_prepare(points.size());
    std::for_each(std::execution::par, index_vector.begin(), index_vector.end(), [&](const int &i)
    {   
        buffer.push_parallel(points[i], cur_tail+i);
    });
    int c_tail = buffer.tail;
    // cout<<"-------------buffer size: "<<buffer.size() << " , "<< points.size()<<" , "<<cur_tail<<" , " << c_tail<<endl;
}

void  DynObjFilter::Buffer2DepthMap(double cur_time)
{
    int len = buffer.size();
    double total_0 = 0.0;
    double total_1 = 0.0;
    double total_2 = 0.0;
    double total_3 = 0.0;
    double t = 0.0;
    int max_point = 0;
     std::cout << "cur_time: " << cur_time << " front time: " << buffer.front()->time << std::endl;

    
    // double in_map = 0;
    // double cal_p = 0;
    for (int k = 0; k < len; k++)
    {   
        // t = omp_get_wtime();
        point_soph* point = buffer.front();
        // total_3 += omp_get_wtime() - t;
//        std::cout << point->time <<", " << k << ", " << buffer_delay << " , " << frame_dur << std::endl;
        if ((cur_time - point->time) >= buffer_delay - frame_dur/2.0)
        {   
            if(depth_map_list.size() == 0)
            {
                if(depth_map_list.size() < dyn_windows_num)
                {
                    map_index ++;
                     cout<<"first map"<<endl;
                    // double t = omp_get_wtime();
        
                    DepthMap::Ptr new_map_pointer(new DepthMap(point->rot, point->transl, point->time, map_index));
//                     cout<<"finished first map "<<omp_get_wtime()-t <<endl;
                    depth_map_list.push_back(new_map_pointer);
                }
                else
                {
                    buffer.pop();
                    continue;
                }
                // map_index ++;
                // // cout<<"first map"<<endl;
                // // double t = omp_get_wtime();
    
                // DepthMap::Ptr new_map_pointer(new DepthMap(point->rot, point->transl, point->time, map_index));
                // // cout<<"finished first map"<<omp_get_wtime()-t <<endl;
                // depth_map_list.push_back(new_map_pointer);
            }
            else if((point->time - depth_map_list.back()->time) >= depth_map_dur - frame_dur/2.0)
            {
                 cout<<"-----------new map point_time: "<<point->time<<" , "<<depth_map_list.back()->time<<endl;
                map_index ++;
                if (depth_map_list.size() == max_depth_map_num)
                {   
                    // double t_build = omp_get_wtime();
                    depth_map_list.front()->Reset(point->rot, point->transl, point->time, map_index);
                    // std::cout << "reset old map time: " << omp_get_wtime() - t_build << std::endl;
                    DepthMap::Ptr new_map_pointer = depth_map_list.front();
                    depth_map_list.pop_front();
                    depth_map_list.push_back(new_map_pointer);
                    // std::cout << "build new map time: " << omp_get_wtime() - t_build << std::endl;
                }
                else
                {   
                    // double t_build = omp_get_wtime();
                    DepthMap::Ptr new_map_pointer(new DepthMap(point->rot, point->transl, point->time, map_index));
                    // std::cout << "build new map time2: " << omp_get_wtime() - t_build << std::endl;
                    depth_map_list.push_back(new_map_pointer);
                    // std::cout << "build new map time3: " << omp_get_wtime() - t_build << std::endl;
                }
            }
            // double t = omp_get_wtime();
            // double t1;
            switch (point->dyn)
            {
                case STATIC:
                    // t1 = omp_get_wtime();
                    SphericalProjection(*point, depth_map_list.back()->map_index, depth_map_list.back()->project_R, depth_map_list.back()->project_T, *point);
					// cal_p += (omp_get_wtime()-t1);;
                    
                    if(depth_map_list.back()->depth_map[point->position].size() < max_pixel_points)
                    {
                        depth_map_list.back()->depth_map[point->position].push_back(point);
                        if (point->vec(2) > depth_map_list.back()->max_depth_all[point->position])  
                        {
                            depth_map_list.back()->max_depth_all[point->position] = point->vec(2);
                            depth_map_list.back()->max_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size()-1;
                        }
                        if (point->vec(2) < depth_map_list.back()->min_depth_all[point->position] ||\
                            depth_map_list.back()->min_depth_all[point->position] < 10E-5)  
                        {
                            depth_map_list.back()->min_depth_all[point->position] = point->vec(2);
                            depth_map_list.back()->min_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size()-1;
                        }
                        if (point->vec(2) < depth_map_list.back()->min_depth_static[point->position] ||\
                            depth_map_list.back()->min_depth_static[point->position] < 10E-5)  
                        {
                            depth_map_list.back()->min_depth_static[point->position] = point->vec(2);
                        }
                        if (point->vec(2) > depth_map_list.back()->max_depth_static[point->position])  
                        {
                            depth_map_list.back()->max_depth_static[point->position] = point->vec(2);
                        }
                        
                    }
                    break;   
                case CASE1:                    
                
                case CASE2:

                case CASE3:
                    // t = omp_get_wtime();
                    // t1 = omp_get_wtime();
                    SphericalProjection(*point, depth_map_list.back()->map_index, depth_map_list.back()->project_R, depth_map_list.back()->project_T, *point);
                    // cal_p += (omp_get_wtime()-t1);
						// depth_map_list.back().depth_map[point.position].push_back(point);
                    
                    if(depth_map_list.back()->depth_map[point->position].size() < max_pixel_points)
                    {
                        depth_map_list.back()->depth_map[point->position].push_back(point);
                        if (point->vec(2) > depth_map_list.back()->max_depth_all[point->position])  
                        {
                            depth_map_list.back()->max_depth_all[point->position] = point->vec(2);
                            depth_map_list.back()->max_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size()-1;
                        }
                        if (point->vec(2) < depth_map_list.back()->min_depth_all[point->position] ||\
                            depth_map_list.back()->min_depth_all[point->position] < 10E-5)  
                        {
                            depth_map_list.back()->min_depth_all[point->position] = point->vec(2);
                            depth_map_list.back()->min_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size()-1;
                        }
                    }
                    
                    
                    // total_1 += omp_get_wtime() - t;
                    break;
                
                

                    
                    // total_1 += omp_get_wtime() - t;
                    // if(point->hor_ind == 7 && point->ver_ind == 77)
                    // {
                    //     cout<<"-----------add to map static! "<< depth_map_list.back()->depth_map[point->position].size() <<endl;
                    // }
                    

                default:           
                    {
                        // if (debug_en)
                        // SphericalProjection(point, depth_map_list.back().map_index, depth_map_list.back().project_R, depth_map_list.back().project_T, point);
                        // depth_map_list.back().depth_map[point.position].push_back(point);
                        // // cout<<"point is valid"<<endl;
                    }
            }
            // in_map += (omp_get_wtime()-t);
            // cout<<"push point in map"<<endl;
            buffer.pop();
        }
        else
        {
            // cout<<"-----------buffer size: "<<buffer.size()<<" time: "<<buffer.front().time<<" , "<<cur_time<<endl;
            // cout<<" minus: "<<cur_time - point.time<<" , "<<buffer_delay<<endl;
            break;
        }
    }
    // cout<<"in map:------------------- "<<in_map<< " cal: "<< cal_p <<endl;
    // last_point_pointer = point_pointer;
    if (debug_en)
    {   
        for (int i = 0; i < depth_map_list.size(); i++)
        {
            for (int j = 0; j < depth_map_list[i]->depth_map.size(); j++)
            {
                for (int k = 0; k < depth_map_list[i]->depth_map[j].size(); k++)
                {
                    PointType po;
                    point_soph* point = depth_map_list[i]->depth_map[j][k];
                    po.x = point->glob(0);
                    po.y = point->glob(1);
                    po.z = point->glob(2);
                    po.intensity = point->local(2);
                    po.curvature = point->local(1);
                    po.normal_x = point->hor_ind;
                    po.normal_y = point->ver_ind;
                    if (po.normal_x == 0 && po.normal_y == 0)
                    {
                        cout<<" all 0: "<<point->glob.transpose()<<endl;
                    }
                    po.normal_z = point->dyn;
                    if(point->dyn == STATIC) laserCloudSteadObj_hist->push_back(po);
                }
            }
        }
    }
}

void  DynObjFilter::SphericalProjection(point_soph &p, int depth_index, const M3D &rot, const V3D &transl, point_soph &p_spherical)
{
    if(fabs(p.last_vecs.at(depth_index%HASH_PRIM)[2]) > 10E-5)
    {
        
        p_spherical.vec = p.last_vecs.at(depth_index%HASH_PRIM);
        p_spherical.hor_ind = p.last_positions.at(depth_index%HASH_PRIM)[0];
        p_spherical.ver_ind = p.last_positions.at(depth_index%HASH_PRIM)[1];
        p_spherical.position = p.last_positions.at(depth_index%HASH_PRIM)[2];
        
    }
    else
    {
        V3D p_proj(rot*(p.glob - transl));
        //// p_spherical.local = p_proj;
        p_spherical.GetVec(p_proj, hor_resolution_max, ver_resolution_max);
        p.last_vecs.at(depth_index%HASH_PRIM) = p_spherical.vec;
        p.last_positions.at(depth_index%HASH_PRIM)[0] = p_spherical.hor_ind;
        p.last_positions.at(depth_index%HASH_PRIM)[1] = p_spherical.ver_ind;
        p.last_positions.at(depth_index%HASH_PRIM)[2] = p_spherical.position;
    }
    // if(should_print)
    // {
    //     V3D p_proj(rot*(p.glob - transl));
    //     cout<<" projection local cor: "<< p_proj.transpose() << " theta and depth: "<< p_spherical.vec.transpose() << " index: "\
    //     <<p_spherical.hor_ind <<" , "<<p_spherical.ver_ind <<endl;
    // }
    // if(should_print)
    // {
    //     V3D p_proj(rot*(p.glob - transl));
    //     cout<<" projection local cor: "<< p_proj.transpose() << " theta and depth: "<< p_spherical.vec.transpose() << " index: "\
    //     <<p_spherical.hor_ind <<" , "<<p_spherical.ver_ind <<endl;
    // }
    
    // if(isnan(p_spherical.vec(0)) || isnan(p_spherical.vec(1)) || isnan(p_spherical.vec(2)))
    // {
    //     cout<<"-------error---------"<<p_spherical.vec.transpose()<<"  ,  "<<p.glob.transpose()<<endl;
    // }

    

}

bool  DynObjFilter::InvalidPointCheck(const V3D &body, const int intensity)
{
    if(should_print)
        cout<<"invalid: "<<body(0)<<"   ,    "<<body(1)<<endl;
    if ( (pow(body(0), 2) + pow(body(1), 2) + pow(body(2), 2)) < blind_dis*blind_dis || (dataset == 1 && fabs(body(0)) < 0.1 && fabs(body(1)) < 1.0) && fabs(body(2)) < 0.1)
    {
        return true;
    } 
    // else if (intensity < 667 && intensity > 665){&& body(2) > -0.3
        // return true;
    // }
    else
    {
        return false;
    }
}

bool  DynObjFilter::SelfPointCheck(const V3D &body, const dyn_obj_flg dyn)
{
    if(should_print) cout<<"body self: "<<body(0)<<" , "<<body(1)<<endl;
    if (dataset == 0 )
    {
        
        if( (body(0) > -1.2 && body(0) < -0.4 && body(1) > -1.7 && body(1) < -1.0 && body(2) > -0.65 && body(2) < -0.4) || \
            (body(0) > -1.75 && body(0) < -0.85 && body(1) > 1.0 && body(1) < 1.6 && body(2) > -0.75 && body(2) < -0.40) || \
            (body(0) > 1.4 && body(0) < 1.7 && body(1) > -1.3 && body(1) < -0.9 && body(2) > -0.8 && body(2) < -0.6) || \
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > -0.6 && body(1) < -0.45 && body(2) > -1.0 && body(2) < -0.9) || \
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > 0.45 && body(1) < 0.6 && body(2) > -1.0 && body(2) < -0.9) )
        {
            return true;
        }
        else
        {
            return false;
        }
    } 
    return false;
    // else 
    // {
    //     if(body(0) > self_x_b && body(0) < self_x_f && body(1) < self_y_l && body(1) > self_y_r)
    //     {
    //         return true;
    //     }
    //     else
    //     {
    //         return false;
    //     }
    // }
}

bool  DynObjFilter::CheckVerFoV(const point_soph & p, const DepthMap &map_info)
{
    // return true;
    bool ver_up = false, ver_down = false;
    for(int i = p.ver_ind; i >= pixel_fov_down; i--)
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(should_print) cout<<" check verfov: "<<p.hor_ind<<" , ver: "<<i<<" size: "<< map_info.depth_map[cur_pos].size()<<endl;
        if(map_info.depth_map[cur_pos].size() > 0)
        {
            ver_down = true;
            break;
        }
    } 
    for(int i = p.ver_ind; i <= pixel_fov_up; i++)
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(should_print) cout<<" check verfov: "<<p.hor_ind<<" , ver: "<<i<<" size: "<< map_info.depth_map[cur_pos].size()<<endl;
        if(map_info.depth_map[cur_pos].size() > 0)
        {
            ver_up = true;
            break;
        }
    }   
    if(ver_up && ver_down)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void DynObjFilter::CheckNeighbor(const point_soph & p, const DepthMap &map_info, float &max_depth, float &min_depth)
{   
    int n = checkneighbor_range;
    for (int i = -n; i <= n; i++)
    {
        for (int j = -n; j <= n; j++)
        {
            int cur_pos = (p.hor_ind + i) * MAX_1D_HALF + p.ver_ind + j;
            if(cur_pos < MAX_2D_N && cur_pos >= 0 && map_info.depth_map[cur_pos].size() > 0)
            {
                // if(should_print) cout<<"neighbor pixel hor ind: " << p.hor_ind + i << ", ver ind: " << p.ver_ind + j  << ", size" << map_info.depth_map[cur_pos].size()<< endl;
                float cur_max_depth = map_info.max_depth_static[cur_pos];
                float cur_min_depth = map_info.min_depth_static[cur_pos];
                if(min_depth > 10E-5) min_depth = std::min(cur_min_depth, min_depth);
                else min_depth = cur_min_depth;
                
                if(max_depth > 10E-5) max_depth = std::max(cur_max_depth, max_depth);
                else max_depth = cur_max_depth;

                // if (p.vec(2) < min_depth - enter_min_thr1 || \
                // (min_depth < p.vec(2) - enter_min_thr1 && max_depth > p.vec(2) + enter_max_thr1))
                // {
                //     return true;
                // }
            }
        }
    }
    // return false;
}

void  DynObjFilter::Draw_depthmap(PointCloudXYZI::Ptr feats_undistort, std::vector<int> dyn_tag)
{   
    double fov_left = -0.349;
    double fov_right = 0.698;
    double fov_up = -0.06;
    double fov_down = 0.36;
    double resolution = 0.003; // 0.03
    int circle_size = 2; //10
    double rotate_angle_unit = 0.0; // 0.02
    double hor_resolution_max = 0.001;
    double ver_resolution_max = 0.001;
    const char* source_window = "Source image";

    pcl::PointCloud<pcl::PointXYZI> reorder_points;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> points_by_line;
    std::vector<std::vector<int>> tag_by_line;
    points_by_line.resize(100);
    tag_by_line.resize(100);
    bool jump_flag = false;
    int line_index = 1;
    int line_index_change = 0;
    double last_horizon_angle =1.0;
    for (int i = 0; i < feats_undistort->size(); i++)
    {   
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
        double horizon_angle = atan2f(double(p_body(1)), double(p_body(0)));
        double vertical_angle = atan2f(double(p_body(2)), sqrt(pow(double(p_body(0)), 2) + pow(double(p_body(1)), 2)));
        if (horizon_angle >= 0.0 && horizon_angle < 1.0 && last_horizon_angle <= 0.0  && last_horizon_angle > -1.0 && line_index_change > 100)
        {
            line_index ++;
            line_index_change = 0;
            jump_flag = false;
        }
        else line_index_change++;
        if(last_horizon_angle < -2.5 && horizon_angle > 2.5) jump_flag = true;
        last_horizon_angle = horizon_angle;
        pcl::PointXYZI point;
        point.x = p_body(0);
        point.y = p_body(1);
        point.z = p_body(2);
        point.intensity = line_index;
        points_by_line[line_index].push_back(point);
        tag_by_line[line_index].push_back(dyn_tag[i]);
    }

    float current_angle = fov_left;
    float rotate_angle = rotate_angle_unit;
    int k = 0;

    int hor_range = (fov_right + (fov_right - fov_left)/resolution * rotate_angle_unit - fov_left) / hor_resolution_max ; //+  floor((fov_down - fov_up) / ver_resolution_max)/8;
    int ver_range = (fov_down- fov_up) / ver_resolution_max;
    cv::Mat color = Mat::zeros(Size(hor_range, ver_range), CV_8UC3);
    color.setTo(255);
    while(current_angle < fov_right)
    {
        for (int i = 1; i < 66; i++)
        {   
            // if (i%4 == 0 ) continue;
            // if (i%4 == 2 ) continue;
            // if (i%4 == 3 ) continue;
            for(int j = 0; j < points_by_line[i].size(); j++)
            {   
                double horizon_angle = atan2f(double(points_by_line[i][j].y), double(points_by_line[i][j].x));
                double vertical_angle = atan2f(double(points_by_line[i][j].z), sqrt(pow(double(points_by_line[i][j].x), 2) + pow(double(points_by_line[i][j].y), 2)));
                double range = sqrt(pow(double(points_by_line[i][j].x), 2) + pow(double(points_by_line[i][j].y), 2) + pow(double(points_by_line[i][j].z), 2));
                if (current_angle < horizon_angle && (current_angle + resolution) > horizon_angle)
                {   
                    horizon_angle += rotate_angle;
                    pcl::PointXYZI point;
                    point.z = range * sin(vertical_angle); 
                    double project_len = range * cos(vertical_angle); 
                    point.x = project_len * cos(horizon_angle); 
                    point.y = project_len * sin(horizon_angle); 
                    point.intensity = range;
                    reorder_points.push_back(point);
                    int hor_ind   = floor((-horizon_angle + fov_right + (fov_right - fov_left)/resolution * rotate_angle_unit) / hor_resolution_max);
                    int ver_ind   = floor((-vertical_angle - fov_up) / ver_resolution_max);
                    // hor_ind += ver_ind/8;
                    Scalar point_rgb;
                    const tinycolormap::Color color_rgb = tinycolormap::GetColor(range/80., tinycolormap::ColormapType::Jet);
                    point_rgb << color_rgb.r() * 255, color_rgb.g()*255, color_rgb.b()*255;
                    if(tag_by_line[i][j] > 0)
                    {
                        point_rgb << 0, 0, 255;
                        cv::circle(color, cv::Point(hor_ind,ver_ind), 3, point_rgb, -1);
                    }
                    else
                    {   
                        point_rgb << 0, 0, 0;
                        cv::circle(color, cv::Point(hor_ind,ver_ind), circle_size, point_rgb, -1);
                    }
                    // cv::circle(color, cv::Point(hor_ind,ver_ind), circle_size, point_rgb, -1);
                    break;
                }
            }
        }
        current_angle += resolution;
        k ++;
        rotate_angle += rotate_angle_unit;
    }
    // for(int j = 0; j < feats_undistort->size(); j++)
    // {   
    //     double horizon_angle = atan2f(double(feats_undistort->points[j].y), double(feats_undistort->points[j].x));
    //     double vertical_angle = atan2f(double(feats_undistort->points[j].z), sqrt(pow(double(feats_undistort->points[j].x), 2) + pow(double(feats_undistort->points[j].y), 2)));
    //     double range = sqrt(pow(double(feats_undistort->points[j].x), 2) + pow(double(feats_undistort->points[j].y), 2) + pow(double(feats_undistort->points[j].z), 2));
    //     // if(horizon_angle < -fov_left && horizon_angle > -fov_right && vertical_angle < -fov_up && vertical_angle > -fov_down)
    //     // {
    //         pcl::PointXYZI point;
    //         point.z = range * sin(vertical_angle); 
    //         double project_len = range * cos(vertical_angle); 
    //         point.x = project_len * cos(horizon_angle); 
    //         point.y = project_len * sin(horizon_angle); 
    //         point.intensity = range;
    //         int hor_ind   = floor((-horizon_angle + fov_right + (fov_right - fov_left)/resolution * rotate_angle_unit) / hor_resolution_max);
    //         int ver_ind   = floor((-vertical_angle - fov_up) / ver_resolution_max);
    //         // hor_ind += ver_ind/6;
    //         cv::Scalar point_rgb;
    //         // if(range> 50) range = 50;

    //         const tinycolormap::Color color_rgb = tinycolormap::GetColor(range/80., tinycolormap::ColormapType::Jet);
    //         // rgb(range/80, point_rgb);
    //         point_rgb << color_rgb.r() * 255, color_rgb.g()*255, color_rgb.b()*255;
    //         if(dyn_tag[j] > 0)
    //         {
    //             point_rgb << 0, 0, 255;
    //             cv::circle(color, cv::Point(hor_ind,ver_ind), circle_size, point_rgb, -1);
    //         }
    //         else
    //         {   
    //             point_rgb << 255, 255, 255;
    //             cv::circle(color, cv::Point(hor_ind,ver_ind), circle_size, point_rgb, -1);
    //         }
    //         // break;
    //     // }
    // }
    stringstream ss;
    ss << cur_point_soph_pointers;
    string str = ss.str();
    cv::imwrite("/home/yihang/Pictures/2" + str + ".jpg", color);
    cv::namedWindow( source_window, WINDOW_NORMAL);
    cv::imshow( source_window, color);
    cv::waitKey(100);
}

bool  DynObjFilter::Case1(point_soph & p)
{
    // if(p.is_distort) return false;
    int depth_map_num = depth_map_list.size();
    int occluded_map = depth_map_num;
    for (int i = depth_map_num- 1; i >= 0; i--)
    {   
        // point_soph p_spherical = p;
        SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);
        // p = p_spherical;
        if (fabs(p.hor_ind) > MAX_1D || fabs(p.ver_ind) > MAX_1D_HALF || p.vec(2) < 0.0f \
            || p.position < 0 || p.position >= MAX_2D_N)
        {
            p.dyn = INVALID;
            cout<<"~~~~INVALID POINTS!!!!~~~~~ hor_ind: "<<p.hor_ind<<" ver_ind: "<<p.ver_ind<<endl;
            continue;
        }
        // if(should_print) cout<< "index: "<<p.hor_ind<<" , "<< p.ver_ind << " in map "<< depth_map_list[i]->map_index << endl;
        if (Case1Enter(p, *depth_map_list[i]))
        {// 
            // std::cout << "the " << i << " depth map, it's dizhi is : " << depth_map_list[i] << std::endl;
            if ((!Case1DepthConsistencyCheck(p, *depth_map_list[i])) || Case1FalseRejection(p, *depth_map_list[i]))
            {
                // if (i == depth_map_list.size()- 1) 
                // {
                //     return false;
                // }
                occluded_map -= 1;
                if(should_print) cout<<"false rejection"<<endl;
            }
        }
        else
        {
            occluded_map -= 1;
        }
        
        if(should_print) cout<<"case1 occ map: "<<occluded_map<<" , "<<i<<" , "<<occluded_map_thr1<<endl;

        if (occluded_map < occluded_map_thr1)
        {   
            return false;
        }
        if (occluded_map -(i) >= occluded_map_thr1)
        {
            // if(should_print) cout<<"case1 occ map: "<<occluded_map<<" , "<<i<<" , "<<occluded_map_thr1<<endl;
            return true;
        }
    }
    if(occluded_map >= occluded_map_thr1)
    {
        return true;
    }
    return false;  
}

bool  DynObjFilter::Case1Enter(const point_soph & p, const DepthMap &map_info)
{
    float max_depth = 0, min_depth = 0;
    float max_depth_all = 0, min_depth_all = 0;
    // cout<<"--------pixel size: "<<map_info.depth_map[p.position].size()<<endl;
    if (map_info.depth_map[p.position].size() > 0)
    {
        max_depth = map_info.max_depth_static[p.position];
        min_depth = map_info.min_depth_static[p.position];
        // max_depth_all = map_info.max_depth_all[p.position];
        // min_depth_all = map_info.min_depth_all[p.position];
    }
    //                        (min_depth <10E-5 && fabs(p.vec(2)-max_depth) < cur_depth) ||\  optional TBD
    else //if(dataset == 0)
    {
        // max_depth = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map); 
        if(p.ver_ind <= pixel_fov_up && p.ver_ind >pixel_fov_down && \
           p.hor_ind <= pixel_fov_left && p.hor_ind >= pixel_fov_right && \
           CheckVerFoV(p, map_info))
        {
            CheckNeighbor(p, map_info, max_depth, min_depth);
            if(should_print) cout<<"min depth : "<<min_depth<<" ,, "<<max_depth<<endl;
        }
        // else 
        // { && p.vec(2) > 15
        if(should_print)
            cout<<"ver_ind: "<<p.ver_ind<< " , "<<pixel_fov_up<< \
            " , "<<pixel_fov_down<<", hor_ind: "<<p.hor_ind<<" , "<<pixel_fov_right<< \
            " checkVerfov: " << CheckVerFoV(p, map_info) << endl;
        // }
         
        // min_depth = -depth_thr1_max;
    }        
    float cur_min = enter_min_thr1;
    float cur_max = enter_max_thr1;
    float cur_depth = depth_thr1;
    if(dataset == 0 && p.is_distort) 
    {
        cur_min = enlarge_distort*enter_min_thr1;
        cur_max = enlarge_distort*enter_max_thr1;
        cur_depth = enlarge_distort*depth_thr1;
    }
    if(should_print) 
    {
        cout<<"case1 enter: "<<p.vec(2)<<" min: "<<min_depth<<" max: "<<max_depth<<" "\
                    <<map_info.depth_map[p.position].size()<<endl;
    }

    if (p.vec(2) < min_depth - cur_max || \
        (min_depth < p.vec(2) - cur_min && max_depth > p.vec(2) + cur_max)||\
        (stop_object_detect && min_depth <10E-5 && max_depth < 10E-5 && map_info.depth_map[p.position].size() > 0 && p.vec(2) < map_info.max_depth_all[p.position] + 1.0)
        )
    // ||\(max_depth > 10E-5 && p.vec(2) > max_depth + 10*cur_depth)
    // if (p.vec(2) < min_depth - cur_min || \
    //     (min_depth < p.vec(2) - cur_min && max_depth > p.vec(2) + cur_max))
    {
        case1_num ++;
        return true;
    }
    if(should_print) 
    {
        cout<<"case1 enter failed: "<<p.vec(2)<<" min: "<<min_depth<<" max: "<<max_depth<<" "\
                    <<map_info.depth_map[p.position].size()<<endl;
        for (int ind_hor = p.hor_ind - map_cons_hor_num1; ind_hor <= p.hor_ind+map_cons_hor_num1; ind_hor ++)
        { 
            for (int ind_ver = p.ver_ind - map_cons_ver_num1; ind_ver <= p.ver_ind + map_cons_ver_num1; ind_ver ++)
            {
                // int ind = ind_hor*(2*ver_num + 1) + ind_ver;
                int pos_new = ind_hor* MAX_1D_HALF + ind_ver;
                if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
                cout<<"case1 enter failed: "<<p.vec(2)<<" min: "<<map_info.min_depth_static[pos_new]<<\
                    " max: "<<map_info.max_depth_static[pos_new]<<" "\
                    <<map_info.depth_map[pos_new].size()<<" , "<<pos_new<<" hor: "<<ind_hor<<\
                    " ver: "<<ind_ver<<endl;      
            }
            
        }
    }
    return false;
}

bool  DynObjFilter::Case1FalseRejection(point_soph & p, const DepthMap &map_info)
{
    return Case1MapConsistencyCheck(p, map_info, case1_interp_en);
}

bool  DynObjFilter::Case1MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp)
{   
    float hor_half = max(map_cons_hor_dis1/(max(p.vec(2), blind_dis)), map_cons_hor_thr1);
    float ver_half = max(map_cons_ver_dis1/(max(p.vec(2), blind_dis)), map_cons_ver_thr1);
    // float hor_half = max(map_cons_hor_dis1/(max(p.vec(2), 0.3f)), map_cons_hor_thr1);
    // float ver_half = max(map_cons_ver_dis1/(max(p.vec(2), 0.3f)), map_cons_ver_thr1);
    float cur_map_cons_depth_thr1 = map_cons_depth_thr1;
    float cur_map_cons_min_thr1 = enter_min_thr1;
    float cur_map_cons_max_thr1 = enter_max_thr1;
    // cout<<"hor num:"<<hor_half<<" , "<<ver_half<<" , "<<p.vec(2)<<endl;
    // if(p.vec(2) < 0.2) cout<<p.glob.transpose()<<p.local.transpose()<<endl;
    // if(p.vec(2) > 20)
    //     cur_map_cons_depth_thr1 += ((p.vec(2) - 20)* 0.1 + 0.5);
    if(dataset == 0 && p.is_distort) 
    {
        cur_map_cons_depth_thr1 = enlarge_distort*cur_map_cons_depth_thr1;
        cur_map_cons_min_thr1 = enlarge_distort*cur_map_cons_min_thr1;
        cur_map_cons_max_thr1 = enlarge_distort*cur_map_cons_max_thr1;
    }
    if (fabs(p.vec(1)) < enlarge_z_thr1 / 57.3)//(fabs(p.local(2)+0.15) < enlarge_z_thr1)
    {
        hor_half = enlarge_angle * hor_half;
        ver_half = enlarge_angle * ver_half;
        cur_map_cons_depth_thr1 = enlarge_depth * cur_map_cons_depth_thr1;
    }
    // if (should_print)
    // {
    //     cout << "cur point body z: " << p.local(2) <<" min_angel: "<<hor_half<<" "<<ver_half<<" "<<\
    //     cur_map_cons_depth_thr1<< endl;
    // }

    int cur_map_cons_hor_num1 = min(ceil(hor_half/hor_resolution_max),50.0f);
    int cur_map_cons_ver_num1 = min(ceil(ver_half/ver_resolution_max),50.0f);
    int num = 0;
    if(should_print)cout<<"hor num:"<<cur_map_cons_hor_num1<<" , "<<cur_map_cons_ver_num1<<" , "\
    <<hor_half<<" , "<<ver_half<<" , "<<p.vec(2)<<endl;
    point_soph closest_point;
    float closest_dis = 100;
    // for (int ind_hor = 0; ind_hor < 2*cur_map_cons_hor_num1 + 1; ind_hor ++)
    // {
    //     for (int ind_ver = 0; ind_ver < 2*cur_map_cons_ver_num1 + 1; ind_ver ++)
    //     {  
    for (int ind_hor = -cur_map_cons_hor_num1; ind_hor <= cur_map_cons_hor_num1; ind_hor ++)
    {
        for (int ind_ver = -cur_map_cons_ver_num1; ind_ver <= cur_map_cons_ver_num1; ind_ver ++)
        {  
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;
            // if(ind >= pos_offset.size()) 
            // cout<<"-------------error-------------"<<ind<<" , "<<pos_offset.size()<<" "<<cur_map_cons_hor_num1 <<" "<< cur_map_cons_ver_num1<<\
            // map_cons_hor_dis1/p.vec(2)<<" , "<<map_cons_hor_thr1<<" , "<< p.vec(2) <<" , "<< ceil(hor_half/hor_resolution_max) <<endl;
            // int pos_new = p.position + pos_offset[ind];
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            // if(pos_new >= map_info.depth_map.size()) cout<<"-------------error-------------"<<pos_new<<" , "<<map_info.depth_map.size()<<endl;
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];                        
            // cout<<"points size: "<<map_info.depth_map.size<<" "<<pos_new<<endl;cur_map_cons_depth_thr1
            if (map_info.max_depth_static[pos_new] < p.vec(2) - cur_map_cons_min_thr1 || \
                map_info.min_depth_static[pos_new] > p.vec(2) + cur_map_cons_max_thr1)
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j];
                if(should_print)
                {
                    // if(point->glob[0] == debug_x && point->glob[1] == debug_y && point->glob[2] == debug_z)
                    {
                        cout<<"map cons1 points: "<< (p.vec(0) -point->vec(1))  << " " <<(p.vec(1)-point->vec(1))\
                        <<" min_angel: "<<hor_half<<" "<<ver_half<<" "<<(p.vec(2)-point->vec(2))<<" "<<\
                        cur_map_cons_depth_thr1<<" "<<cur_map_cons_min_thr1<<" global cor: "<< point->glob.transpose()<< ", dyn: " << point->dyn << endl;
                    }
                }

                // if(p.vec(2)-point->vec(2) > 0 && (p.glob-point->glob).norm() < closest_dis)
                // {
                //     closest_point = *point;
                //     // closest_dis = p.vec(2)-point->vec(2);
                //     closest_dis = (p.glob-point->glob).norm();
                // }
                // std::cout << "point: "<<  (long)point << std::endl;
                // std::cout << "sizeof(*point):" << sizeof(*point) << std::endl;
                if (point->dyn == STATIC &&\
                  (fabs(p.vec(2)-point->vec(2)) <  cur_map_cons_depth_thr1 ||\
                   ((p.vec(2)-point->vec(2)) >  cur_map_cons_depth_thr1 && (p.vec(2)-point->vec(2)) <  cur_map_cons_min_thr1)) && \
                    fabs(p.vec(0)-point->vec(0)) < hor_half && \
                  fabs(p.vec(1)-point->vec(1)) < ver_half)
                {
                    if(should_print) cout<<"map cons1: "<<point->glob.transpose()<<\
                    " "<<fabs(p.vec(0)-point->vec(0))<<" "<<fabs(p.vec(1)-point->vec(1))\
                    <<" , "<<p.vec(2)-point->vec(2)\
                    <<" min_angel: "<<hor_half<<" "<<ver_half<<" "<<p.vec(2)<<" "<<\
                    map_cons_hor_dis1<<" "<<map_cons_hor_dis1/p.vec(2)<<endl;
                    if(should_print) cout << "closest front point: " << closest_point.glob.transpose() << " , closest distance: " << closest_dis << endl;
                    // p.last_closest = closest_point.glob;
                    return true;
                }    
            }         
        }
        
    }
    if(should_print) cout << "closest front point: " << closest_point.glob.transpose() << " , closest distance: " << closest_dis << endl;
    // if(p.last_closest.norm() > 0.1)
    // {   
    //     if(should_print) cout << "last closest front point: " << p.last_closest.transpose() << endl;
    //     V3D p_proj_last(map_info.project_R*(p.last_closest -  map_info.project_T));
    //     V3F vec_last;
    //     vec_last(2)    = float(p_proj_last.norm());
    //     vec_last(0)    = atan2f(float(p_proj_last(1)), float(p_proj_last(0)));
    //     vec_last(1)    = atan2f(float(p_proj_last(2)), sqrt(pow(float(p_proj_last(0)), 2) + pow(float(p_proj_last(1)), 2)));
    //     V3D p_proj_cur(map_info.project_R*(closest_point.glob -  map_info.project_T));
    //     V3F vec_cur;
    //     vec_cur(2)    = float(p_proj_cur.norm());
    //     vec_cur(0)    = atan2f(float(p_proj_cur(1)), float(p_proj_cur(0)));
    //     vec_cur(1)    = atan2f(float(p_proj_cur(2)), sqrt(pow(float(p_proj_cur(0)), 2) + pow(float(p_proj_cur(1)), 2)));
    //     if(should_print) cout << "last hor angle: " << vec_last(0) << ", cur hor angle: "  << vec_cur(0) << endl;
    //     p.last_closest = closest_point.glob;
    //     // if(abs(vec_last(0) - vec_cur(0)) < hor_resolution_max) return true;
    // }
    // else p.last_closest = closest_point.glob;
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        if(should_print) cout<<"local: "<<p.local(0)<<" , "<<p.local(1)<<endl;
        float depth_static = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
        if (should_print) cout<<"depth interp1: "<<depth_static<<" real:"<<p.vec(2)<<" in "<<map_info.map_index<<endl;// || depth_in_map > p.vec(2) + interp_all_thr
        float cur_interp = interp_thr1;
        if(p.vec(2) > interp_start_depth1)
            cur_interp += ((p.vec(2) - interp_start_depth1)* interp_kp1 + interp_kd1);
        if(should_print) cout<<"  "<<cur_interp<<endl;
        if(dataset == 0 )
        {
            if(fabs(depth_static+1) < 10E-5 || fabs(depth_static+2) < 10E-5)
            {
                // float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
                // if(should_print) cout<<"depth all:" << depth_all << "  cur depth:"<<p.vec(2)<<endl;
                // if(fabs(depth_all+1) < 10E-5)
                // {
                //     return true;
                // }
                // if(fabs(depth_all+20) < 10E-5)
                // {
                //     return false;
                // }
                // if((depth_all > 0 && depth_all < 1000 && (p.vec(2) - depth_all) > 0.5*cur_interp) || \
                //     (depth_all > 1000 && depth_all - 1000 - p.vec(2) < 0.5*cur_interp))//(p.vec(2)-depth_all) > interp_bg
                // {
                //     return true;
                // }
                // else
                // {
                //     return false;
                // }
                return false;
            }
            else 
            {
                if(fabs(depth_static - p.vec(2)) < cur_interp)
                {
                    return true;
                } 
            }
        }
        else
        {
            if(fabs(depth_static+1) < 10E-5 || fabs(depth_static+2) < 10E-5)
            {
                // float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
                // if(should_print) cout<<"depth all:" << depth_all << "  cur depth:"<<p.vec(2)<<endl;
                // if(fabs(depth_all+1) < 10E-5)
                // {
                //     return true;
                // }
                // if(fabs(depth_all+20) < 10E-5)
                // {
                //     return false;
                // }
                // if((depth_all > 0 && depth_all < 1000 && (p.vec(2) - depth_all) > 0.5*cur_interp) || \
                //     (depth_all > 1000 && depth_all - 1000 - p.vec(2) < 0.5*cur_interp))//(p.vec(2)-depth_all) > interp_bg
                // {
                //     return true;
                // }
                // else
                // {
                //     return false;
                // }
                return false;
            }
            else 
            {
                if(fabs(depth_static - p.vec(2)) < cur_interp)
                {
                    return true;
                } 
            }
            
        }
        
    }
    return false;
}

bool  DynObjFilter::Case1DepthConsistencyCheck(const point_soph & p, const DepthMap &map_info)
{
    return true;
    float all_minus = 0;
    int num = 0;
    float cur_depth_max_thr = depth_cons_depth_max_thr1 ;//+ p.vec(2)*0.1
    float cur_depth_thr = depth_cons_depth_thr1 ;//+ p.vec(2)*0.01
    for (int ind_hor = 0; ind_hor < 2*depth_cons_hor_num1 + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*depth_cons_ver_num1 + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;
            int pos_new = p.position + pos_offset[ind];
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;

            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph*  point = points_in_pixel[j]; 
                if(fabs(point->vec(0)-p.vec(0)) < depth_cons_hor_thr1 && \
                  fabs(point->vec(1)-p.vec(1)) < depth_cons_ver_thr1)
                {
                    if(fabs(p.vec(2)-point->vec(2)) < cur_depth_max_thr)
                    {
                        num ++;
                        all_minus += fabs(point->vec(2)-p.vec(2));
                        if(should_print) cout<<" depth cons: "<<(p.vec(2)-point->vec(2));
                    }
                }
            }

        }
    }
    if(num > 1)
    {
        if(should_print) cout<<"depth_cons: "<<all_minus/(num-1)<<" "<<depth_cons_depth_thr1<<endl;
        if(all_minus/(num-1) < cur_depth_thr)
        {
            
            return true;
        }
        // if(should_print) cout<<"depth_cons: "<<all_minus/num<<" "<<depth_cons_depth_thr2<<endl;
    }
    else
    {
        if(should_print) cout<<"depth_cons: no points"<<endl;
    }
    if(should_print) cout<<"depth_cons failed: "<<all_minus/(num-1)<<" "<<depth_cons_depth_thr1<<endl;
    return false;
}

float DynObjFilter::DepthInterpolationStatic(point_soph & p, int map_index, const DepthMap2D &depth_map)
{
    if(map_index - depth_map_list.front()->map_index > dyn_windows_num) 
    {
        cout<<"-------error in last depth interpolations"<<endl;
    }
    if(fabs(p.last_depth_interps.at(map_index - depth_map_list.front()->map_index)) > 10E-4)
    {
        if(should_print) cout<<"same interp"<<endl;
        float depth_cal = p.last_depth_interps.at(map_index - depth_map_list.front()->map_index);
        return depth_cal;
    }
    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    vector<V3F> p_neighbors;
    int all_num = 0, static_num = 0, no_bg_num = 0;
    // cout<<"---------num: "<<interp_hor_num<<" , "<<interp_ver_num<<endl;
    for (int ind_hor = 0; ind_hor < 2*interp_hor_num + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*interp_ver_num + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;
            int pos_new = p.position + pos_offset[ind];           
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points_in_pixel = depth_map[pos_new];
            // cout<<" , "<<points_in_pixel.size();
            if(points_in_pixel.size() > 50)
            {
                cout<<points_in_pixel[0]->dyn<<" , "<<points_in_pixel.size()<<"--------"<<points_in_pixel[0]->glob.transpose()<<" , "<<points_in_pixel[0]->local.transpose()<<endl;
            }
            
            for (int j = 0; j < points_in_pixel.size(); j ++)
            {   
                const point_soph* point = points_in_pixel[j]; 
                if (fabs(point->time - p.time) < frame_dur)
                {
                    continue;
                }
                float hor_minus =  point->vec(0) - p.vec(0);
                float ver_minus =  point->vec(1) - p.vec(1);
                if (fabs(hor_minus) < interp_hor_thr && fabs(ver_minus) < interp_ver_thr)
                {// &&
                    all_num ++;
                    if(point->dyn == STATIC) 
                    {
                        static_num ++;
                    }
                    if((point->vec(2) - p.vec(2)) <= interp_static_max && (p.vec(2) - point->vec(2)) < 5.0)
                    {
                        no_bg_num ++;
                    }// && (point->vec(2) - p.vec(2)) <= interp_static_max && (p.vec(2) - point->vec(2)) < 5.0
                    if(point->dyn == STATIC)
                    {
                        p_neighbors.push_back(point->vec);
                        if (p_1(2)<0.000001 || fabs(hor_minus) + fabs(ver_minus) < fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1)))
                        {
                            p_1 = point->vec;
                        }
                    }
                    
                    if(should_print)   
                    {
                        cout<<"depth interp points: "<<fabs(p.vec(0)-point->vec(0))<<" "<<fabs(p.vec(1)-point->vec(1))\
                        <<" depth minus: "<<(p.vec(2)-point->vec(2))<<" "<<\
                        " "<<point->glob.transpose()<<endl;
                    }
                }
            }
        }
    }
    if(should_print) cout<<"--------all num: "<<all_num<< " " << p_neighbors.size() <<endl;
    // cout<<"--------all num: "<<all_num<<" , "<<static_num<<" , "<<no_bg_num<<" , "<<p_neighbors.size()<<endl;
    if (p_1(2)<10E-5)
    {
        if(should_print) cout<<"all num: "<<all_num<<" static num: "<<static_num<<" , "<<p_neighbors.size()<<endl;
        // if(no_bg_num < all_num/4)
        // {
        //     p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = p.vec(2) + interp_static_max;
        //     return p.vec(2) + interp_static_max;
        // }
        // else 
        {
            // if(static_num < all_num/5)
            // {
            //     p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = p.vec(2) + 10;
            //     return p.vec(2) + 10;
            // }

            p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -1;
            return -1;
        }
    }

    int cur_size = p_neighbors.size();
    for(int t_i = 0; t_i < cur_size-2; t_i++)
    {
        // cout<<"size: "<<cur_size<<" "<<cur_size-2<<" "<<t_i<<endl;
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2*(interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha  = 0, beta = 0;
        for(int i = t_i+1; i < cur_size-1; i++)
        {
            
            if(fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                // float l2_a = p_2.vec(1) -  p.vec(1);
                // float l2_b = -p_2.vec(0) +  p.vec(0);
                // float l2_c = p_2.vec(0)*p.vec(1) -  p.vec(0)*p_2.vec(1);
                float single_fabs = fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1));
                if (single_fabs >= min_fabs) continue;
                for(int ii = i+1; ii < cur_size; ii++)
                {
                    // if(p_neighbors[ii].dyn == STATIC)
                    {
                        float cur_fabs = fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1)) + \
                                        fabs(p_neighbors[ii](0)-p.vec(0)) + fabs(p_neighbors[ii](1)-p.vec(1));
                        if( cur_fabs < min_fabs)
                        {
                            // float l1_out = l1_a*p_neighbors[ii].vec(0) + l1_b*p_neighbors[ii].vec(1) + l1_c;
                            // float l2_out = l2_a*p_neighbors[ii].vec(0) + l2_b*p_neighbors[ii].vec(1) + l2_c;
                            // if(l1_out > 0 && l2_out > 0)
                            float x1 = p_neighbors[i](0) - p_1(0);
                            float x2 = p_neighbors[ii](0) - p_1(0);
                            float y1 = p_neighbors[i](1) - p_1(1);
                            float y2 = p_neighbors[ii](1) - p_1(1);
                            float lower = x1*y2-x2*y1;
                            if(fabs(lower) > 10E-5)
                            {
                                alpha = (x*y2-y*x2)/lower;
                                beta = -(x*y1-y*x1)/lower;
                                // if(should_print) 
                                // {
                                //     cout<<" alpha: "<<alpha<<" beta: "<<beta<<endl;
                                // }
                                if(alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                                {
                                    // if(p_3.vec(2)<0.000001)
                                    {
                                        p_3 = p_neighbors[ii];
                                        min_fabs = cur_fabs; 
                                    }
                                }
                                
                            }
                        }
                    }
                    
                    
                }
                
            }
            
        }
        if (p_2(2)<10E-5 || p_3(2)<10E-5)
        {
            // cout<<"no enough points"<<endl;
            // p.last_depth_interps.emplace(map_index, -2);
            // cout<<"------------2 "<<map_index<<", "<<depth_map_list.front()->map_index<<", "<<depth_map_list.size()<<endl;
            // if(p_neighbors.size() < 4)
            // {
            //     if(should_print) cout<<p_neighbors.size()<<endl;
            //     p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -p_neighbors.size();
            //     // cout<<"input "<<p.last_depth_interps[map_index - depth_map_list.front()->map_index]<<endl;
            //     return -p_neighbors.size();
            // }
            continue;
        }
        // Eigen::Matrix3d A;
        // A <<         1,      1,      1,
        //         p_1(0), p_2(0), p_3(0),
        //         p_1(1), p_2(1), p_3(1);
        // Eigen::Vector3d b;
        // b << 1, p.vec(0), p.vec(1);
        // Eigen::Vector3d sol = A.lu().solve(b); 
        // Eigen::Vector3d depth;
        // Eigen::Vector3d sol;
        // sol<< 1-alpha-beta, alpha, beta;
        // depth << p_1(2), p_2(2), p_3(2);
        if(isnan(p_1(0)) || isnan(p_1(1)) || isnan(p_1(2)) || isnan(p_2(0)) || isnan(p_2(1)) || isnan(p_2(2)) \
            || isnan(p_3(0)) || isnan(p_3(1)) || isnan(p_3(2)))
        {
            cout<<"-------error---------"<<p_1.transpose()<<"  ,  "<<p_2.transpose()<<"  ,  "<<p_3.transpose()<<endl;
        }
        // float depth_cal = sol.transpose()*depth;
        float depth_cal = (1-alpha-beta)*p_1(2) + alpha*p_2(2) + beta*p_3(2);
        
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = depth_cal;
        if(should_print) 
        {
            cout<<"p: "<<p.vec.transpose()<<" p1: "<<p_1.transpose()<<" p2: "<<p_2.transpose()<<" p3: "\
            <<p_3.transpose()<<endl;
        }
        return depth_cal;
    }
    if(should_print) cout<<"no interpolation "<<p_neighbors.size()<<" "<<all_num<<" static num: "<<static_num<<", "<<no_bg_num<<endl;
    if(static_num > 0 && cur_size < all_num/2)
    {
        // if(no_bg_num < static_num)
        // {
        //     p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = p.vec(2) + interp_static_max;
        //     return p.vec(2) + interp_static_max;
        // }
        // else
        {
            p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -2;
            return -2;
        }
    }
    else
    {
        // if(static_num < cur_size/5)
        // {
        //     p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = p.vec(2) + 10;
        //     return p.vec(2) + 10;
        // }
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -2;
        return -2;
    }
    // float gauss = GaussionInterpolation(p, p_neighbors);
    // p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = gauss;
    // if(should_print)
    // {
    //     cout<<"-----gauss: "<<p.glob.transpose()<<" ,, "<<gauss<<" "<<p.vec(2)<<" , ";
    //     for(int i=0; i < cur_size; i++)
    //     {
    //         cout<<p_neighbors[i](2)<<" , ";
    //     }
    //     cout<<endl;
    // }
    
    // return gauss;
}// return -1 denotes no point, -2 denotes no trianguolar but with points

float DynObjFilter::GaussionInterpolation(point_soph & p, vector<V3F>& p_neighbors)
{
    int size = p_neighbors.size();
    if(size < 3) return -1;
    float minus_left = 1, minus_right = -1, minus_up = 1, minus_down = -1;
    int id_left = 1, id_right = -1, id_up = 1, id_down = -1;
    for(int j = 0; j < size; j++)
    {
        if(fabs(p_neighbors[j](1)-p.vec(1)) < interp_ver_thr/2 && p_neighbors[j](0)-p.vec(0) > 0 \
            && p_neighbors[j](0)-p.vec(0) < minus_left)
        {
            minus_left = p_neighbors[j](0)-p.vec(0);
            id_left = j;
        }
        if(fabs(p_neighbors[j](1)-p.vec(1)) < interp_ver_thr/2 && p_neighbors[j](0)-p.vec(0) < 0 \
            && p_neighbors[j](0)-p.vec(0) > minus_right)
        {
            minus_right = p_neighbors[j](0)-p.vec(0);
            id_right = j;
        }
        if(fabs(p_neighbors[j](0)-p.vec(0)) < interp_hor_thr/2 && p_neighbors[j](1)-p.vec(1) > 0 \
            && p_neighbors[j](1)-p.vec(1) < minus_up)
        {
            minus_up = p_neighbors[j](1)-p.vec(1);
            id_up = j;
        }
        if(fabs(p_neighbors[j](0)-p.vec(0)) < interp_hor_thr/2 && p_neighbors[j](1)-p.vec(1) < 0 \
            && p_neighbors[j](1)-p.vec(1) > minus_down)
        {
            minus_down = p_neighbors[j](1)-p.vec(1);
            id_down = j;
        }
    }
    if (minus_left == 1  || minus_right == -1)
    {
        if(minus_up == 1 || minus_down == -1)
        {
            return -20;
        }
        else
        {
            return (p_neighbors[minus_up](2) + p_neighbors[minus_down](2))/2 + 1000;
        }
    } 
    else
    {
        if(minus_up == 1 || minus_down == -1)
        {
            return (p_neighbors[minus_left](2) + p_neighbors[minus_right](2))/2 + 1000;
        }
        else
        {
            return (p_neighbors[minus_up](2) + p_neighbors[minus_down](2) + \
                    p_neighbors[minus_left](2) + p_neighbors[minus_right](2))/4 + 1000;
        }
    }
    // for(int i = 0; i < 2; i++)
    // {
    //     float min_minus = fabs(p_neighbors[i].(0)-p.vec(0)) + fabs(p_neighbors[i].(1)-p.vec(1));
    //     int min_id = i;
    //     for(int j = i; j < size; j++)
    //     {
    //         if(fabs(p_neighbors[j].(0)-p.vec(0)) + fabs(p_neighbors[j].(1)-p.vec(1)) < min_minus)
    //         {
    //             min_minus = fabs(p_neighbors[j].(0)-p.vec(0)) + fabs(p_neighbors[j].(1)-p.vec(1));
    //             min_id = j;
    //         }
    //     }
    //     if(i != min_id) swap(p_neighbors[i], p_neighbors[min_id]);
    //     minus.push_back(min_minus);
    //     depth.push_back(p_neighbors[i](2));
    // }

} //-1 denotes no points, -20 denotes lots of points but no triangular, >1000 denotes success 

bool  DynObjFilter::Case2(point_soph & p)
{   
    if(dataset == 0 && p.is_distort) return false;
    int first_i = depth_map_list.size();
    first_i -= 1;
    if(first_i < 0) return false;
    point_soph p_spherical = p;
    
    SphericalProjection(p, depth_map_list[first_i]->map_index, depth_map_list[first_i]->project_R, depth_map_list[first_i]->project_T, p_spherical);

    
    
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f || \
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        cout<<"~~~~INVALID POINTS!!!!~~~~~ hor_ind: "<<p_spherical.hor_ind<<" ver_ind: "<<p_spherical.ver_ind<<endl;
        return false;
    }
    if(should_print) 
        cout<<"check case 2"<<endl;
    int cur_occ_times = 0;
    if (Case2Enter(p_spherical, *depth_map_list[first_i]))
    {
        if(should_print) 
            cout<<"in case 2"<<endl;
        if (!Case2MapConsistencyCheck(p_spherical, *depth_map_list[first_i], case2_interp_en))
        {
            // if(should_print) 
            //     cout<<"in map consistency "<<2*occ_hor_num2 + 1<<" "<<2*occ_ver_num2 + 1<<" "<<p.glob.transpose()<<endl;
            // vector<point_soph> p_occs;     
            // int cur_occ_times = 0;
            double ti = 0;
            float vi = 0; 
            float min_hor = occ_hor_thr2, min_ver = occ_ver_thr2;
            bool map_cons = true;
            for (int ind_roll = 0; ind_roll < 2*occ_hor_num2 + 1 && map_cons; ind_roll ++)
            {
                for (int ind_pitch = 0; ind_pitch < 2*occ_ver_num2 + 1 && map_cons; ind_pitch ++)
                {
                    // neighbor_in ++;
                    int ind = ind_roll*(2*ver_num + 1) + ind_pitch;               
                    int pos_new = p_spherical.position + pos_offset[ind];           
                    if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
                    const vector<point_soph*> & points_in_pixel = depth_map_list[first_i]->depth_map[pos_new]; 
                    
                    if (depth_map_list[first_i]->min_depth_all[pos_new] > p_spherical.vec(2))
                    {
                        if(should_print) cout<<"continue"<<endl;
                        continue;
                    }   
                    // if(abs(pos_new / MAX_1D_HALF - 628) < 10) neighbor_num ++;
                    // neighbor_num += points_in_pixel.size();
                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const point_soph*  p_occ = points_in_pixel[k];
                            
                        if(Case2IsOccluded(p_spherical, *p_occ) && Case2DepthConsistencyCheck(*p_occ, *depth_map_list[first_i]))
                            //&& Case2ReCheck(*p_occ, depth_map_list[first_i]->depth_map)
                        {
                            if(should_print) 
                            {
                                cout<<"first occlude"<<p_occ->glob.transpose()<<" angle: "<<\
                                fabs(p_spherical.vec(0)-p_occ->vec(0))<<" "<<fabs(p_spherical.vec(1)-p_occ->vec(1))\
                                <<" "<<(p_spherical.vec(2)-p_occ->vec(2))<<endl;
                            }
                                
                            // if(DepthConsistencyCheck(p_occ, depth_map_list[first_i]))
                            {
                                cur_occ_times = 1;
                                if(cur_occ_times >= occluded_times_thr2) break;
                                ti = (p_occ->time + p.time)/2;
                                vi = (p_spherical.vec(2) - p_occ->vec(2))/(p.time - p_occ->time);
                                if(should_print) 
                                {
                                    cout<<"vi: "<<vi<<"  "<<ti<<" "<<p.glob.transpose()<<" , "<<p_occ->glob.transpose()<<" time: "\
                                    <<p.time<<" "<<p_occ->time<<" depth:"<<p_spherical.vec(2)<<" "<<p_occ->vec(2)<<endl;
                                }
                                // if(p.occu_index[0] != depth_map_list[first_i]->map_index)
                                {
                                    p.occu_index[0] = depth_map_list[first_i]->map_index;
                                    p.occu_index[1] = pos_new;
                                    p.occu_index[2] = k;
                                    p.occ_vec = p_spherical.vec;
                                    p.occu_times = cur_occ_times;
                                }
                                
                                point_soph  p0 = p;
                                // point_soph* p1 = points_in_pixel[k];
                                point_soph p1 = *points_in_pixel[k];
                                
                                int i = depth_map_list.size();
                                i = i - 2;
                                V3D t1, t2;
                                t1.setZero();
                                t2.setZero();
                                while(i >= 0)
                                {
                                    // if(p1->occu_index[0] != -1 && p1->occu_index[0] < depth_map_list.front()->map_index) cout<<"------------error-----------"<<endl;
                                    // if(t2.norm() > 10E-3 && (t2-p1->glob).norm() > 10E-3) cout<<"!!!!!!!!!!!!!!!!!error!!!!!!!!!!!!!!!"<<endl;
                                    
                                    if(p1.occu_index[0] == -1 || p1.occu_index[0] < depth_map_list.front()->map_index)
                                    {
                                        // point_soph p1_spherical = *p1;
                                        SphericalProjection(p1, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p1);
                                        // double cur_t = ros::Time::now().toSec();
                                        if(Case2SearchPointOccludingP(p1, *depth_map_list[i]))
                                        {
                                            // p1->occu_index = p1_spherical.occu_index; 
                                            // p1->occ_vec = p1_spherical.vec;
                                            // time_search += (ros::Time::now().toSec() - cur_t);
                                            // times_search ++;
                                            p1.occ_vec = p1.vec;
                                        }
                                        else
                                        {
                                            // times_search ++;
                                            // time_search += (ros::Time::now().toSec() - cur_t);
                                            // p1->occu_index[0] = -2;
                                            if(should_print) 
                                                cout<<"no p2 "<<i<<endl;
                                            break;
                                        }
                                        
                                    }
                                    
                                    i = p1.occu_index[0]-depth_map_list.front()->map_index;
                                    point_soph*  p2 = depth_map_list[i]->depth_map[p1.occu_index[1]][p1.occu_index[2]];                        
                                    
                                    SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);
                                    if(Case2MapConsistencyCheck(p, *depth_map_list[i], case2_interp_en))//false
                                    {
                                        map_cons = false;
                                        break;
                                    }
                                    // if(should_print) 
                                    //     cout<<"------get p2 "<<p2->glob.transpose()<<endl;
                                    float vc = (p1.occ_vec(2) - p2->vec(2))/(p1.time - p2->time);
                                    double tc = (p2->time + p1.time)/2;
                                    if(should_print) 
                                    {
                                        cout<<"vc: "<<vc<<" "<<tc<<" "<<p2->glob.transpose()<<" time: "<<p1.time<<" "<<p2->time\
                                        <<" depth:"<<p1.occ_vec(2)<<" "<<p2->vec(2)<<" "<<p1.vec(2)<<endl;
                                    }
                                    // point_soph pi_spherical = p; && Case2ReCheck(*p2, depth_map_list[i]->depth_map)
                                    // if(should_print) 
                                    //         cout<<"get p2 then"<<p.glob.transpose()<<endl;
                                    
                                    if (Case2IsOccluded(p, *p2) &&\
                                        Case2DepthConsistencyCheck(*p2, *depth_map_list[i]) && Case2VelCheck(vi, vc, ti-tc) )
                                    {
                                        
                                        cur_occ_times += 1;
                                        if(should_print) 
                                        {
                                            cout<<"continuous occlude "<<cur_occ_times<<"   "<<p2->glob.transpose()\
                                            <<" angle: "<<fabs(p.vec(0)-p2->vec(0))<<" "\
                                            <<fabs(p.vec(1)-p2->vec(1))<<" "<<\
                                            (p.vec(2)-p2->vec(2))<<endl;
                                        }
                                        if(cur_occ_times >= occluded_times_thr2)
                                        {
                                            // p.occu_index[0] = depth_map_list[first_i]->map_index;
                                            // p.occu_index[1] = pos_new;
                                            // p.occu_index[2] = k;
                                            // p.occ_vec = p_spherical.vec;
                                            p.occu_times = cur_occ_times;
                                            return true;
                                        }
                                        // p0 = p1;
                                        t2 = p2->glob;
                                        p1 = *p2;
                                        vi = vc;
                                        ti = tc;
                                    }
                                    else
                                    {
                                        break;
                                    }
                                    i--;
                                }
                            }  
                        } 

                        if(cur_occ_times >= occluded_times_thr2) break;
                    }
                    if(cur_occ_times >= occluded_times_thr2) break;
                }
                if(cur_occ_times >= occluded_times_thr2) break;
            }
        }
        
    }
    if (cur_occ_times >= occluded_times_thr2) 
    {
        p.occu_times = cur_occ_times;
        return true;
    }
    return false;
}

bool  DynObjFilter::Case2Enter(point_soph & p, const DepthMap &map_info)
{
    if(p.dyn != STATIC)
    {
        if(should_print) cout<<"case2 enter failed: "<<p.dyn<<endl;
        return false;
    }
    float max_depth = 0;
    float depth_thr2_final = occ_depth_thr2;
    if(map_info.depth_map[p.position].size() > 0)
    {
        const point_soph* max_point = map_info.depth_map[p.position][map_info.max_depth_index_all[p.position]];
        max_depth = max_point->vec(2); //make sure the max index is ok for this max depth
        float delta_t = (p.time - max_point->time);
        depth_thr2_final = min(occ_depth_thr2, v_min_thr2*delta_t);
        if(should_print) 
            cout<<"max point: "<<max_point->glob.transpose()<<" "<<max_point->vec.transpose()<<endl;
    }
    //                TBD
    // else
    // {
    //     max_depth = DepthInterpolationAll();
    // }
    if(p.vec(2) > max_depth + depth_thr2_final) //TD     no points, enter directly
    {
        case2_num ++;
        return true;
    }
    else
    {
        if(should_print) cout<<"case2 enter failed: "<<p.vec(2)<<" , "<<max_depth<<" , "<<depth_thr2_final<<endl;
        return false;
    }
}

bool  DynObjFilter::Case2MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp)
{
    // return false;
    float cur_hor = map_cons_hor_thr2;
    float cur_ver = map_cons_ver_thr2;
    // for (int ind_hor = 0; ind_hor < 2*map_cons_hor_num2 + 1; ind_hor ++)
    // { 
    //     for (int ind_ver = 0; ind_ver < 2*map_cons_ver_num2 + 1; ind_ver ++)
    //     {
    for (int ind_hor = -map_cons_hor_num2; ind_hor <= map_cons_hor_num2; ind_hor ++)
    {
        for (int ind_ver = -map_cons_ver_num2; ind_ver <= map_cons_ver_num2; ind_ver ++)  
        {
            // int ind = ind_hor*(2*ver_num + 1) + ind_ver;
            // int pos_new = p.position + pos_offset[ind];
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];                        
  
            if (map_info.max_depth_all[pos_new] > p.vec(2) + map_cons_depth_thr2 && \
                map_info.min_depth_all[pos_new] < p.vec(2) - map_cons_depth_thr2)
            {
                continue;
            }   

            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j];
                if (point->dyn == STATIC && \
                    fabs(p.time-point->time) > frame_dur && \
                    fabs(p.vec(2)-point->vec(2)) <  map_cons_depth_thr2 && \
                    fabs(p.vec(0)-point->vec(0)) < map_cons_hor_thr2 && \
                    fabs(p.vec(1)-point->vec(1)) < map_cons_ver_thr2 
                  )
                {
                    if(should_print) cout<<"case2 map cons: "<<point->glob.transpose()<<" , "<<p.vec(2)<<" "<<point->vec(2)<<" "<< \
                    fabs(p.vec(0)-point->vec(0))<<" "<< map_cons_hor_thr2 <<" "<< \
                    fabs(p.vec(1)-point->vec(1)) <<" "<< map_cons_ver_thr2 <<endl;
                    return true;
                }               
            }         
        }
        
    }
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r) )
    {
        // float depth_static = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
        // if (should_print) cout<<"depth interp2: "<<depth_static<<" real:"<<p.vec(2)<<" in "<<map_info.map_index<<endl;// || depth_in_map > p.vec(2) + interp_all_thr
        float cur_interp = interp_thr2*(depth_map_list.back()->map_index - map_info.map_index + 1);
        // if(p.vec(2) > interp_start_depth2)
        //     cur_interp += ((p.vec(2) - interp_start_depth2)* interp_kp2 + interp_kd2);
        // if(should_print) cout<<"  "<<cur_interp<<endl;
        // if(depth_static < 10E-5)
        {
            float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
            if(should_print) cout<<"depth all:" << depth_all << "  cur depth:"<<p.vec(2)<<endl;
            if( fabs(p.vec(2) - depth_all)  < cur_interp) //depth_all < -11 ||
            {
                return true;
            }
            else
            {
                return false;
            }
            
        }
        // else if(depth_static < -11 || p.vec(2) - depth_static  < cur_interp)
        // {
        //     return true;
        // }

        // float depth_in_map = DepthInterpolationAll(p, map_info.depth_map);
        // float depth_in_map1 = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
        // float cur_interp = interp_thr2;
        // // if(p.vec(2) > interp_start_depth2)
        // //     cur_interp += ((p.vec(2) - interp_start_depth2)* interp_kp2 + interp_kd2);
        // if(should_print) cout<<"  "<<cur_interp<<endl;
        // if(should_print) 
        // {
        //     cout<<"depth interp2: "<<depth_in_map<<" , "<<depth_in_map1<<\
        //     " real: "<<p.vec(2)<<" in "<<map_info.map_index<<endl;
        // }
            
        // if(depth_in_map < -11 || p.vec(2) - depth_in_map  < cur_interp) // 
        // {
        //     if(should_print) cout<<"depth interp2 failed "<<p.vec(2)<<" "<<depth_in_map<<" "<<p.glob.transpose()<<endl;
        //     return true;
        // }
    }
    return false;
}

bool  DynObjFilter::Case2SearchPointOccludingP(point_soph & p, const DepthMap &map_info)
{
    // return false;
    // int i = 0;  
    for (int ind_hor = 0; ind_hor < 2*occ_hor_num2 + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*occ_ver_num2 + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;               
            int pos_new = p.position + pos_offset[ind];           
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];            
            if (map_info.min_depth_all[pos_new] > p.vec(2))
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                // i++;
                const point_soph* p_cond = points_in_pixel[j];
                // if(should_print && fabs(p_cond->time - p.time) > frame_dur) cout<<"-------error"<<endl;
                if (Case2IsOccluded(p, *p_cond) && Case2DepthConsistencyCheck(*p_cond, map_info)) 
                    // Case2ReCheck(*p_cond, map_info.depth_map)&& 
                {
                    p.occu_index[0] = map_info.map_index;
                    p.occu_index[1] = pos_new;
                    p.occu_index[2] = j;
                    p.occ_vec = p.vec;
                    if(should_print) 
                    {
                        cout<<"------find new p2 "<<\
                        depth_map_list[p.occu_index[0]-depth_map_list.front()->map_index]->depth_map[p.occu_index[1]][p.occu_index[2]]->glob.transpose()<<\
                        " "<<p_cond->local.transpose()<<endl;
                        cout<<" depth: "<<p.vec(2)<<" "<<p_cond->vec(2)<<endl;
                        // V3D p_proj(map_info.project_R.transpose()*p.glob - map_info.project_R.transpose()*map_info.project_T);
                        // point_soph p_spherical(p_proj, hor_resolution_max, ver_resolution_max);
                    }
                    // cout<<"success i: "<< i <<endl;
                    return true;
                }
            }        
        }
    }
    // cout<<"failed i: "<< i <<endl;
    if(should_print) cout<<"------no points "<<p.glob<<endl;
    return false;
}

bool  DynObjFilter::Case2IsOccluded(const point_soph & p, const point_soph & p_occ)
{
    if((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID) return false;
    if((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) || \
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }
    float delta_t = p.time - p_occ.time;
    float cur_occ_hor = occ_hor_thr2; //min(, 0.1/p.vec(2));
    float cur_occ_ver = occ_ver_thr2; //min(, 0.1/p.vec(2));
    if(delta_t > 0)
    {
        float depth_thr2_final = min(occ_depth_thr2, v_min_thr2*delta_t);
        
        if (p.vec(2) >  p_occ.vec(2) + depth_thr2_final && \
            fabs(p.vec(0)-p_occ.vec(0)) < cur_occ_hor && \
            fabs(p.vec(1)-p_occ.vec(1)) < cur_occ_ver )
        {
            // if(should_print) cout<<"v_min: "<<v_min_thr2<<" t "<<delta_t<<" "<<depth_thr2_final<<endl;
            return true;
        } 
        else
        {
            if(should_print) cout<<"thr2: "<<depth_thr2_final<<" "<<fabs(p.vec(0)-p_occ.vec(0))<<" "<<\
            fabs(p.vec(1)-p_occ.vec(1))<<" "<<p.vec(2)<<" "<<p_occ.vec(2)<<"  "<<p.glob.transpose()<<",,,,"<<\
            p_occ.glob.transpose()<<endl;
        }              
    }
    
    return false;
}

float DynObjFilter::DepthInterpolationAll(point_soph & p, int map_index, const DepthMap2D &depth_map)
{
    // unordered_map<int, float>::iterator it;
    // it =  p.last_depth_interps.find(map_index);
    // if(it != p.last_depth_interps.end())
    // {
    //     cout<<"same index"<<endl;
    //     return it->second;
    // }
    // cout<<"------------ "<<map_index<<", "<<depth_map_list.front()->map_index<<", "<<depth_map_list.size()<<endl;
    // if(map_index - depth_map_list.front()->map_index > dyn_windows_num) 
    // {
    //     cout<<"-------error in last depth interpolations"<<endl;
    // }
    // if(fabs(p.last_depth_interps.at(map_index - depth_map_list.front()->map_index)+1) > 10E-4)
    // {
    //     // cout<<"last value: "<<p.last_depth_interps[map_index - depth_map_list.front()->map_index]<<endl;
    //     float depth_cal = p.last_depth_interps.at(map_index - depth_map_list.front()->map_index);
    //     return depth_cal;
    // }
    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    vector<V3F> p_neighbors;
    int all_num = 0;
    for (int ind_hor = 0; ind_hor < 2*interp_hor_num + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*interp_ver_num + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;
            int pos_new = p.position + pos_offset[ind];           
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points_in_pixel = depth_map[pos_new];
            
            for (int j = 0; j < points_in_pixel.size(); j ++)
            {
                const point_soph*  point = points_in_pixel[j]; 
                
                if (fabs(point->time - p.time) < frame_dur)// ||\(point->vec(2) - p.vec(2)) > interp_all_max)
                {
                    continue;
                }
                float hor_minus =  point->vec(0) - p.vec(0);
                float ver_minus =  point->vec(1) - p.vec(1);
                if (fabs(hor_minus) < interp_hor_thr && fabs(ver_minus) < interp_ver_thr)
                {
                    all_num ++;
                    p_neighbors.push_back(point->vec);
                    if(isnan(point->vec(0)) || isnan(point->vec(1)) || isnan(point->vec(2)))
                    {
                        cout<<"-------error---------"<<point->vec.transpose()<<"  ,  "<<point->glob.transpose()<<endl;
                    }
                    if (p_1(2)<0.000001 || fabs(hor_minus) + fabs(ver_minus) < fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1)))
                    {
                        p_1 = point->vec;
                    }
                    
                }
            }
        }
    }
    // if(should_print) cout<<endl;
    int cur_size = p_neighbors.size();
    if (p_1(2)<10E-5 || cur_size < 3)
    {
        // cout<<"no points"<<endl;
        // p.last_depth_interps.emplace(map_index, -1);
        // cout<<"------------1 "<<map_index<<", "<<depth_map_list.front()->map_index<<", "<<depth_map_list.size()<<endl;
        if(should_print) cout<<"all num: "<<all_num<<" , "<<p_neighbors.size()<<endl;
        // p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -1.0;
        return -1;
    }

    
    for(int t_i = 0; t_i < cur_size-2; t_i++)
    {
        // cout<<"size: "<<cur_size<<" "<<cur_size-2<<" "<<t_i<<endl;
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2*(interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha  = 0, beta = 0;
        for(int i = t_i+1; i < cur_size-1; i++)
        {
            
            if(fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                float single_fabs = fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1));
                if (single_fabs >= min_fabs) continue;
                for(int ii = i+1; ii < cur_size; ii++)
                {
                    {
                        float cur_fabs = fabs(p_neighbors[i](0)-p.vec(0)) + fabs(p_neighbors[i](1)-p.vec(1)) + \
                                        fabs(p_neighbors[ii](0)-p.vec(0)) + fabs(p_neighbors[ii](1)-p.vec(1));
                        if( cur_fabs < min_fabs)
                        {
                            // float l1_out = l1_a*p_neighbors[ii].vec(0) + l1_b*p_neighbors[ii].vec(1) + l1_c;
                            // float l2_out = l2_a*p_neighbors[ii].vec(0) + l2_b*p_neighbors[ii].vec(1) + l2_c;
                            // if(l1_out > 0 && l2_out > 0)
                            float x1 = p_neighbors[i](0) - p_1(0);
                            float x2 = p_neighbors[ii](0) - p_1(0);
                            float y1 = p_neighbors[i](1) - p_1(1);
                            float y2 = p_neighbors[ii](1) - p_1(1);
                            float lower = x1*y2-x2*y1;
                            if(fabs(lower) > 10E-5)
                            {
                                alpha = (x*y2-y*x2)/lower;
                                beta = -(x*y1-y*x1)/lower;
                                // if(should_print) 
                                // {
                                //     cout<<" alpha: "<<alpha<<" beta: "<<beta<<endl;
                                // }
                                if(alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                                {
                                    // if(p_3.vec(2)<0.000001)
                                    {
                                        p_3 = p_neighbors[ii];
                                        min_fabs = cur_fabs; 
                                    }
                                }
                                
                            }
                        }
                    }
                    
                    
                }
                
            }
            
        }
        if (p_2(2)<10E-5 || p_3(2)<10E-5)
        {
            // cout<<"no enough points"<<endl;
            // p.last_depth_interps.emplace(map_index, -2);
            // cout<<"------------2 "<<map_index<<", "<<depth_map_list.front()->map_index<<", "<<depth_map_list.size()<<endl;
            // if(p_neighbors.size() < 4)
            // {
            //     if(should_print) cout<<p_neighbors.size()<<endl;
            //     p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -p_neighbors.size();
            //     // cout<<"input "<<p.last_depth_interps[map_index - depth_map_list.front()->map_index]<<endl;
            //     return -p_neighbors.size();
            // }
            continue;
        }
        // Eigen::Matrix3d A;
        // A <<         1,      1,      1,
        //         p_1(0), p_2(0), p_3(0),
        //         p_1(1), p_2(1), p_3(1);
        // Eigen::Vector3d b;
        // b << 1, p.vec(0), p.vec(1);
        // Eigen::Vector3d sol = A.lu().solve(b); 
        // Eigen::Vector3d depth;
        // Eigen::Vector3d sol;
        // sol<< 1-alpha-beta, alpha, beta;
        // depth << p_1(2), p_2(2), p_3(2);
        if(isnan(p_1(0)) || isnan(p_1(1)) || isnan(p_1(2)) || isnan(p_2(0)) || isnan(p_2(1)) || isnan(p_2(2)) \
            || isnan(p_3(0)) || isnan(p_3(1)) || isnan(p_3(2)))
        {
            cout<<"-------error---------"<<p_1.transpose()<<"  ,  "<<p_2.transpose()<<"  ,  "<<p_3.transpose()<<endl;
        }
        // float depth_cal = sol.transpose()*depth;
        float depth_cal = (1-alpha-beta)*p_1(2) + alpha*p_2(2) + beta*p_3(2);
        
        // p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = depth_cal;
        if(should_print) 
        {
            cout<<"p: "<<p.vec.transpose()<<" d: "<<depth_cal<<endl;
        }
        return depth_cal;
    }
    if(should_print) cout<<"no interpolation "<<p_neighbors.size()<<" "<<all_num<<endl;
    // p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -2.0;
    return -2;
    // float gauss = GaussionInterpolation(p, p_neighbors);
    // p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = gauss;
    // if(should_print)
    // {
    //     cout<<"-----gauss: "<<p.glob.transpose()<<" ,, "<<gauss<<" "<<p.vec(2)<<" , ";
    //     for(int i=0; i < cur_size; i++)
    //     {
    //         cout<< p_neighbors[i](0) - p.vec(0) << " "<< p_neighbors[i](1) - p.vec(1) <<" " << p_neighbors[i](2)<<" , ";
    //     }
    //     cout<<endl;
    // }
    // return gauss;
    // p.last_depth_interps.at(map_index - depth_map_list.front().map_index) = -20;
    // return -20;
} // -1 denotes no points, -2 denotes no triangular > 1000 denotes gauss interpolation 

bool  DynObjFilter::Case2DepthConsistencyCheck(const point_soph & p, const DepthMap &map_info)
{
    // return true;
    float all_minus = 0;
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;//
    if(should_print) cout<<" depth cons ";
    for (int ind_hor = 0; ind_hor < 2*depth_cons_hor_num2 + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*depth_cons_ver_num2 + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;
            int pos_new = p.position + pos_offset[ind];
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;

            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];
            for (int j = 0; j < points_in_pixel.size(); j ++)
            {
                const point_soph* point = points_in_pixel[j]; 
                if(fabs(point->time - p.time) < frame_dur && fabs(point->vec(0)-p.vec(0)) < depth_cons_hor_thr2 && \
                  fabs(point->vec(1)-p.vec(1)) < depth_cons_ver_thr2)
                {
                    // if(should_print) cout<<" , "<<(p.vec(2)-point->vec(2))<<" "<<(p.vec(1)-point->vec(1));
                    all_num ++;
                    if (point->dyn == STATIC) 
                    {
                        float cur_minus = p.vec(2)-point->vec(2);
                        if (fabs(cur_minus) < depth_cons_depth_max_thr2)
                        {
                            num ++;
                            all_minus += fabs(point->vec(2)-p.vec(2));
                        }
                        else if (cur_minus > 0)
                        {
                            smaller_num ++;
                        }
                        else
                        {
                            greater_num ++;
                        }
                    }
                }
            }

        }
    }
    if(should_print) cout<<" cur: "<<p.vec(2)<<endl;
    if(all_num > 0)
    {
        if(num > 1)
        {
            
            float cur_depth_thr = max(depth_cons_depth_thr2, k_depth2*p.vec(2));
            if(should_print) 
                cout<<"depth_cons: "<<all_minus/(num-1)<<" "<<cur_depth_thr <<" num: "<<greater_num<<\
                "  "<<smaller_num<<endl;
            if(all_minus/(num-1) > cur_depth_thr)
            {
                if(should_print) cout<<" depth false"<<endl;
                return false;
            }      
            else
            {
                // int all_num = greater_num + smaller_num;
                // if(all_num > depth_cons_min_num_thr2 && greater_num >= (all_num/2 - 1) && greater_num <= (all_num/2 +1))
                // {
                //     if(should_print) cout<<" num false"<<endl;
                //     return false;
                // }
                // if(all_num > depth_cons_min_num_thr2 && smaller_num > all_num/3)
                // {
                //     if(should_print) cout<<" num false: "<<all_num<<" "<<smaller_num<<endl;
                //     return false;
                // }
                
            }
            // if(should_print) cout<<"depth_cons: "<<all_minus/num<<" "<<depth_cons_depth_thr2<<endl;
        }
        if(greater_num == 0 || smaller_num == 0)
        {
            return true;
        }
        else
        {
            if(should_print) cout<<"depth_cons: num error " << greater_num << " " << smaller_num <<endl;
            return false;
        }
    }
    else
    {
        if(should_print) cout<<"depth_cons: no points"<<endl;
        return false;
    }
    
    
}

bool  DynObjFilter::Case2VelCheck(float v1, float v2, double delta_t)
{
    
    if(fabs(v1 - v2) < delta_t*acc_thr2)
    {
        return true;
    }
    if(should_print) cout<<"velcheck: "<< fabs(v1 - v2)<<" baseline: "<<delta_t*acc_thr2<<" "<<delta_t<<endl;
    return false;
}

bool  DynObjFilter::Case3VelCheck(float v1, float v2, double delta_t)
{

    if(fabs(v1 - v2) < delta_t*acc_thr3)
    {
        return true;
    }
    if(should_print) cout<<"velcheck: "<< fabs(v1 - v2)<<" baseline: "<<delta_t*acc_thr3<<" "<<delta_t<<endl;
    return false;
}

bool  DynObjFilter::Case3(point_soph & p)
{   
    if(dataset == 0 && p.is_distort) return false;
    int first_i = depth_map_list.size();
    first_i -= 1;
    if(first_i < 0) return false;
    point_soph p_spherical = p;
    
    SphericalProjection(p, depth_map_list[first_i]->map_index, depth_map_list[first_i]->project_R, depth_map_list[first_i]->project_T, p_spherical);
        
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f || \
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        cout<<"~~~~INVALID POINTS!!!!~~~~~ hor_ind: "<<p_spherical.hor_ind<<" ver_ind: "<<p_spherical.ver_ind<<endl;
        return false;
    }
    if(should_print) 
        cout<<"check case 3"<<endl;
    int cur_occ_times = 0;
    if (Case3Enter(p_spherical, *depth_map_list[first_i]))
    {
        if (should_print) 
            cout<<"in case 3"<<endl;
        if (!Case3MapConsistencyCheck(p_spherical, *depth_map_list[first_i], case3_interp_en))
        // if(true)
        {
            if(should_print) 
                cout<<"in map consistency "<<2*occ_hor_num3 + 1<<" "<<2*occ_ver_num3 + 1<<" "<<p.glob.transpose()<<endl;
            // vector<point_soph> p_occs;     
            // int cur_occ_times = 0;
            double ti = 0;
            float vi = 0; 
            float min_hor = occ_hor_thr3, min_ver = occ_ver_thr3;
            bool map_cons = true;
            for (int ind_roll = 0; ind_roll < 2*occ_hor_num3 + 1 && map_cons; ind_roll ++)
            {
                for (int ind_pitch = 0; ind_pitch < 2*occ_ver_num3 + 1 && map_cons; ind_pitch ++)
                {
                    int ind = ind_roll*(2*ver_num + 1) + ind_pitch;               
                    int pos_new = p_spherical.position + pos_offset[ind];           
                    if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
                    const vector<point_soph*> & points_in_pixel = depth_map_list[first_i]->depth_map[pos_new]; 
                    
                    if (depth_map_list[first_i]->max_depth_all[pos_new] < p_spherical.vec(2))
                    {
                        if(should_print) cout<<"continue"<<endl;
                        continue;
                    }   

                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const point_soph* p_occ = points_in_pixel[k];
                            
                        if(Case3IsOccluding(p_spherical, *p_occ) && Case3DepthConsistencyCheck(*p_occ, *depth_map_list[first_i]))
                            // && Case3ReCheck(*p_occ, depth_map_list[first_i]->depth_map)
                        {
                            if(should_print) 
                            {
                                cout<<"first occlude"<<p_occ->glob.transpose()<<" angle: "<<\
                                fabs(p_spherical.vec(0)-p_occ->vec(0))<<" "<<fabs(p_spherical.vec(1)-p_occ->vec(1))\
                                <<" "<<(p_spherical.vec(2)-p_occ->vec(2))<<endl;
                            }
                                
                            // if(DepthConsistencyCheck(p_occ, depth_map_list[first_i]))
                            {
                                cur_occ_times = 1;
                                ti = (p_occ->time + p.time)/2;
                                vi = (p_occ->vec(2) - p_spherical.vec(2))/(p.time - p_occ->time);
                                if(should_print) 
                                {
                                    cout<<"vi: "<<vi<<"  "<<ti<<" "<<p.glob.transpose()<<" , "<<p_occ->glob.transpose()<<" time: "\
                                    <<p.time<<" "<<p_occ->time<<" depth:"<<p_spherical.vec(2)<<" "<<p_occ->vec(2)<<endl;
                                }
                                // if(p.is_occu_index[0] != depth_map_list[first_i]->map_index)
                                {
                                    p.is_occu_index[0] = depth_map_list[first_i]->map_index;
                                    p.is_occu_index[1] = pos_new;
                                    p.is_occu_index[2] = k;
                                    p.is_occ_vec = p_spherical.vec;
                                    p.is_occu_times = cur_occ_times;
                                }
                                
                                point_soph  p0 = p;
                                // point_soph*  p1 = points_in_pixel[k];
                                point_soph p1 = *points_in_pixel[k];
                                
                                int i = depth_map_list.size();
                                i = i -2;
                                while(i >= 0)
                                {
                                    // if(p1->is_occu_index[0] != -1 && p1->is_occu_index[0] < depth_map_list.front()->map_index) cout<<"------------error-----------"<<endl;
                                    if(p1.is_occu_index[0] == -1 || p1.is_occu_index[0] < depth_map_list.front()->map_index)
                                    {
                                        
                                        // point_soph  p1_spherical = *p1;
                                        SphericalProjection(p1, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p1);
                                        if(Case3SearchPointOccludedbyP(p1, *depth_map_list[i]))
                                        {
                                            // p1->is_occu_index = p1_spherical.is_occu_index; 
                                            // p1->is_occ_vec = p1_spherical.vec;
                                            p1.is_occ_vec = p1.vec;
                                        }
                                        else
                                        {
                                            if(should_print) 
                                                cout<<"no p2 "<<i<<endl;
                                            break;
                                        }
                                        
                                    }
                                    i = p1.is_occu_index[0]-depth_map_list.front()->map_index;
                                    point_soph* p2 = depth_map_list[i]->depth_map[p1.is_occu_index[1]][p1.is_occu_index[2]];                        
                                    
                                    SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);
                                    if(Case3MapConsistencyCheck(p, *depth_map_list[i], case3_interp_en))
                                    {
                                        map_cons = false;
                                        break;
                                    }
                                    // if(should_print) 
                                    //     cout<<"------get p2 "<<p2->glob.transpose()<<endl;
                                    float vc = -(p1.is_occ_vec(2) - p2->vec(2))/(p1.time - p2->time);
                                    double tc = (p2->time + p1.time)/2;
                                    if(should_print) 
                                    {
                                        cout<<"vc: "<<vc<<" "<<tc<<" "<<p2->glob.transpose()<<" time: "<<p1.time<<" "<<p2->time\
                                        <<" depth:"<<p1.is_occ_vec(2)<<" "<<p2->vec(2)<<" "<<p1.vec(2)<<endl;
                                    }
                                    // point_soph pi_spherical = p; && Case3ReCheck(*p2, depth_map_list[i]->depth_map) 
                                    // if(should_print) 
                                    //         cout<<"get p2 then"<<p.glob.transpose()<<endl;
                                    
                                    if (Case3IsOccluding(p, *p2) &&\
                                        Case3DepthConsistencyCheck(*p2, *depth_map_list[i]) && Case3VelCheck(vi, vc, ti-tc) )
                                    {
                                        
                                        cur_occ_times += 1;
                                        if(should_print) 
                                        {
                                            cout<<"continuous occlude "<<cur_occ_times<<"   "<<p2->glob.transpose()\
                                            <<" angle: "<<fabs(p.vec(0)-p2->vec(0))<<" "\
                                            <<fabs(p.vec(1)-p2->vec(1))<<" "<<\
                                            (p.vec(2)-p2->vec(2))<<endl;
                                        }
                                        if(cur_occ_times >= occluding_times_thr3)
                                        {
                                            // p.is_occu_index[0] = depth_map_list[first_i]->map_index;
                                            // p.is_occu_index[1] = pos_new;
                                            // p.is_occu_index[2] = k;
                                            // p.is_occ_vec = p_spherical.vec;
                                            p.is_occu_times = cur_occ_times;
                                            return true;
                                        }
                                        // p0 = p1;
                                        p1 = *p2;
                                        vi = vc;
                                        ti = tc;
                                    }
                                    else
                                    {
                                        break;
                                    }
                                    i--;
                                }
                            }  
                        } 

                        if(cur_occ_times >= occluding_times_thr3) break;
                    }
                    if(cur_occ_times >= occluding_times_thr3) break;
                }
                if(cur_occ_times >= occluding_times_thr3) break;
            }
        }
        
    }
    if (cur_occ_times >= occluding_times_thr3) 
    {
        p.is_occu_times = cur_occ_times;
        return true;
    }
    return false;
}

bool  DynObjFilter::Case3Enter(point_soph & p, const DepthMap &map_info)
{
    if(p.dyn != STATIC)
    {
        if(should_print) cout<<"case3 enter failed: "<<p.dyn<<endl;
        return false;
    }
    float min_depth = 0;
    float depth_thr3_final = occ_depth_thr3;
    if(map_info.depth_map[p.position].size() > 0)
    {
        const point_soph* min_point = map_info.depth_map[p.position][map_info.min_depth_index_all[p.position]];
        min_depth = min_point->vec(2); //make sure the max index is ok for this max depth
        float delta_t = (p.time - min_point->time);
        depth_thr3_final = min(occ_depth_thr3, v_min_thr3*delta_t);
        if(should_print) cout<<"v_min: "<<v_min_thr3<<" "<<delta_t<<endl;
    }
    //                TBD
    // else
    // {
    //     min_depth = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
    // }
    if(dataset == 0 && p.is_distort)
    {
        depth_thr3_final = enlarge_distort*depth_thr3_final;
    }
    if(p.vec(2) < min_depth - depth_thr3_final)
    {
        case3_num ++;
        return true;
    }
    else
    {
        if(should_print) 
        {
            cout<<map_info.depth_map[p.position].size()<<" case3 enter failed: "<<p.vec(2)<<\
            " , "<<min_depth<<" , "<<depth_thr3_final<<endl;
        }
        return false;
    }
}

bool  DynObjFilter::Case3MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp)
{
    // return false;
    float cur_v_min = v_min_thr3;
    if(dataset == 0 && p.is_distort) cur_v_min = enlarge_distort*cur_v_min;
    // for (int ind_hor = 0; ind_hor < 2*map_cons_hor_num3 + 1; ind_hor ++)
    // { 
    //     for (int ind_ver = 0; ind_ver < 2*map_cons_ver_num3 + 1; ind_ver ++)
    //     {
    for (int ind_hor = -map_cons_hor_num3; ind_hor <= map_cons_hor_num3; ind_hor ++)
    {
        for (int ind_ver = -map_cons_hor_num3; ind_ver <= map_cons_hor_num3; ind_ver ++)      
        {
            // int ind = ind_hor*(2*ver_num + 1) + ind_ver;
            // int pos_new = p.position + pos_offset[ind];
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];                        
  
            if (map_info.max_depth_all[pos_new] > p.vec(2) + map_cons_depth_thr3 && \
                map_info.min_depth_all[pos_new] < p.vec(2) - map_cons_depth_thr3)
            {
                continue;
            }   

            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j];
                if (point->dyn == STATIC && \
                    fabs(p.time-point->time) > frame_dur && \
                    (point->vec(2)-p.vec(2)) <  cur_v_min*fabs(p.time-point->time) && \ 
                    //fabs point->vec(2) < p.vec(2) - map_cons_depth_thr3 && 
                    fabs(p.vec(0)-point->vec(0)) < map_cons_hor_thr3 && \
                    fabs(p.vec(1)-point->vec(1)) < map_cons_ver_thr3)
                {
                    if(should_print) cout<<p.vec(2)<<"case3 map cons: "<<point->vec(2)<<" "<<point->glob.transpose()<<" "<<v_min_thr3*fabs(p.time-point->time)<<endl;
                    return true;
                }               
            }         
        }
        
    }
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float cur_interp = interp_thr3*(depth_map_list.back()->map_index - map_info.map_index + 1);
        // if(p.vec(2) > interp_start_depth2)
        //     cur_interp += ((p.vec(2) - interp_start_depth2)* interp_kp2 + interp_kd2);
        // if(should_print) cout<<"  "<<cur_interp<<endl;
        // if(depth_static < 10E-5)
        {
            float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
            if(should_print) cout<<"depth all:" << depth_all << "  cur depth:"<<p.vec(2)<<endl;
            if( fabs(p.vec(2) - depth_all)  < cur_interp) //depth_all < -11 ||
            {
                return true;
            }
            else
            {
                return false;
            }
            
        }
        
        // float depth_static = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
        
        // float cur_interp = interp_thr3*(depth_map_list.back()->map_index - map_info.map_index + 1);
        // if (should_print) cout<<"cur_interp: "<<interp_thr3<<"  "<<depth_map_list.back()->map_index<<" "<<map_info.map_index<<endl;
        // if (should_print) cout<<"depth interp3: "<<depth_static<<" real:"<<p.vec(2)<<" in "<<map_info.map_index<<" "<<cur_interp<<endl;// || depth_in_map > p.vec(2) + interp_all_thr
        // // if(p.vec(2) > interp_start_depth2)
        // //     cur_interp += ((p.vec(2) - interp_start_depth2)* interp_kp2 + interp_kd2);
        // // if(should_print) cout<<"  "<<cur_interp<<endl;
        // if(depth_static < 10E-5)
        // {
        //     float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
        //     if(should_print) cout<<"depth all:" << depth_all << "  cur depth:"<<p.vec(2)<<endl;
        //     if(depth_all > 0 && depth_all - p.vec(2) < cur_interp)
        //     {
        //         return true;
        //     }
        //     else
        //     {
        //         return false;
        //     }
            
        // }
        // else if(fabs(depth_static - p.vec(2)) < cur_interp)
        // {//depth_static < -11 || p.vec(2) - depth_static  < cur_interp
        //     return true;
        // }

        // float depth_in_map = DepthInterpolationAll(p, map_info.depth_map);
        // float depth_in_map1 = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
        // float cur_interp = interp_thr2;
        // // if(p.vec(2) > interp_start_depth2)
        // //     cur_interp += ((p.vec(2) - interp_start_depth2)* interp_kp2 + interp_kd2);
        // if(should_print) cout<<"  "<<cur_interp<<endl;
        // if(should_print) 
        // {
        //     cout<<"depth interp2: "<<depth_in_map<<" , "<<depth_in_map1<<\
        //     " real: "<<p.vec(2)<<" in "<<map_info.map_index<<endl;
        // }
            
        // if(depth_in_map < -11 || p.vec(2) - depth_in_map  < cur_interp) // 
        // {
        //     if(should_print) cout<<"depth interp2 failed "<<p.vec(2)<<" "<<depth_in_map<<" "<<p.glob.transpose()<<endl;
        //     return true;
        // }
    }
    return false;
}

bool  DynObjFilter::Case3SearchPointOccludedbyP(point_soph & p, const DepthMap &map_info)
{
    for (int ind_hor = 0; ind_hor < 2*occ_hor_num3 + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*occ_ver_num3 + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;               
            int pos_new = p.position + pos_offset[ind];           
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];            
            if (map_info.min_depth_all[pos_new] > p.vec(2))
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* p_cond = points_in_pixel[j];
                // if(should_print && fabs(p_cond->time - p.time) > frame_dur) cout<<"-------error"<<endl;
                if (Case3IsOccluding(p, *p_cond) && Case3DepthConsistencyCheck(*p_cond, map_info) ) 
                    // Case3ReCheck(*p_cond, map_info.depth_map)&&
                {
                    p.is_occu_index[0] = map_info.map_index;
                    p.is_occu_index[1] = pos_new;
                    p.is_occu_index[2] = j;
                    p.occ_vec = p.vec;
                    if(should_print) 
                    cout<<"------find new p2 "<<\
                    depth_map_list[p.is_occu_index[0]-depth_map_list.front()->map_index]->depth_map[p.is_occu_index[1]][p.is_occu_index[2]]->glob.transpose()<<endl;
                    return true;
                }
                // else
                // {
                //     if(should_print) cout<<" failed reason: "<<Case3IsOccluding(p, *p_cond)<<" , "<<Case3DepthConsistencyCheck(*p_cond, map_info)<<\
                //     " , "<<Case3ReCheck(*p_cond, map_info.depth_map)<<endl;
                // }
            }        
        }
    }
    if(should_print) cout<<"------no points "<<p.glob.transpose()<<endl;
    return false;
}

bool  DynObjFilter::Case3IsOccluding(const point_soph & p, const point_soph & p_occ)
{
    if((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID) return false;
    if((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) || \
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }
    float delta_t = p.time - p_occ.time;
    if(delta_t > 0)
    {
        float depth_thr3_final = min(occ_depth_thr3, v_min_thr3*delta_t);
        if(dataset == 0 && p.is_distort) depth_thr3_final = enlarge_distort*depth_thr3_final;
        if (p_occ.vec(2) > p.vec(2)  + depth_thr3_final && \
            fabs(p.vec(0)-p_occ.vec(0)) < occ_hor_thr3 && \
            fabs(p.vec(1)-p_occ.vec(1)) < occ_ver_thr3 )
        {
            return true;
        } 
        else
        {
            // if(should_print) cout<<"thr2: "<<depth_thr2_final<<" "<<fabs(p.vec(0)-p_occ->vec(0))<<" "<<\
            // fabs(p.vec(1)-p_occ->vec(1))<<" "<<p.vec(2)<<" "<<p_occ->vec(2)<<"  "<<p.glob.transpose()<<",,,,"<<\
            // p_occ->glob.transpose()<<endl;
        }              
    }
    
    return false;
}

bool  DynObjFilter::Case3DepthConsistencyCheck(const point_soph & p, const DepthMap &map_info)
{
    // return true;
    float all_minus = 0;
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;//
    if(should_print) cout<<" depth cons ";
    for (int ind_hor = 0; ind_hor < 2*depth_cons_hor_num3 + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*depth_cons_ver_num3 + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;
            int pos_new = p.position + pos_offset[ind];
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;

            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];
            for (int j = 0; j < points_in_pixel.size(); j ++)
            {
                const point_soph* point = points_in_pixel[j]; 
                if(fabs(point->time - p.time) < frame_dur && fabs(point->vec(0)-p.vec(0)) < depth_cons_hor_thr3 && \
                  fabs(point->vec(1)-p.vec(1)) < depth_cons_ver_thr3)
                {
                    // if(should_print) cout<<" , "<<(p.vec(2)-point->vec(2))<<" "<<(p.vec(1)-point->vec(1));
                    all_num ++;
                    if (point->dyn == STATIC) 
                    {
                        float cur_minus = p.vec(2)-point->vec(2);
                        if (fabs(cur_minus) < depth_cons_depth_max_thr3)
                        {
                            num ++;
                            all_minus += fabs(point->vec(2)-p.vec(2));
                        }
                        else if (cur_minus > 0)
                        {
                            smaller_num ++;
                        }
                        else
                        {
                            greater_num ++;
                        }
                    }
                }
            }

        }
    }
    if(should_print) cout<<" cur: "<<p.vec(2)<<endl;
    if(all_num > 0)
    {
        if(num > 1)
        {
            
            float cur_depth_thr = max(depth_cons_depth_thr3, k_depth3*p.vec(2));
            if(should_print) 
                cout<<"depth_cons: "<<all_minus/(num-1)<<" "<<cur_depth_thr <<" num: "<<greater_num<<\
                "  "<<smaller_num<<endl;
            if(all_minus/(num-1) > cur_depth_thr)
            {
                if(should_print) cout<<" depth false"<<endl;
                return false;
            }      

            // if(should_print) cout<<"depth_cons: "<<all_minus/num<<" "<<depth_cons_depth_thr2<<endl;
        }
        if(greater_num == 0 || smaller_num == 0)
        {
            return true;
        }
        else
        {
            if(should_print) cout<<"depth_cons: num error" << greater_num << " " << smaller_num <<endl;
            return false;
        }
    }
    else
    {
        if(should_print) cout<<"depth_cons: no points"<<endl;
        return false;
    }
    
    

}

void  DynObjFilter::Case2Check(vector<point_soph> &points)
{
    std::vector<std::vector<V3F>> tmp_map;
    tmp_map.assign(MAX_2D_N, std::vector<V3F>());
    int len = points.size();
    for (int k = 0; k < len; k++)
    {
        if (points[k].dyn != STATIC) continue;
        V3F cur_p;
        cur_p(2) = float(points[k].local.norm());
        cur_p(0) = atan2f(float(points[k].local(1)), float(points[k].local(0)));
        cur_p(1) = atan2f(float(points[k].local(2)), sqrt(pow(float(points[k].local(0)), 2) + pow(float(points[k].local(1)), 2)));
        int hor_ind   = floor((cur_p(0) + PI_MATH) / hor_resolution_max);
        int ver_ind   = floor((cur_p(1) + 0.5 * PI_MATH) / ver_resolution_max);
        int position  = hor_ind * MAX_1D_HALF + ver_ind;
        points[k].cur_vec = cur_p;
        // pair<V3F, int> cur_info;
        // cur_info.first = cur_p;
        // cur_info.second = position;
        if(position < 0 || position >= MAX_2D_N)
        {
            cout<<"--------position error"<<endl;
            continue;
        }
        tmp_map[position].push_back(cur_p);
    }   
    for(int k = 0; k < len; k++)
    {
        bool is_break = false;  
        if(points[k].dyn == CASE2)
        {
            bool is_print = (k==point_index); 
            V3F cur_p;
            cur_p(2) = float(points[k].local.norm());
            cur_p(0) = atan2f(float(points[k].local(1)), float(points[k].local(0)));
            cur_p(1) = atan2f(float(points[k].local(2)), sqrt(pow(float(points[k].local(0)), 2) + pow(float(points[k].local(1)), 2)));
            points[k].cur_vec = cur_p;
            int hor_ind   = floor((cur_p(0) + PI_MATH) / hor_resolution_max);
            int ver_ind   = floor((cur_p(1) + 0.5 *PI_MATH) / ver_resolution_max);
            int position  = hor_ind * MAX_1D_HALF + ver_ind;
            if(position < 0 || position >= MAX_2D_N)
            {
                cout<<"--------position error"<<endl;
                continue;
            }   
            // cout<<"----------------------k : "<<k<<" , "<<point_index<<" "<<is_print<<endl;
            // if(is_print)
            //     cout<<"k: "<<k<<" , "<<point_index<<", "<<points[k].dyn<<endl;
            for (int ind_hor = 0; ind_hor < 2*occ_hor_num2 + 1; ind_hor ++)
            {
                for (int ind_ver = 0; ind_ver < 2*occ_ver_num2 + 1; ind_ver ++)
                {
                    int ind = ind_hor*(2*ver_num + 1) + ind_ver;               
                    int pos_new = position + pos_offset[ind];           
                    if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
                    const vector<V3F> & points_in_pixel = tmp_map[pos_new];                             
                    for (int j = 0; j < points_in_pixel.size(); j++)
                    {
                        const V3F & p_cur = points_in_pixel[j];                     
                        if (is_print) 
                            cout<<" depth cur: "<<p_cur(2)<<" , "<<points[k].cur_vec(2)<<"  "<<\
                            fabs(p_cur(0) - points[k].cur_vec(0))<<" "<<fabs(p_cur(0) - points[k].cur_vec(0))<<endl;
                        if (points[k].cur_vec(2) > p_cur(2) + 2*occ_depth_thr2 && \
                            fabs(p_cur(0) - points[k].cur_vec(0)) < occ_hor_thr2 && \
                            fabs(p_cur(1) - points[k].cur_vec(1)) < occ_ver_thr2 )
                        {
                            points[k].dyn = STATIC;
                            if(is_print) cout<<"--------recheck failed"<<endl;
                            is_break = true; 
                            break;
                        }
                    }  
                    if(is_break) break;      
                }
                if(is_break) break; 
            }
        }
    }     
}

bool  DynObjFilter::Case2ReCheck(const point_soph &p, const DepthMap2D &depth_map)
{ 
    return true;
    for (int ind_hor = 0; ind_hor < 2*occ_hor_num2 + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*occ_ver_num2 + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;               
            int pos_new = p.position + pos_offset[ind];           
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points = depth_map[pos_new];            
            
            for (int j = 0; j < points.size(); j++)
            {
                const point_soph* p_cur = points[j];
                
                if (should_print) 
                    cout<<" depth cur: "<<p_cur->vec(2)<<" , "<<p.vec(2)<<"  "<<\
                    fabs(p_cur->vec(0) - p.vec(0))<<" "<<fabs(p_cur->vec(1) - p.vec(1))<<endl;
                if (p_cur->dyn == STATIC && fabs(p.time - p_cur->time) < frame_dur && \
                    p.vec(2) + occ_depth_thr2 < p_cur->vec(2)  && \
                    fabs(p_cur->vec(0) - p.vec(0)) < occ_hor_thr2 && \
                    fabs(p_cur->vec(1) - p.vec(1)) < occ_ver_thr2 )
                {
                    if (should_print) 
                    cout<<" recheck failed: "<<endl;
                    return false;
                }
            }      
        }
    }
    return true;
        
}

bool  DynObjFilter::Case3ReCheck(const point_soph &p, const DepthMap2D &depth_map)
{ 
    return true;
    for (int ind_hor = 0; ind_hor < 2*occ_hor_num2 + 1; ind_hor ++)
    {
        for (int ind_ver = 0; ind_ver < 2*occ_ver_num2 + 1; ind_ver ++)
        {
            int ind = ind_hor*(2*ver_num + 1) + ind_ver;               
            int pos_new = p.position + pos_offset[ind];           
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points = depth_map[pos_new];            
            
            for (int j = 0; j < points.size(); j++)
            {
                const point_soph* p_cur = points[j];
                
                if (should_print) 
                    cout<<" depth cur: "<<p_cur->vec(2)<<" , "<<p.vec(2)<<"  "<<\
                    fabs(p_cur->vec(0) - p.vec(0))<<" "<<fabs(p_cur->vec(1) - p.vec(1))<<endl;
                if (p_cur->dyn == STATIC && fabs(p.time - p_cur->time) < frame_dur && \
                    p.vec(2) - occ_depth_thr2 > p_cur->vec(2) && \
                    fabs(p_cur->vec(0) - p.vec(0)) < occ_hor_thr2 && \
                    fabs(p_cur->vec(1) - p.vec(1)) < occ_ver_thr2 )
                {
                    return false;
                }
            }      
        }
    }
    return true;
        
}



void DynObjFilter::publish_dyn(const ros::Publisher & pubLaserCloudEffect, const ros::Publisher & pubLaserCloudEffect_depth, const double & scan_end_time)
{
    // return;
    // if(!laserCloudDynObj->empty())
    // {
    if(cluster_coupled)
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj_clus->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    else
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    cout<<"case1 num: "<<case1_num<<" case2 num: "<<case2_num<<" case3 num: "<<case3_num<<endl;
    case1_num = 0;
    case2_num = 0;
    case3_num = 0;
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudDynObj_world, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes3.header.frame_id = frame_id;
    pubLaserCloudEffect.publish(laserCloudFullRes3);
    if(cluster_coupled || cluster_future)
    {
        sensor_msgs::PointCloud2 laserCloudFullRes4;
        pcl::toROSMsg(*laserCloudDynObj_clus, laserCloudFullRes4);
        laserCloudFullRes4.header.stamp = ros::Time().fromSec(scan_end_time);
        laserCloudFullRes4.header.frame_id = frame_id;
        Cluster.pub_pcl_dyn_extend.publish(laserCloudFullRes4);
//        if(laserCloudDynObj_clus->points.size() > 0 )
//        {
////            string all_points_dir("/home/yihang/Documents/PCD_dynamic/" + to_string(time_ind) + string(".pcd"));
////            pcl::PCDWriter pcd_writer;
////            pcd_writer.writeBinary(all_points_dir, *laserCloudDynObj_clus);
//            dyn_pcd->clear();
//            *dyn_pcd += *laserCloudDynObj_clus;
//        }
    }
    // }
    sensor_msgs::PointCloud2 laserCloudFullRes2;
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);

    // ros::Time time1= ros::Time::now();
    if(cluster_coupled)
    {
        cout<<"Found Steady Objects after cluster, numbers: " << laserCloudSteadObj_clus->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::VoxelGrid<PointType> downSizeFiltermap;
        downSizeFiltermap.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
        PointCloudXYZI laserCloudSteadObj_down;
        downSizeFiltermap.filter(laserCloudSteadObj_down);
        pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
         if(laserCloudSteadObj_pub->points.size() >0)
         {
             cout << "save!!!!!!!!!!" << endl;
             string all_points_dir1(save_file + to_string(time_ind) + string(".pcd"));
             cout << "-----------" << all_points_dir1 << "-----------" << laserCloudSteadObj_clus->size() << endl;
             pcl::PCDWriter pcd_writer;
             pcd_writer.writeBinary(all_points_dir1, *laserCloudSteadObj_clus);
         }
    }
    else
    {
        cout<<"Found Steady Objects, numbers: " << laserCloudSteadObj->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
    }
    // time_file << (ros::Time::now() - time1).toSec() << " ";

    laserCloudFullRes2.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes2.header.frame_id = frame_id;
    pubLaserCloudEffect_depth.publish(laserCloudFullRes2);
    // if(!pcl_his_list.back()->empty())
    // {

    // }


}

void DynObjFilter::publish_dyn(PointCloudXYZI::Ptr &steady_pcd, const M3D & rot_end, const M3D & rot_lidar_to_vehicle_, const V3D & pos_end, const V3D & t_lidar_to_vehicle_, const ros::Publisher & pubLaserCloudEffect, const ros::Publisher & pubLaserCloudEffect_depth, const double & scan_end_time)
{
    // return;
    // if(!laserCloudDynObj->empty())
    // {
    if(cluster_coupled)
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj_clus->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    else
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    cout<<"case1 num: "<<case1_num<<" case2 num: "<<case2_num<<" case3 num: "<<case3_num<<endl;
    case1_num = 0;
    case2_num = 0;
    case3_num = 0;
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudDynObj_world, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes3.header.frame_id = frame_id;
    pubLaserCloudEffect.publish(laserCloudFullRes3);
    if(cluster_coupled || cluster_future)
    {
        sensor_msgs::PointCloud2 laserCloudFullRes4;
        pcl::toROSMsg(*laserCloudDynObj_clus, laserCloudFullRes4);
        laserCloudFullRes4.header.stamp = ros::Time().fromSec(scan_end_time);
        laserCloudFullRes4.header.frame_id = frame_id;
        Cluster.pub_pcl_dyn_extend.publish(laserCloudFullRes4);
//        if(laserCloudDynObj_clus->points.size() > 0 )
//        {
////            string all_points_dir("/home/yihang/Documents/PCD_dynamic/" + to_string(time_ind) + string(".pcd"));
////            pcl::PCDWriter pcd_writer;
////            pcd_writer.writeBinary(all_points_dir, *laserCloudDynObj_clus);
//            dyn_pcd->clear();
//            *dyn_pcd += *laserCloudDynObj_clus;
//        }
    }
    // }
    sensor_msgs::PointCloud2 laserCloudFullRes2;
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);

    // ros::Time time1= ros::Time::now();
    if(cluster_coupled)
    {
        cout<<"Found Steady Objects after cluster, numbers: " << laserCloudSteadObj_clus->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::VoxelGrid<PointType> downSizeFiltermap;
        downSizeFiltermap.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
        PointCloudXYZI laserCloudSteadObj_down;
        downSizeFiltermap.filter(laserCloudSteadObj_down);
        pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
        PointCloudXYZI::Ptr to_save_pcd(new pcl::PointCloud<pcl::PointXYZINormal>);
        if(laserCloudSteadObj_pub->points.size() >0)
        {
            cout << "save!!!!!!!!!!" << endl;
            string all_points_dir1(save_file + to_string(time_ind) + string(".pcd"));
            cout << "-----------" << all_points_dir1 << "-----------" << laserCloudSteadObj_clus->size() << endl;
            pcl::PCDWriter pcd_writer;
            for (size_t i = 0; i < laserCloudSteadObj_clus->points.size(); i++) {
                Eigen::Vector3d body_p(laserCloudSteadObj_clus->points[i].x, laserCloudSteadObj_clus->points[i].y,
                                       laserCloudSteadObj_clus->points[i].z);
                body_p = rot_end * (body_p - pos_end);
//                body_p = rot_lidar_to_vehicle_ * (body_p -
//                       t_lidar_to_vehicle_);
                pcl::PointXYZINormal r_p = laserCloudSteadObj_clus->points[i];
                r_p.x = body_p[0];
                r_p.y = body_p[1];
                r_p.z = body_p[2];
                r_p.normal_x = laserCloudSteadObj_clus->points[i].normal_x;
                r_p.normal_y = laserCloudSteadObj_clus->points[i].normal_y;
                r_p.normal_z = laserCloudSteadObj_clus->points[i].normal_z;
//                pcl::PointXYZINormal pd;
                to_save_pcd->push_back(r_p);
            }
            pcd_writer.writeBinary(all_points_dir1, *to_save_pcd);
            *steady_pcd = *to_save_pcd;
            cout << "----finish-----" << endl;
        }
    }
    else
    {
        cout<<"Found Steady Objects, numbers: " << laserCloudSteadObj->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
    }
    // time_file << (ros::Time::now() - time1).toSec() << " ";

    laserCloudFullRes2.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes2.header.frame_id = frame_id;
    pubLaserCloudEffect_depth.publish(laserCloudFullRes2);
    // if(!pcl_his_list.back()->empty())
    // {

    // }


}

void DynObjFilter::publish_dyn_rgb(PointCloudXYZI::Ptr &steady_pcd, const M3D & rot_end, const M3D & rot_lidar_to_vehicle_, const V3D & pos_end, const V3D & t_lidar_to_vehicle_, const ros::Publisher & pubLaserCloudEffect, const ros::Publisher & pubLaserCloudEffect_depth, const double & scan_end_time)
{
    // return;
    // if(!laserCloudDynObj->empty())
    // {
    if(cluster_coupled)
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj_clus->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    else
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    cout<<"case1 num: "<<case1_num<<" case2 num: "<<case2_num<<" case3 num: "<<case3_num<<endl;
    case1_num = 0;
    case2_num = 0;
    case3_num = 0;
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudDynObj_world, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes3.header.frame_id = frame_id;
    pubLaserCloudEffect.publish(laserCloudFullRes3);
    if(cluster_coupled || cluster_future)
    {
        sensor_msgs::PointCloud2 laserCloudFullRes4;
        pcl::toROSMsg(*laserCloudDynObj_clus, laserCloudFullRes4);
        laserCloudFullRes4.header.stamp = ros::Time().fromSec(scan_end_time);
        laserCloudFullRes4.header.frame_id = frame_id;
        Cluster.pub_pcl_dyn_extend.publish(laserCloudFullRes4);
//        if(laserCloudDynObj_clus->points.size() > 0 )
//        {
////            string all_points_dir("/home/yihang/Documents/PCD_dynamic/" + to_string(time_ind) + string(".pcd"));
////            pcl::PCDWriter pcd_writer;
////            pcd_writer.writeBinary(all_points_dir, *laserCloudDynObj_clus);
//            dyn_pcd->clear();
//            *dyn_pcd += *laserCloudDynObj_clus;
//        }
    }
    // }
    sensor_msgs::PointCloud2 laserCloudFullRes2;
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);

    // ros::Time time1= ros::Time::now();
    if(cluster_coupled)
    {
        cout<<"Found Steady Objects after cluster, numbers: " << laserCloudSteadObj_clus->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::VoxelGrid<PointType> downSizeFiltermap;
        downSizeFiltermap.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
        PointCloudXYZI laserCloudSteadObj_down;
        downSizeFiltermap.filter(laserCloudSteadObj_down);
        pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
        PointCloudXYZI::Ptr to_save_pcd(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZRGB> rgb_pcd;
        if(laserCloudSteadObj_pub->points.size() >0)
        {
            cout << "save!!!!!!!!!!" << endl;
            string all_points_dir1(save_file + to_string(time_ind) + string(".pcd"));
            cout << "-----------" << all_points_dir1 << "-----------" << laserCloudSteadObj_clus->size() << endl;
            pcl::PCDWriter pcd_writer;
            for (size_t i = 0; i < laserCloudSteadObj_clus->points.size(); i++) {
                Eigen::Vector3d body_p(laserCloudSteadObj_clus->points[i].x, laserCloudSteadObj_clus->points[i].y,
                                       laserCloudSteadObj_clus->points[i].z);
                body_p = rot_end * (body_p - pos_end);
//                body_p = rot_lidar_to_vehicle_ * (body_p -
//                       t_lidar_to_vehicle_);
                pcl::PointXYZINormal r_p = laserCloudSteadObj_clus->points[i];
                r_p.x = body_p[0];
                r_p.y = body_p[1];
                r_p.z = body_p[2];
                r_p.normal_x = laserCloudSteadObj_clus->points[i].normal_x;
                r_p.normal_y = laserCloudSteadObj_clus->points[i].normal_y;
                r_p.normal_z = laserCloudSteadObj_clus->points[i].normal_z;
                pcl::PointXYZRGB pd;
                pd.x = body_p[0];
                pd.y = body_p[1];
                pd.z = body_p[2];
                pd.r = static_cast<std::uint8_t>(r_p.normal_x * 255.0f);
                pd.g = static_cast<std::uint8_t>(r_p.normal_y * 255.0f);
                pd.b = static_cast<std::uint8_t>(r_p.normal_z * 255.0f);
                to_save_pcd->push_back(r_p);
                rgb_pcd.push_back(pd);
            }
            pcd_writer.writeBinary(all_points_dir1, rgb_pcd);
            *steady_pcd = *to_save_pcd;
            cout << "----finish-----" << endl;
        }
    }
    else
    {
        cout<<"Found Steady Objects, numbers: " << laserCloudSteadObj->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
    }
    // time_file << (ros::Time::now() - time1).toSec() << " ";

    laserCloudFullRes2.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes2.header.frame_id = frame_id;
    pubLaserCloudEffect_depth.publish(laserCloudFullRes2);
    // if(!pcl_his_list.back()->empty())
    // {

    // }


}

void DynObjFilter::publish_dyn(PointCloudXYZI::Ptr &steady_pcd, const ros::Publisher & pubLaserCloudEffect, const ros::Publisher & pubLaserCloudEffect_depth, const double & scan_end_time)
{
    // return;
    // if(!laserCloudDynObj->empty())
    // {
    if(cluster_coupled)
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj_clus->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    else
    {
        cout<<"Found Dynamic Objects, numbers: " << laserCloudDynObj->points.size() << " Total time: " << time_total << " Average total time: "<< time_total_avr << endl;
    }
    cout<<"case1 num: "<<case1_num<<" case2 num: "<<case2_num<<" case3 num: "<<case3_num<<endl;
    case1_num = 0;
    case2_num = 0;
    case3_num = 0;
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudDynObj_world, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes3.header.frame_id = frame_id;
    pubLaserCloudEffect.publish(laserCloudFullRes3);
    if(cluster_coupled || cluster_future)
    {
        sensor_msgs::PointCloud2 laserCloudFullRes4;
        pcl::toROSMsg(*laserCloudDynObj_clus, laserCloudFullRes4);
        laserCloudFullRes4.header.stamp = ros::Time().fromSec(scan_end_time);
        laserCloudFullRes4.header.frame_id = frame_id;
        Cluster.pub_pcl_dyn_extend.publish(laserCloudFullRes4);
//        if(laserCloudDynObj_clus->points.size() > 0 )
//        {
////            string all_points_dir("/home/yihang/Documents/PCD_dynamic/" + to_string(time_ind) + string(".pcd"));
////            pcl::PCDWriter pcd_writer;
////            pcd_writer.writeBinary(all_points_dir, *laserCloudDynObj_clus);
//            dyn_pcd->clear();
//            *dyn_pcd += *laserCloudDynObj_clus;
//        }
    }
    // }
    sensor_msgs::PointCloud2 laserCloudFullRes2;
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);

    // ros::Time time1= ros::Time::now();
    if(cluster_coupled)
    {
        cout<<"Found Steady Objects after cluster, numbers: " << laserCloudSteadObj_clus->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::VoxelGrid<PointType> downSizeFiltermap;
        downSizeFiltermap.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
        PointCloudXYZI laserCloudSteadObj_down;
        downSizeFiltermap.filter(laserCloudSteadObj_down);
        pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
//        PointCloudXYZI::Ptr to_save_pcd(new pcl::PointCloud<pcl::PointXYZINormal>);
        if(laserCloudSteadObj_pub->points.size() >0)
        {
            cout << "save!!!!!!!!!!" << endl;
            string all_points_dir1(save_file + to_string(time_ind) + string(".pcd"));
            cout << "-----------" << all_points_dir1 << "-----------" << laserCloudSteadObj_clus->size() << endl;
            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinary(all_points_dir1, *laserCloudSteadObj_clus);
            *steady_pcd = *laserCloudSteadObj_clus;
            cout << "----finish-----" << endl;
        }
    }
    else
    {
        cout<<"Found Steady Objects, numbers: " << laserCloudSteadObj->points.size() << endl;
        if(laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times ++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for(int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
    }
    // time_file << (ros::Time::now() - time1).toSec() << " ";

    laserCloudFullRes2.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes2.header.frame_id = frame_id;
    pubLaserCloudEffect_depth.publish(laserCloudFullRes2);
    // if(!pcl_his_list.back()->empty())
    // {

    // }


}


void DynObjFilter::publish_img(const ros::Publisher & image_pub, std::deque<sensor_msgs::CompressedImage> &image_buffer, const double & scan_end_time)
{
    return;
    while(!image_buffer.empty() && (image_buffer.front().header.stamp.toSec() - scan_end_time) < 0.1)
    {   
        double image_time = image_buffer.front().header.stamp.toSec();
        std::cout << "image - scan time: " << image_time - scan_end_time << std::endl;
        if(fabs(image_time - scan_end_time) < frame_dur && fabs(image_buffer[1].header.stamp.toSec() - scan_end_time) > fabs(image_time - scan_end_time))
        {   
            image_pub.publish(image_buffer.front());
            image_buffer.pop_front();
            break;
        }
        else if (image_time - scan_end_time < 0.)
        {
            image_pub.publish(image_buffer.front());
            image_buffer.pop_front();
        }
        else
        {
            image_buffer.pop_front();
        }
    }
}

void DynObjFilter::publish_hist(const ros::Publisher & pubLaserCloudhist_depth, const double & scan_end_time)
{
    return;
    if(!laserCloudSteadObj_hist->empty())
    {
        cout<<"Historys numbers: " << laserCloudSteadObj_hist->points.size() << endl;
        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudSteadObj_hist, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
        laserCloudFullRes3.header.frame_id = frame_id;
        pubLaserCloudhist_depth.publish(laserCloudFullRes3);
    }
}

void DynObjFilter::ReadFromLabel(ros::Publisher &pubLaserCloudEffect, PointCloudXYZI::Ptr &feats_undistort, const M3D & rot_end, const V3D & pos_end, string &label_folder, int cur_frame, const double & scan_end_time)
{
    cout << "frame : " << cur_frame << endl;
    string label_file = label_folder;
    stringstream ss;
    ss << setw(6) << setfill('0') << cur_frame + 1 ;
    label_file += ss.str(); 
    label_file.append(".label");

    std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
    if(!label_input.good())
    {
        std::cerr << "Could not read label file: " << label_file << std::endl;
        exit(EXIT_FAILURE);
    }
    label_input.seekg(0, std::ios::beg);


    pcl::PointCloud<pcl::PointXYZI>::Ptr dyn_points (new pcl::PointCloud<pcl::PointXYZI>);
    
    for (int i=0; i<feats_undistort->points.size(); i++) 
    {   
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
        V3D p_glob(rot_end * (p_body) + pos_end);
        pcl::PointXYZI point;
        point.x = p_glob(0);
        point.y = p_glob(1);
        point.z = p_glob(2);

        int label_num = -1, pred_num = -1;
        label_input.read((char *) &label_num, sizeof(int));
        label_num = label_num & 0xFFFF;

        point.intensity = 0;
        if(label_num >= 251 and label_num < 260)
        {   
            point.intensity = 10;
            dyn_points->push_back(point);
        }
        else
        {
            point.intensity = 20;
        }
    }
    cout << "dyn points size: " << dyn_points->points.size() << endl;

    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(*dyn_points, pcl_ros_msg);
    pcl_ros_msg.header.frame_id = "camera_init";
    pcl_ros_msg.header.stamp = ros::Time().fromSec(scan_end_time);
    pubLaserCloudEffect.publish(pcl_ros_msg);
}

void DynObjFilter::set_path(string file_path, string file_path_origin)
{
    is_set_path = true;
    out_file = file_path;
    out_file_origin = file_path_origin;
}

