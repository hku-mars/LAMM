
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <types.h>
#include "DynObjFilter.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/filters/random_sample.h>

#include <deque>

#include "preprocess.h"

using namespace std;

shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());
M3D cur_rot = Eigen::Matrix3d::Identity();
V3D cur_pos = Eigen::Vector3d::Zero();
// PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());

int     QUAD_LAYER_MAX  = 1;
int     dyn_windows_num = 3;
int     occlude_windows = 3;
int     point_index = 0;
float   VER_RESOLUTION_MAX  = 0.01;
float   HOR_RESOLUTION_MAX  = 0.01;
float   angle_noise     = 0.001;
float   angle_occlude     = 0.02;
float   dyn_windows_dur = 0.5;
bool    dyn_filter_en = true, dyn_filter_dbg_en = true;
string  points_topic, odom_topic;
double  lidar_end_time = 0;
M3D last_rot = Eigen::Matrix3d::Identity();
V3D last_pos = Eigen::Vector3d::Zero();
Eigen::Quaterniond last_q(1, 0, 0, 0);
bool init = false;
int dataset = 0;

std::string pose_file, out_file, out_file_origin, pose_folder;
std::vector<Eigen::Matrix4d> trans, raw_trans;
std::vector<M3D> rots, raw_rots;
std::vector<V3D> poss, raw_poss;
deque<M3D> odom_rots;
deque<V3D> odom_poss;
deque<double> odom_times;
deque<boost::shared_ptr<PointCloudXYZI>> odom_pcs;
int max_odom_buffer = 10;
int cur_odom_pc = 0;


ros::Publisher pubLaserCloudEffect_depth, pubLaserCloudhist_depth, pubLaserCloudEffect, odom_pub, all_pub;
ros::Publisher pubLaserCloudFullRes;
bool use_file = true;
int cur_frame = 0;

void InitOdomBuffer()
{
    // for(int i = 0; i < max_odom_buffer; i++)
    // {

    // }
}

void get_transforms(std::string pose_file)
{
    std::string line;
    std::ifstream ifs;
    ifs.open(pose_file, std::ios::in);
    // FILE *file = NULL;
    // if((file = fopen(pose_file.data(), "r")) == NULL) cout<<"open failed"<<endl;
    // char c[1000];
    // fscanf(file,"%[^\n]", c);
    // printf("````%s", c);
    if (!ifs)
    {
        std::cout << "cannot open file: " << pose_file << std::endl;
        return ;
    }
    
    while (std::getline(ifs, line) && ifs.good())
    {
        if (line.empty()) return;
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<double> vdata;
        while (std::getline(lineStream, cell, ' '))
        {
        vdata.push_back(std::stod(cell));
        }

        double roll, pitch, yaw;
        Eigen::Matrix4d tform;
        M3D c_rot;
        V3D c_pos;
        tf::Matrix3x3 tf_mat;
        tf_mat.setValue(vdata[0], vdata[1], vdata[2], vdata[4], vdata[5], vdata[6], vdata[8], vdata[9], vdata[10]);
        // tf_mat.getRPY(roll, pitch, yaw);
        // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        // tf_mat.setRotation(tf::Quaternion(quat.z, -quat.x, -quat.y, quat.w));
        // tform(0, 0) = tf_mat[0][0]; tform(0, 1) = tf_mat[0][1]; tform(0, 2) = tf_mat[0][2]; tform(0, 3) = vdata[11];
        // tform(1, 0) = tf_mat[1][0]; tform(1, 1) = tf_mat[1][1]; tform(1, 2) = tf_mat[1][2]; tform(1, 3) = -vdata[3];
        // tform(2, 0) = tf_mat[2][0]; tform(2, 1) = tf_mat[2][1]; tform(2, 2) = tf_mat[2][2]; tform(2, 3) = -vdata[7];
        tform(0, 0) = tf_mat[0][0]; tform(0, 1) = tf_mat[0][1]; tform(0, 2) = tf_mat[0][2]; tform(0, 3) = vdata[3];
        tform(1, 0) = tf_mat[1][0]; tform(1, 1) = tf_mat[1][1]; tform(1, 2) = tf_mat[1][2]; tform(1, 3) = vdata[7];
        tform(2, 0) = tf_mat[2][0]; tform(2, 1) = tf_mat[2][1]; tform(2, 2) = tf_mat[2][2]; tform(2, 3) = vdata[11];
        tform(3, 0) = 0; tform(3, 1) = 0; tform(3, 2) = 0; tform(3, 3) = 1;
        c_rot = tform.block<3,3>(0,0);
        c_pos = tform.block<3,1>(0,3);
        trans.push_back(tform);
        rots.push_back(c_rot);
        poss.push_back(c_pos);
        // static int count = 0;
        // std::cout << count++ << "transform: \n" << tform << std::endl;
    }
    
}

void get_transforms_fromwaymo(std::string pose_folder)
{
    int all_num = 0;
    if(pose_folder != "")
    {
        DIR* dir;	
        dir = opendir(pose_folder.c_str());
        struct dirent* ptr;
        while((ptr = readdir(dir)) != NULL)
        {
            if(ptr->d_name[0] == '.') {continue;}
            all_num++;
        }
        closedir(dir);
    }
    else
    {
        return;
    }
    cout<<" all_num: "<<all_num<<endl;
    for(int i = 0; i < all_num; i++)
    {
        string cur_pose_file = pose_folder;
        stringstream sss;
        sss << setw(6) << setfill('0') << i ;
        cur_pose_file += sss.str(); 
        cur_pose_file.append(".txt");
        std::ifstream ifs;
        ifs.open(cur_pose_file, std::ios::in);
        if (!ifs)
        {
            std::cout << "cannot open file: " << cur_pose_file << std::endl;
            return ;
        }
        string line;
        std::vector<double> vdata;
        while (std::getline(ifs, line) && ifs.good())
        {
            if (line.empty()) continue;
            std::stringstream lineStream(line);
            std::string cell;
            
            while (std::getline(lineStream, cell, ' '))
            {
                vdata.push_back(std::stod(cell));
            }

           
        } 
        double roll, pitch, yaw;
        Eigen::Matrix4d tform;
        M3D c_rot;
        V3D c_pos;
        tf::Matrix3x3 tf_mat;
        tf_mat.setValue(vdata[0], vdata[1], vdata[2], vdata[4], vdata[5], vdata[6], vdata[8], vdata[9], vdata[10]);
        tf_mat.getRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        tf_mat.setRotation(tf::Quaternion(quat.z, -quat.x, -quat.y, quat.w));
        tform(0, 0) = tf_mat[0][0]; tform(0, 1) = tf_mat[0][1]; tform(0, 2) = tf_mat[0][2]; tform(0, 3) = vdata[3];
        tform(1, 0) = tf_mat[1][0]; tform(1, 1) = tf_mat[1][1]; tform(1, 2) = tf_mat[1][2]; tform(1, 3) = vdata[7];
        tform(2, 0) = tf_mat[2][0]; tform(2, 1) = tf_mat[2][1]; tform(2, 2) = tf_mat[2][2]; tform(2, 3) = vdata[11];
        tform(3, 0) = 0; tform(3, 1) = 0; tform(3, 2) = 0; tform(3, 3) = 1;
        c_rot = tform.block<3,3>(0,0);
        c_pos = tform.block<3,1>(0,3);
        raw_trans.push_back(tform);
        raw_rots.push_back(c_rot);
        raw_poss.push_back(c_pos);
    }
    Eigen::Matrix4d rot_lidar;
    rot_lidar << -8.47772463e-01, -5.30354157e-01, -2.51365711e-03, 1.43000000e+00, \
                 5.30355440e-01, -8.47775368e-01,  1.80144262e-04, 0.00000000e+00, \  
                 -2.22655684e-03, -1.18041038e-03,  9.99996825e-01, 2.18400000e+00, \
                 0, 0, 0, 1;
    for(int i = 0; i < raw_trans.size(); i++)
    {
        Eigen::Matrix4d raw_tf = raw_trans[i];
        Eigen::Matrix4d cur_tf = raw_trans[0].inverse()*raw_tf*rot_lidar;
        trans.push_back(cur_tf);
        rots.push_back(cur_tf.block<3,3>(0,0));
        poss.push_back(cur_tf.block<3,1>(0,3));
    }
    cout<<" all_num: "<<trans.size() << ", "<<rots.size()<<" , "<<poss.size() <<endl;
    
    
    
}

void OdomCallback(const nav_msgs::Odometry &cur_odom)
{
    Eigen::Quaterniond cur_q;
    geometry_msgs::Quaternion tmp_q;
    tmp_q = cur_odom.pose.pose.orientation;
    // tmp_q.x = cur_odom.pose.pose.orientation.z;
    // tmp_q.y = -cur_odom.pose.pose.orientation.x;
    // tmp_q.z = -cur_odom.pose.pose.orientation.y;
    // tmp_q.w = cur_odom.pose.pose.orientation.w;
    Eigen::Matrix3d rot_r = Eigen::Matrix3d::Identity(); 
    rot_r << 0, -1, 0,
            0, 0, -1,
            1, 0, 0;
    // Eigen::Quaterniond rot_q(rot_r);
    Eigen::Quaterniond rot_q(-0.5, -0.5, 0.5, -0.5);

    // Eigen::Quaterniond q1(1,0,0,0);
    // Eigen::Quaterniond q2(-0.5, -0.5, 0.5, -0.5);
//     Eigen::Quaterniond q4(-0.5, 0.5, -0.5, 0.5);
    // Eigen::Quaterniond q3;
    // q3 = q1*q2;
    // cout<<"q3 :"<<q3.x()<<", "<<q3.y()<<", "<<q3.z()<<", "<<q3.w()<<endl;
    // cout<<"rot_q :"<<rot_q.x()<<", "<<rot_q.y()<<", "<<rot_q.z()<<", "<<rot_q.w()<<endl;
    // Eigen::Matrix3d r3(q3);
    // cout<<r3<<endl;
//     Eigen::Matrix3d rr(q4);
//     cout<<rr<<endl;
    // Eigen::Quaterniond q5(r3);
    // cout<<"q5 :"<<q5.x()<<", "<<q5.y()<<", "<<q5.z()<<", "<<q5.w()<<endl;
    // q3 = q3*q4;
//     cur_q = rot_q*cur_q;
    // cout<<"q3 :"<<q3.x()<<", "<<q3.y()<<", "<<q3.z()<<", "<<q3.w()<<endl;
    // cout<<"cur_q :"<<cur_q.x()<<", "<<cur_q.y()<<", "<<cur_q.z()<<", "<<cur_q.w()<<endl;

    tf::quaternionMsgToEigen(tmp_q, cur_q);
//     cur_q = last_q*cur_q;
    // cout<<"cur_q :"<<cur_q.x()<<", "<<cur_q.y()<<", "<<cur_q.z()<<", "<<cur_q.w()<<endl;
    // cout<<"rot_q :"<<rot_q.x()<<", "<<rot_q.y()<<", "<<rot_q.z()<<", "<<rot_q.w()<<endl;
//     cur_q = rot_q*cur_q;
    // cout<<"cur_q :"<<cur_q.x()<<", "<<cur_q.y()<<", "<<cur_q.z()<<", "<<cur_q.w()<<endl;
    // Eigen::Matrix3d temp_r(cur_q);
    // 
    
    // cout<<rot_q.x()<<", "<<rot_q.y()<<", "<<rot_q.z()<<", "<<rot_q.w()<<endl;
    cur_rot = cur_q.matrix();//rot_r*= temp_r
//     cur_rot = cur_rot*last_rot;

    // cur_pos << cur_odom.pose.pose.position.z, -cur_odom.pose.pose.position.x, -cur_odom.pose.pose.position.y;
    cur_pos << cur_odom.pose.pose.position.x, cur_odom.pose.pose.position.y, cur_odom.pose.pose.position.z;
//     cur_pos = rot_q*cur_pos;
//     cur_pos = last_q*cur_pos - last_pos;
    
    // Eigen::Matrix4d velo2cam, cam2velo, cur = Eigen::Matrix4d::Identity();
    // cam2velo << 0, 0, 1, 0,
    //             -1, 0, 0, 0,
    //             0, -1, 0, 0.08,
    //             0, 0, 0, 1;
    // velo2cam << 0, -1, 0, 0,
    //             0, 0, -1, 0,
    //             1, 0, 0, -0.08,
    //             0, 0, 0, 1;
    // cout<<"rot:"<<cur_rot<<",  "<<cur_pos<<endl;
    // cur.block<3,3>(0,0) = cur_rot;
    // cur.block<3,1>(0,3) = cur_pos;
    // cout<<"cur2:"<<cur<<endl;
    // cur = cam2velo*cur*velo2cam;
    // cur_rot = cur.block<3,3>(0,0);
    // cur_pos = cur.block<3,1>(0,3);


    
    
    lidar_end_time = cur_odom.header.stamp.toSec();
    nav_msgs::Odometry cur_odom_real;
    cur_odom_real = cur_odom;
    cur_odom_real.pose.pose.position.x = cur_pos[0];
    cur_odom_real.pose.pose.position.y = cur_pos[1];
    cur_odom_real.pose.pose.position.z = cur_pos[2];
    cur_odom_real.pose.pose.orientation.x = cur_q.x();
    cur_odom_real.pose.pose.orientation.y = cur_q.y();
    cur_odom_real.pose.pose.orientation.z = cur_q.z();
    cur_odom_real.pose.pose.orientation.w = cur_q.w();

    odom_pub.publish(cur_odom_real);
    if(!init)
    {
        last_q = cur_q;
        last_pos = cur_pos;
        init = true;
    }
//     last_rot = cur_rot;
//     last_pos = cur_pos;
}

void FastLioOdomCallback(const nav_msgs::Odometry &cur_odom)
{
    // cout<<" im odom "<<endl;
    Eigen::Quaterniond cur_q;
    geometry_msgs::Quaternion tmp_q;
    tmp_q = cur_odom.pose.pose.orientation;
    tf::quaternionMsgToEigen(tmp_q, cur_q);
    cur_rot = cur_q.matrix();
    cur_pos << cur_odom.pose.pose.position.x, cur_odom.pose.pose.position.y, cur_odom.pose.pose.position.z;
    odom_rots.push_back(cur_rot);
    odom_poss.push_back(cur_pos);

    lidar_end_time = cur_odom.header.stamp.toSec();
    odom_times.push_back(lidar_end_time);
    // cout<<"odom : "<<cur_pos.transpose()<<endl;
}

void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    if(use_file)
    {
        ros::Duration(0.001);
        PointCloudXYZI::Ptr feats_all(new PointCloudXYZI());
        PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
        lidar_end_time = msg_in->header.stamp.toSec();
        pcl::fromROSMsg(*msg_in, *feats_undistort);
        cout<<"points length: "<<msg_in->data.size()<<",,"<<feats_undistort->size()<<endl;
        // cout<<"msg info:"<<feats_undistort->points[0].normal<<" "<<feats_undistort->points[0].curvature<<endl;
        for(int i = 0; i<feats_undistort->size(); i++)
        {
            V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);

            V3D p_glob(rots[cur_frame] * (p_body) + poss[cur_frame]);
            // V3D p_glob(cur_rot * (p_body) + cur_pos);
            // V3D p_glob(state_point.rot * p_body + state_point.pos);

            PointType po;
            po.x = p_glob(0);
            po.y = p_glob(1);
            po.z = p_glob(2);
            feats_all->push_back(po);
            // double dis = pow(po.x, 2) + pow(po.y, 2) + pow(po.z, 2);
            // if(dis<0.00005) cout<<"------"<<endl;
        }
        string file_name = out_file;
        stringstream ss;
        ss << setw(6) << setfill('0') << cur_frame ;
        file_name += ss.str(); 
        file_name.append(".label");

        string file_name_origin = out_file_origin;
        stringstream sss;
        sss << setw(6) << setfill('0') << cur_frame ;
        file_name_origin += sss.str(); 
        file_name_origin.append(".label");
        if(file_name.length() > 15 || file_name_origin.length() > 15)
            DynObjFilt->set_path(file_name, file_name_origin);

        DynObjFilt->publish_hist(pubLaserCloudhist_depth, lidar_end_time);
        // DynObjFilt->filter(feats_undistort, cur_rot, cur_pos, lidar_end_time);
        DynObjFilt->filter(feats_undistort, rots[cur_frame], poss[cur_frame], lidar_end_time);
        // cout<<"lidar_end_time: "<<lidar_end_time<<endl;
        
        DynObjFilt->publish_dyn(pubLaserCloudEffect, pubLaserCloudEffect_depth, lidar_end_time);
        cur_frame += 1;

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*feats_all, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time::now();
        laserCloudFullRes3.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);
    }
    else
    {
        // PointCloudXYZI::Ptr feats_all(new PointCloudXYZI());
        boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
        pcl::fromROSMsg(*msg_in, *feats_undistort);
        odom_pcs.push_back(feats_undistort);
        // cout<<" pointcloud: "<<feats_undistort->size()<<endl;
    }
    
}

void WaymoCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    ros::Duration(0.001);
    PointCloudXYZI::Ptr feats_all(new PointCloudXYZI());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr raw_points(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
    lidar_end_time = msg_in->header.stamp.toSec();
    pcl::fromROSMsg(*msg_in, *feats_undistort);
    cout<<"points length: "<<msg_in->data.size()<<",,"<<feats_undistort->size() << " , " << trans.size() <<endl;
    // cout<<"msg info:"<<feats_undistort->points[0].normal<<" "<<feats_undistort->points[0].curvature<<endl;
    Eigen::Matrix3d rot_lidar;
    rot_lidar << -8.47772463e-01, -5.30354157e-01, -2.51365711e-03, \
                 5.30355440e-01, -8.47775368e-01,  1.80144262e-04, \  
                 -2.22655684e-03, -1.18041038e-03,  9.99996825e-01;
    Eigen::Vector3d pos_lidar;
    pos_lidar << 1.43000000e+00, 0.00000000e+00, 2.18400000e+00;
    
    for(int i = 0; i < (int)(feats_undistort->size()); i++)
    {
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
        
        // V3D p_body_raw (rot_lidar*p_body + pos_lidar);
        // V3D p_glob_t_raw(raw_rots[cur_frame] * (p_body_raw) + raw_poss[cur_frame]);
        // V3D p_glob_raw(raw_rots[0].transpose() * (p_glob_t_raw - raw_poss[0]));
        
        V3D p_glob(rots[cur_frame]*p_body + poss[cur_frame]);
        // cout<<" point: "<<p_glob_raw.transpose()<<"   ,   "<<p_glob.transpose()<<endl;
        // cout<<"p_body: "<<p_body.transpose()<<" , "<<p_glob.transpose()<<endl;
        // cout<<" rot: "<< rots[cur_frame]<<" ,,,,, "<<poss[cur_frame].transpose()<<endl;
        // cout<<" rot 0: "<< rots[0]<<" ,,,,, "<<poss[0].transpose()<<endl;
        // getchar();
        // V3D p_glob_t(rots[cur_frame].transpose() * (p_body - poss[cur_frame]));
        // V3D p_glob(rots[0] * (p_glob_t) + poss[0]);
        // V3D p_glob(cur_rot * (p_body) + cur_pos);
        // V3D p_glob(state_point.rot * p_body + state_point.pos);

        PointType po;
        po.x = p_glob(0);
        po.y = p_glob(1);
        po.z = p_glob(2);
        feats_all->push_back(po);
        // cout<<"x y z "<<po.x<<" , "<<po.y<<" , "<<po.z<<endl;
        // PointType pi;
        // pi.x = feats_undistort->points[i].x;
        // pi.y = feats_undistort->points[i].y;
        // pi.z = feats_undistort->points[i].z;
        // feats_undistort->push_back(pi);
        // double dis = pow(po.x, 2) + pow(po.y, 2) + pow(po.z, 2);
        // if(dis<0.00005) cout<<"------"<<endl;
    }
    cout<<"feats_all size: "<<feats_all->size()<<endl;
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*feats_all, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time::now();
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish(laserCloudFullRes3);

    // pcl::VoxelGrid<pcl::PointXYZINormal> sor;//滤波器处理对象
    // sor.setInputCloud(feats_all);//设置输入点云
    // sor.setLeafSize(0.01f, 0.01f, 0.01f);//设置滤波器处理时采用的体素大小的参数，体素大小是长宽高均为0.01
    // sor.filter(*feats_undistort);
    // pcl::RandomSample<pcl::PointXYZINormal> rs;
    // rs.setInputCloud(feats_all);
    // rs.setSample(60000);
    // rs.filter(*feats_undistort);
//     pcl::PointCloud<pcl::PointXYZINormal> laserCloud;
//     pcl::PointXYZINormal cur_point;
//     V3D global_pos,local_pos;
//     for(int i=0; i<feats_undistort->points.size(); i++)
//     {
//         cur_point.intensity = feats_undistort->points[i].intensity;
//         local_pos << feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z;
//         // global_pos = cur_rot*local_pos + cur_pos;
//         global_pos = rots[cur_frame]*local_pos + poss[cur_frame];
//         cur_point.x = global_pos[0];
//         cur_point.y = global_pos[1];
//         cur_point.z = global_pos[2];
//         laserCloud.push_back(cur_point);
//     }
//     sensor_msgs::PointCloud2 pcl_ros_msg;
//     pcl::toROSMsg(laserCloud, pcl_ros_msg);
//     pcl_ros_msg.header = msg_in->header;
// //     pcl_ros_msg.header.frame_id = "/livox_frame";
//     all_pub.publish(pcl_ros_msg);
    // string file_name = out_file;
    // stringstream ss;
	// ss << setw(6) << setfill('0') << cur_frame ;
	// file_name += ss.str(); 
	// // cout << str;

    // // file_name += to_string(cur_frame); 
    // file_name.append(".txt");
    // DynObjFilt->set_path(file_name);
    // DynObjFilt->publish_hist(pubLaserCloudhist_depth, lidar_end_time);
    // DynObjFilt->filter(feats_undistort, cur_rot, cur_pos, lidar_end_time);

    string file_name = out_file;
    stringstream ss;
    ss << setw(6) << setfill('0') << cur_frame ;
    file_name += ss.str(); 
    file_name.append(".label");

    string file_name_origin = out_file_origin;
    stringstream sss;
    sss << setw(6) << setfill('0') << cur_frame ;
    file_name_origin += sss.str(); 
    file_name_origin.append(".label");
    if(file_name.length() > 15 || file_name_origin.length() > 15)
        DynObjFilt->set_path(file_name, file_name_origin);

    DynObjFilt->filter(feats_undistort, rots[cur_frame], poss[cur_frame], lidar_end_time);
    // // cout<<"lidar_end_time: "<<lidar_end_time<<endl;
    
    DynObjFilt->publish_dyn(pubLaserCloudEffect, pubLaserCloudEffect_depth, lidar_end_time);
    cur_frame += 1;
}

void TimerCallback(const ros::TimerEvent& e)
{
    if(odom_pcs.size() > 0 && odom_poss.size() > 0 && odom_rots.size() > 0 && odom_times.size() > 0)
    {
        boost::shared_ptr<PointCloudXYZI> cur_pc = odom_pcs.at(0);
        odom_pcs.pop_front();
        auto cur_rot = odom_rots.at(0);
        odom_rots.pop_front();
        auto cur_pos = odom_poss.at(0);
        odom_poss.pop_front();
        auto cur_time = odom_times.at(0);
        odom_times.pop_front();

        // cout<<"points length: "<<cur_pc->size()<<" . " << cur_pos.transpose()<<endl;
        DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time);
        DynObjFilt->publish_dyn(pubLaserCloudEffect, pubLaserCloudEffect_depth, cur_time);
    }
    // else
    // {
    //     cout<<"size: "<<odom_pcs.size()<<" , "<<odom_poss.size()<<" , "<<odom_rots.size()<<" , "<<odom_times.size()<<endl;
    // }
    
        // cout<<"msg info:"<<feats_undistort->points[0].normal<<" "<<feats_undistort->points[0].curvature<<endl;
        
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynfilter_odom");
    ros::NodeHandle nh;


    nh.param<bool>("dyn_obj/use_odom_file", use_file, true);
    nh.param<string>("dyn_obj/pose_file", pose_file, "");



    nh.param<string>("dyn_obj/out_file", out_file,"");
    nh.param<string>("dyn_obj/out_file_origin", out_file_origin,"");
    nh.param<string>("common/lid_out_topic", points_topic, "/velodyne_points");
    nh.param<string>("common/odom_out_topic", odom_topic, "/aft_mapped_to_init");
    ros::Publisher pub_pcl_dyn_extend = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl_dyn_extend", 10000);
    ros::Publisher cluster_vis_high = nh.advertise<visualization_msgs::MarkerArray>("/cluster_vis_high", 10000);
    ros::Publisher pub_ground_points = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10000);

    // DynObjFilt->init(dyn_windows_num, dyn_windows_dur, occlude_windows, HOR_RESOLUTION_MAX, VER_RESOLUTION_MAX, angle_noise, angle_occlude, dyn_filter_dbg_en, point_index);
    DynObjFilt->SetParam(nh);
    DynObjFilt->Cluster.Init(pub_pcl_dyn_extend, cluster_vis_high, pub_ground_points);
    
    
    
    /*** ROS subscribe initialization ***/
    

    // ros::Subscriber sub_imu = nh.subscribe(odom_topic, 200000, OdomCallback);
    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_odom", 100000);
    // ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/cloud_registered_body", 100000);
    pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    // all_pub  = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/cloud_registered", 100000);
    // ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/cloud_dyn_obj", 100000);
    pubLaserCloudEffect_depth = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_dyn_obj_removed", 100000);
    pubLaserCloudhist_depth = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected_hist", 100000);    
    // ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/Laser_map", 100000);
    // ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
    //         ("/path", 100000);
    // ros::Publisher plane_pub = nh.advertise<visualization_msgs::Marker>
    //         ("/planner_normal", 1);
    
    odom_pub = nh.advertise<nav_msgs::Odometry>
            ("/odom_real", 1);
    
    
    ros::Timer timer;
    ros::Subscriber sub_pcl;
    ros::Subscriber sub_odom;
    if(use_file)
    {
        if( dataset == 2)
        {
            sub_pcl = nh.subscribe(points_topic, 200000, WaymoCallback);
            get_transforms_fromwaymo(pose_folder);
        }
        else
        {
            sub_pcl = nh.subscribe(points_topic, 200000, PointsCallback);
            trans.clear();
            get_transforms(pose_file);
        }
        
    }
    else
    {
        sub_pcl = nh.subscribe(points_topic, 200000, PointsCallback);
        // cout<<" in no file"<<endl;
        sub_odom = nh.subscribe(odom_topic, 200000, FastLioOdomCallback);
        timer = nh.createTimer(ros::Duration(0.01), TimerCallback);
    }

    
    // cout<<"trans size: "<<trans.size()<<endl;

    ros::spin();
    return 0;
}
