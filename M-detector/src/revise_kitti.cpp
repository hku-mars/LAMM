
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

#include "preprocess.h"

using namespace std;


ros::Publisher pub_pointcloud, pub_pointcloud_eff, pub_pointcloud_all;

int cur_frame = 1, k = 3;

         
void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    pcl::PointCloud<PointType>::Ptr feats_undistort(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg_in, *feats_undistort);
    // calib the point Cloud 
    // int t1 = 1;
    // int t2 = 2;
    // int& p1 = t1;
    // int& p2 = t2;
    // // p1 = &t1;
    // // p2 = &t2;
    // cout<<"raw: "<<p1<<" , "<<p2<<endl;
    // p1 = p2;
    // cout<<"after"<<p1<<" , "<<p2<<" *p1: "<<p1<<" , "<<t1<<endl;
    
    for (size_t i = 0; i < feats_undistort->size(); i++) 
    { 
        PointType pi = feats_undistort->points[i]; 
        double range = sqrt(pi.x * pi.x + pi.y * pi.y + pi.z * pi.z); 
        // 0.225 good 
        double calib_vertical_angle = deg2rad(0.15); 
        // Eigen::Vector3d euler_angle(0, deg2rad(-0.6), 0); 
        // Eigen::Matrix3d calib_rotation; 
        // calib_rotation = 
        // Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) * 
        // Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) * 
        // Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX()); 
        // Eigen::Vector3d pv(pi.x, pi.y, pi.z); 
        // pv = calib_rotation * pv; 
        // pi.x = pv[0]; 
        // pi.y = pv[1]; 
        // pi.z = pv[2]; 
        double vertical_angle = atan2(pi.z, range) + calib_vertical_angle; 
        double horizon_angle = atan2(pi.y, pi.x); 
        pi.z = range * tan(vertical_angle); 
        double project_len = range * cos(vertical_angle); 
        pi.x = project_len * cos(horizon_angle); 
        pi.y = project_len * sin(horizon_angle); 
        feats_undistort->points[i] = pi; 
    }

    
    
    
    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(*feats_undistort, pcl_ros_msg);
    pcl_ros_msg.header = msg_in->header;
    pub_pointcloud.publish(pcl_ros_msg);
    

    

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "replay_pc");
    ros::NodeHandle nh;
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    
    
    
    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = nh.subscribe("/velodyne_points", 200000, PointsCallback);

    pub_pointcloud  = nh.advertise<sensor_msgs::PointCloud2>
            ("/velodyne_points_revise", 100000);

    ros::spin();
    return 0;
}
