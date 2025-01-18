
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

pcl::PointCloud<pcl::PointXYZINormal> lastcloud;
pcl::PointCloud<PointType>::Ptr lastcloud_eff(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr lastcloud_all(new pcl::PointCloud<PointType>());

PointCloudXYZI::Ptr last_pc(new PointCloudXYZI());
ros::Publisher pub_pointcloud, pub_pointcloud_eff, pub_pointcloud_all;
int cur_frame = 0, k = 12;

vector<PointCloudXYZI::Ptr>  last_pcs;      
void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{

    PointCloudXYZI::Ptr cur_pc(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *cur_pc);
    last_pcs.push_back(cur_pc);
    if(last_pcs.size() > k)
    {
        vector<PointCloudXYZI::Ptr>::iterator tmp = last_pcs.begin();
        last_pcs.erase(tmp);
    }


    PointCloudXYZI::Ptr pub_pc(new PointCloudXYZI());
    pcl::PointXYZINormal cur_point;
    for(int i = 0; i < last_pcs.size()-1; i++)
    {
        for(int j = 0; j < last_pcs[i]->size(); j++)
        {
            pub_pc->points.push_back(last_pcs[i]->points[j]);
            pub_pc->points.back().intensity = i*10;
            pub_pc->points.back().curvature = j;
        }
    }

    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(*pub_pc, pcl_ros_msg);
    pcl_ros_msg.header = msg_in->header;
    pub_pointcloud.publish(pcl_ros_msg);
    cur_frame ++;

}

int cur_eff = 0;
void DynCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{


    PointCloudXYZI::Ptr cur_pc(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *cur_pc);
    *lastcloud_eff += *cur_pc;
    cur_eff ++;
    
    
    if(cur_eff == k)
    {
        cur_eff = 0;

        
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*lastcloud_eff, pcl_msg);
        // pcl_ros_msg.header.stamp = ros::Time::now();
        pcl_msg.header.stamp = msg_in->header.stamp;
        pcl_msg.header.frame_id = msg_in->header.frame_id;//"/livox_frame"

        pub_pointcloud_eff.publish(pcl_msg);
 


        lastcloud_eff->clear();
    }
    

    

}

int cur_all = 0;
void AllCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{


    PointCloudXYZI::Ptr cur_pc(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *cur_pc);
    *lastcloud_all += *cur_pc;
    cur_all ++;
    
    
    if(cur_all == k)
    {
        cur_all = 0;

        
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*lastcloud_all, pcl_msg);
        // pcl_ros_msg.header.stamp = ros::Time::now();
        pcl_msg.header.stamp = msg_in->header.stamp;
        pcl_msg.header.frame_id = msg_in->header.frame_id;//"/livox_frame"

        pub_pointcloud_all.publish(pcl_msg);
 


        lastcloud_all->clear();
    }
    

    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "replay_pc");
    ros::NodeHandle nh;

    
    
    
    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = nh.subscribe("cloud_registered", 200000, PointsCallback);

    pub_pointcloud  = nh.advertise<sensor_msgs::PointCloud2>
            ("/last_cloud_registered", 100000);

    ros::Subscriber sub_dyn = nh.subscribe("cloud_effected", 200000, DynCallback);

    pub_pointcloud_eff  = nh.advertise<sensor_msgs::PointCloud2>
            ("/last_cloud_effected", 100000);

    ros::Subscriber sub_all = nh.subscribe("cloud_registered", 200000, AllCallback);

    pub_pointcloud_all  = nh.advertise<sensor_msgs::PointCloud2>
            ("/last_cloud_all", 100000);

    ros::spin();
    return 0;
}
