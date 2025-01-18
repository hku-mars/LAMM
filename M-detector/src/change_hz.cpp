#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>

#include <std_msgs/Header.h>
#include "livox_ros_driver/CustomMsg.h"



typedef pcl::PointXYZINormal PointType;
float max_dis = 20.0;
float resolution = 0.5;
ros::Publisher pub_pcl_out1;
ros::Publisher pub_pcl_out2;
ros::Publisher pub_pcl_out3;
ros::Publisher pub_pcl_out4;
ros::Publisher cluster_vis;
ros::Publisher obs_pos;
ros::Publisher predict_path;

pcl::PointCloud<PointType>::Ptr one_frame(new pcl::PointCloud<PointType>());
int k = 5;
int itr = 0;
int cluster_max = -1;
double delta_t = 0.02;
int t_length = 10;
double dis_thresh = 0.5;

double k_cov = 10;



void SelfSegCallback(const livox_ros_driver::CustomMsgConstPtr& msg_in) 
{

    auto time_end = msg_in->points.back().offset_time;
    for (unsigned int i = 0; i < msg_in->point_num; ++i) 
    {
        PointType pt;
        pt.x = msg_in->points[i].x;
        pt.y = msg_in->points[i].y;
        pt.z = msg_in->points[i].z;
        float s = msg_in->points[i].offset_time / (float)time_end;

        // pt.intensity = livox_msg->points[i].line +livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
        pt.intensity = 100;
        pt.curvature = s*0.1;
        one_frame->push_back(pt);
      
    }

    pcl::PointCloud<PointType> pcl_in;
    PointType pt;
    itr ++;

    if(itr == k)
    {
        itr = 0;

        
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*one_frame, pcl_msg);
        // pcl_ros_msg.header.stamp = ros::Time::now();
        pcl_msg.header.stamp = msg_in->header.stamp;
        pcl_msg.header.frame_id = msg_in->header.frame_id;//"/livox_frame"

        pub_pcl_out4.publish(pcl_msg);
 


        one_frame->clear();
    }

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "cluster_predict");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    nh.param("test/k", k, 1);
    ROS_INFO("start cluster predict");
    // ros::Subscriber sub_livox_msg1 = nh.subscribe<sensor_msgs::PointCloud2>(
    //     "/cloud_registered", 100, SelfSegCallback);// livox_pcl0/low_intensity
    
    ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
        "/livox/lidar", 100, SelfSegCallback);

    pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl1", 100);
    pub_pcl_out2 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl2", 100);
    pub_pcl_out3 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl3", 100);
    pub_pcl_out4 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl4", 100);
    cluster_vis = nh.advertise<visualization_msgs::MarkerArray>("/cluster_vis", 10);
    obs_pos = nh.advertise<visualization_msgs::Marker>("/obstacle_predict", 10);
    predict_path = nh.advertise<visualization_msgs::MarkerArray>("/predict_path", 100);

    // ReadFileAndPlot("/home/huajie/planning/dyn_ws/src/cluster_predict/pics/test_1631843313.204504.csv");
    // ReadFileAndPlot("/home/huajie/planning/dyn_ws/src/cluster_predict/test.txt");
    // ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry> 
    //           ("/mavros/odometry/out", 100, OdomCallback);
    // Test();
    // ReadPCD("/home/dji/huajie_ws/src/06063pcd/1626619441.604700.pcd");
    // std::cout<<"...."<<std::endl;

    // std::vector<double> xs, ys;
    // for(int i=0;i<200;i++)
    // {
    //     xs.push_back(i);
    //     ys.push_back(i*i);
    // }
    // plt::plot(xs, ys);
    // plt::title("test");
    // plt::save(save_path + "/test.png");
    // std::cout<<save_path + "/test.png"<<std::endl;

    ros::spin();
    // ReadFileAndPlot("/home/huajie/planning/dyn_ws/src/cluster_predict/pics/test.csv");
}
