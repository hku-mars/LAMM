#include <typeinfo>
#include "boost/range.hpp"
#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <omp.h>
#include <mutex>
#include <math.h>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <Eigen/Core>
#include <types.h>
#include "DynObjFilter.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/filters/random_sample.h>

#include "preprocess.h"

using namespace std;

ros::Publisher pub_pointcloud;

void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    cout<<"get points"<<endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr points_all(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg_in, *points_all);
    pcl::PointCloud<pcl::PointXYZI>::Ptr feats_undistort(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i=0; i<points_all->size(); i++)
    {
        // if(i%20==0)
        {
            feats_undistort->points.push_back(points_all->points[i]);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());

	//创建随机采样一致性对象    
    // 平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZI> seg;
        // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType(pcl::SACMODEL_PLANE);//设置模型类型
//                SACMODEL_PLANE, 三维平面
//                SACMODEL_LINE,    三维直线
//                SACMODEL_CIRCLE2D, 二维圆
//                SACMODEL_CIRCLE3D,  三维圆
//                SACMODEL_SPHERE,      球
//                SACMODEL_CYLINDER,    柱
//                SACMODEL_CONE,        锥
//                SACMODEL_TORUS,       环面
//                SACMODEL_PARALLEL_LINE,   平行线
//                SACMODEL_PERPENDICULAR_PLANE, 垂直平面
//                SACMODEL_PARALLEL_LINES,  平行线
//                SACMODEL_NORMAL_PLANE,    法向平面
//                SACMODEL_NORMAL_SPHERE,   法向球
//                SACMODEL_REGISTRATION,
//                SACMODEL_REGISTRATION_2D,
//                SACMODEL_PARALLEL_PLANE,  平行平面
//                SACMODEL_NORMAL_PARALLEL_PLANE,   法向平行平面
//                SACMODEL_STICK
    seg.setMethodType(pcl::SAC_RANSAC);//设置随机采样一致性方法类型
        // you can modify the parameter below
    seg.setMaxIterations(1000);//设置最大迭代次数
    seg.setDistanceThreshold(0.1);//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
    seg.setInputCloud(feats_undistort);
        //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) 
    {
        cout << "error! Could not found any inliers!" << endl;
    }
	// 从点云中抽取分割的处在平面上的点集
    pcl::ExtractIndices<pcl::PointXYZI> extractor;//点提取对象
    extractor.setInputCloud(feats_undistort);
    extractor.setIndices(inliers);
    //true表示的是输入点集以外的点
    extractor.setNegative(true);
    extractor.filter(*result);
    // 从点云中抽取分割的处在平面上的点集
    // pcl::ExtractIndices<pcl::PointXYZI> extractor1;//点提取对象
    // extractor1.setInputCloud(cloudPtrv);
    // extractor1.setIndices(inliers);
    //     //false表示的是输入点集的点
    // extractor1.setNegative(false);
    // extractor1.filter(*ground_cloud);
    // cout << "filter done." << endl;
    cout<<"coef: "<<coefficients<<endl;
    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(*result, pcl_ros_msg);
    pcl_ros_msg.header = msg_in->header;
    pub_pointcloud.publish(pcl_ros_msg);

}

int main (int argc, char **argv) 
{
    ros::init (argc, argv, "pcl_create"); 
    ros::NodeHandle nh; 

    ros::Subscriber sub_pcl = nh.subscribe("/velodyne_points_revise", 200000, PointsCallback);

    pub_pointcloud  = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_without_plane", 100000);


    ros::spin();
    return 0;
    

}