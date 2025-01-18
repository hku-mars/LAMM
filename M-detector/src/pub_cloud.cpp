#include <ros/ros.h>
#include "std.h"
#include "std_ba.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_cloud");
    ros::NodeHandle nh;

    // 读取pcd文件路径参数
    std::string pcd_file_path;
    int if_rgb;
    float ds_size;
    nh.param<int>("if_rgb", if_rgb, 0);
    nh.param<float>("ds_size", ds_size, 0.5);
    nh.param<std::string>("pcd_file", pcd_file_path, "");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>());

    // 加载pcd文件
    if (if_rgb){

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file_path, *cloud_rgb) == -1)
        {
            ROS_ERROR_STREAM("Failed to load pcd file: " << pcd_file_path);
            return -1;
        }
    } else{

        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_file_path, *cloud) == -1)
        {
            ROS_ERROR_STREAM("Failed to load pcd file: " << pcd_file_path);
            return -1;
        }
    }




    // 创建PointCloud2消息
    sensor_msgs::PointCloud2 cloud_msg;
    if (if_rgb){
        pcl::toROSMsg(*cloud_rgb, cloud_msg);
    } else{
        down_sampling_voxel(*cloud, ds_size);
        pcl::toROSMsg(*cloud, cloud_msg);
    }
    cloud_msg.header.frame_id = "camera_init"; // 设置坐标系

    // 创建Publisher发布PointCloud2消息
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_published", 1, true);

    // 发布PointCloud2消息
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        cloud_msg.header.stamp = ros::Time::now();
        pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}