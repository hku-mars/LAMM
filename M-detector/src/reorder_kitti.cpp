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
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tinycolormap.hpp>

using namespace cv;

ros::Publisher lidar_pub;
double fov_left = -0.3;
double fov_right = 0.3;
double fov_up = -0.05;
double fov_down = 0.35;
double resolution = 0.004;
double rotate_angle_unit = 0.008;
double hor_resolution_max = 0.001;
double ver_resolution_max = 0.001;
const char* source_window = "Source image";
int frame = 0;
int circle_size = 2;
void rgb(double ratio, Scalar &RGB)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 6);

    //find the region for this position
    int region = normalized / 256;

    //find the distance to the start of the closest region
    int x = normalized % 256;

    uint8_t r = 0, g = 0, b = 0;
    switch (region)
    {
    case 0: r = 255; g = 0;   b = 0;   g += x; break;
    case 1: r = 255; g = 255; b = 0;   r -= x; break;
    case 2: r = 0;   g = 255; b = 0;   b += x; break;
    case 3: r = 0;   g = 255; b = 255; g -= x; break;
    case 4: r = 0;   g = 0;   b = 255; r += x; break;
    case 5: r = 255; g = 0;   b = 255; b -= x; break;
    }
    RGB << r, g, b;
}

void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
	// Mat hh;
	// hh = *(Mat*)userdata;
	// Point p(x, y);
	// switch (EVENT)
	// {
	// case EVENT_LBUTTONDOWN:
	// {
	// 	printf("b=%d\t", hh.at<Vec3b>(p)[0]);
	// 	printf("g=%d\t", hh.at<Vec3b>(p)[1]);
	// 	printf("r=%d\n", hh.at<Vec3b>(p)[2]);
	// 	circle(hh, p, 2, Scalar(255), 3);
	// }
	// break;
	// }
    cout<<"x: "<< x << "y: "<< y << endl;
}


void lidar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in) 
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(*msg_in));
    pcl::PointCloud<pcl::PointXYZI> current_points;
    pcl::PointCloud<pcl::PointXYZI> reorder_points;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> points_by_line;
    points_by_line.resize(100);
    pcl::fromROSMsg(*msg, current_points);
    bool jump_flag = false;
    int line_index = 1;
    int line_index_change = 0;
    double last_horizon_angle =1.0;
    for (int i = 0; i < current_points.size(); i++)
    {   
        V3D p_body(current_points.points[i].x, current_points.points[i].y, current_points.points[i].z);
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
    }

    float current_angle = fov_left;
    float rotate_angle = rotate_angle_unit;
    int k = 0;
    // int hor_range = (fov_right + (fov_right - fov_left)/resolution * rotate_angle_unit - fov_left) / hor_resolution_max + floor((fov_down - fov_up) / ver_resolution_max)/6;
    int hor_range = (fov_right + (fov_right - fov_left)/resolution * rotate_angle_unit - fov_left) / hor_resolution_max;
    int ver_range = (fov_down- fov_up) / ver_resolution_max;
    cv::Mat color = Mat::zeros(Size(hor_range, ver_range), CV_8UC3);
    // color.setTo(255);
    while(current_angle < fov_right)
    {
        for (int i = 1; i < 66; i++)
        {   
            // if (i%2 == 0 ) continue;
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
                    // point.intensity = k;
                    point.intensity = range;
                    reorder_points.push_back(point);
                    int hor_ind   = floor((-horizon_angle + fov_right + (fov_right - fov_left)/resolution * rotate_angle_unit) / hor_resolution_max);
                    int ver_ind   = floor((-vertical_angle - fov_up) / ver_resolution_max);
                    // hor_ind += ver_ind/6;
                    Scalar point_rgb;
                    // if(range> 50) range = 50;

                    const tinycolormap::Color color_rgb = tinycolormap::GetColor(range/80., tinycolormap::ColormapType::Jet);
                    // rgb(range/80, point_rgb);
                    point_rgb << color_rgb.r() * 255, color_rgb.g()*255, color_rgb.b()*255;
                    cv::circle(color, cv::Point(hor_ind,ver_ind), circle_size, point_rgb, -1);
                    break;
                }
            }
        }
        current_angle += resolution;
        k ++;
        rotate_angle += rotate_angle_unit;
    }
    stringstream ss;
    ss << frame;
    string str = ss.str();
    // imwrite("/home/yihang/Pictures/0" + str + ".png", color);
    namedWindow( source_window, WINDOW_NORMAL);
    imshow( source_window, color);
    cv::setMouseCallback(source_window, on_mouse, 0);
    waitKey(1000);

    sensor_msgs::PointCloud2 point_pub;
    pcl::toROSMsg(reorder_points, point_pub);
    point_pub.header.stamp = msg_in->header.stamp;
    point_pub.header.frame_id = "camera_init";
    lidar_pub.publish(point_pub);
    frame++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reorder_kitti");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/velodyne_points_revise", 1000, lidar_cbk);
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    nh.param<double>("fov_left",fov_left, -0.5);
    nh.param<double>("fov_right",fov_right, 0.5);
    nh.param<double>("resolution",resolution,0.004);
    nh.param<double>("rotate_angle_unit",rotate_angle_unit,0.008);
    ros::spin();
    return 0;
}