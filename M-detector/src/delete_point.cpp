#include <ros/ros.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloudXYZI;

int main(int argc, char **argv) {
    ros::init(argc, argv, "std_loop");
    ros::NodeHandle nh;

    PointCloudXYZI::Ptr dyn_pcd(new pcl::PointCloud<pcl::PointXYZINormal>);
    PointCloudXYZI::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    std::string pcd_file1 = "/home/weihairuo/bag/pcd_correct/scan.pcd";
    std::string pcd_file2 = "/home/weihairuo/bag/pcd_correct/dyn.pcd";
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_file1, *save_cloud) != -1 && pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_file2, *dyn_pcd) != -1) {
        std::cout << "dyn points: " << dyn_pcd->points.size() << std::endl;
        std::cout << "save points: " << save_cloud->points.size() << std::endl;

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*save_cloud, *save_cloud, indices); // 移除 NaN 点

        result_cloud->clear();
        for(int i = 0; i < save_cloud->points.size(); i++){
//            std::cout << "i: " << i << std::endl;
            bool found = false;
            for(int j = 0; j < dyn_pcd->points.size(); j++){
                if(save_cloud->points[i].x == dyn_pcd->points[j].x && save_cloud->points[i].y == dyn_pcd->points[j].y && save_cloud->points[i].z == dyn_pcd->points[j].z){
                    found = true;
                    break;
                }
            }
            if(!found){
                result_cloud->push_back(save_cloud->points[i]);
            }
        }

        std::cout << "result points: " << result_cloud->points.size() << std::endl;
        std::string all_points_dir("/home/weihairuo/bag/pcd_correct/result.pcd");
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *result_cloud);
    }

    return 0;
}