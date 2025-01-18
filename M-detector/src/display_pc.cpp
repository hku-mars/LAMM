
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
#include <unistd.h> 
#include <dirent.h> 
#include <iomanip>

#include "preprocess.h"

using namespace std;

pcl::PointCloud<pcl::PointXYZINormal> lastcloud;
PointCloudXYZI::Ptr last_pc(new PointCloudXYZI());
ros::Publisher pub_pointcloud, pub_marker, pub_iou_view, cluster_gt_pub;
ros::Publisher pub_tp, pub_fp, pub_fn, pub_tn;

string points_topic = "/velodyne_points_revise";
string pc_folder, label_folder, pred_folder, semantic_folder, out_folder, iou_file, odom_file, vel_folder, bin_folder;
int type = 0; //0 for  kitti, 1 for nuscenes

vector<float> pos;

void get_transforms(std::string pose_file)
{
    pos.clear();
    std::fstream pose_input(pose_file.c_str(), std::ios::in | std::ios::binary);
    if(!pose_input.good())
    {
        std::cerr << "Could not read pose file: " << pose_file << std::endl;
        exit(EXIT_FAILURE);
    }

    
    while (!pose_input.eof())
    {
        float cur_pose;
        pose_input.read((char *) &cur_pose, sizeof(float));
        pos.push_back(cur_pose);
    }
    pose_input.close();
}



int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
// int objects_number = 0;
std::vector<int> objects_numbers_kitti(7, 0);
std::vector<float> average_recalls_kitti(7, 0.0);
std::unordered_map<int, std::string> objects_types_map_kitti;

std::vector<int> objects_numbers_nuscenes(11, 0);
std::vector<float> average_recalls_nuscenes(11, 0.0);
std::unordered_map<int, std::string> objects_types_map_nuscenes;
std::unordered_map<int, int> objects_class_map_nuscenes;

std::vector<int> objects_numbers_waymo(3, 0);
std::vector<float> average_recalls_waymo(3, 0.0);
std::unordered_map<int, std::string> objects_types_map_waymo;
// float average_recall = 0.0;
         
float PointsRead(const string &pc_file, const string &label_file, const string &pred_file)
{


    std::fstream pc_input(pc_file.c_str(), std::ios::in | std::ios::binary);
    if(!pc_input.good())
    {
        std::cerr << "Could not read pointcloud file: " << pc_file << std::endl;
        exit(EXIT_FAILURE);
    }
    pc_input.seekg(0, std::ios::beg);

    std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
    if(!label_input.good())
    {
        std::cerr << "Could not read label file: " << label_file << std::endl;
        exit(EXIT_FAILURE);
    }
    label_input.seekg(0, std::ios::beg);

    std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
    if(!pred_input.good())
    {
        std::cerr << "Could not read prediction file: " << pred_file << std::endl;
        exit(EXIT_FAILURE);
    }
    // pred_input.seekg(0, std::ios::beg);

    std::fstream test(label_file.c_str(), std::ios::in | std::ios::binary);
    if(!test.good())
    {
        std::cerr << "Could not read label file: " << label_file << std::endl;
        exit(EXIT_FAILURE);
    }
    test.seekg(0, std::ios::beg);
    int class_id;
    test.read((char *) &class_id, sizeof(int));
    test.read((char *) &class_id, sizeof(int));

    // cl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZINormal>);

    int tp = 0, fn = 0, fp = 0, count = 0;
    float iou = 0.0f;
    float cur_self_vel;
    
    if(test.eof())
    {
        cout<<" empty"<<endl;
        for (int i=0; pc_input.good() && !pc_input.eof(); i++) 
        {
            pcl::PointXYZINormal point;
            pc_input.read((char *) &point.x, 3*sizeof(float));
            pc_input.read((char *) &point.intensity, sizeof(float));

            int pred_num;
            // pred_input.read((char *) &pred_num, sizeof(int));
            // pred_num = pred_num & 0xFFFF;
            
            // if(pred_num >= 251)
            // {
            //     point.intensity = 10; //tp
            // }
            // else
            // {
            //     point.intensity = 20;
            // }
            point.intensity = 10;
            // count += 1;
            
            iou_out->push_back(point);
            // points_out->push_back(point);
        }
    }
    else
    {
        for (int i=0; pc_input.good() && !pc_input.eof(); i++) 
        {
            pcl::PointXYZINormal point;
            pc_input.read((char *) &point.x, 3*sizeof(float));
            pc_input.read((char *) &point.intensity, sizeof(float));
            point.intensity = 0;

            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int));
            // pred_num = pred_num & 0xFFFF;

            int  label_num = -1, id = -1;
            // pred_input >> pred_num;
            label_input.read((char *) &label_num, sizeof(int));
            // label_num = label_num & 0xFFFF;

            if(label_num >= 251 && label_num < 65535)
            {
                if(pred_num >= 251)
                {
                    point.intensity = 10; //tp
                    tp += 1;
                }
                else
                {
                    point.intensity = 20;
                    fn += 1; //
                    // cout<<" , "<<label_num<<" "<<point.x<<" "<<point.y<<" "<<point.z;
                }
                count += 1;
            }
            else
            {
                if(pred_num >= 251)
                {
                    point.intensity = 30; //fp
                    fp += 1;
                }
            }
            point.normal_x = label_num;
            point.normal_y = pred_num;
            iou_out->push_back(point);
            // points_out->push_back(point);
        }
    }
    
    if(tp+fn+fp > 10e-5)iou = ((float)tp)/(float)(tp+fn+fp);
    total_tp += tp;
    total_fn += fn;
    total_fp += fp;
    // label_input.close();
    // vel_input.close();
    pred_input.close();
    cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<<" count: "<<count<<" iou: "<<iou<<endl;
    cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: "<<total_fn<<endl;
    cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

    // sensor_msgs::PointCloud2 pcl_ros_msg;
    // pcl::toROSMsg(*points_out, pcl_ros_msg);
    // pcl_ros_msg.header.frame_id = "camera_init";
    // pcl_ros_msg.header.stamp = ros::Time::now();
    // pub_pointcloud.publish(pcl_ros_msg);

    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*iou_out, pcl_msg);
    pcl_msg.header.frame_id = "camera_init";
    pcl_msg.header.stamp = ros::Time::now();
    pub_iou_view.publish(pcl_msg); 

    total_tp += tp;
    total_fn += fn;
    total_fp += fp;
    
    if (tp+fn+fp < 1)
    {
        iou = -1;
    }
    else 
    {
        iou = ((float)tp)/(float)(tp+fn+fp);
        if(iou > 1) 
        {
            int a;
            cout<<"iou:--------"<<iou<<endl;
            cin>>a;
        }
    }
    pc_input.close();
    label_input.close();
    pred_input.close();
    test.close();
    
    cout<<"tp: "<<tp<<" fn: "<<fn<<" fp: "<<fp<<" count: "<<count<<" iou: " << iou <<endl;
    cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: "<<total_fn<<endl;
    cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

    return iou;
    // marker.text=str.str();
    // marker.pose=pose;
    // pub_marker.publish(marker);
}

int frames = 0, minus_num = 0;

void KittiPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    cout<<"frames: "<<frames<<endl;
    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *points_in);

    if(frames < minus_num)
    {
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_in, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "camera_init";
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);
    }
    else
    {   
        cout << "frame: " << frames << endl;
        string label_file = label_folder;
        stringstream ss;
        ss << setw(6) << setfill('0') << frames;
        label_file += ss.str(); 
        label_file.append(".label");
         std::cout << "label_file: " << label_file << std::endl;

        string pred_file = pred_folder;
        stringstream sss;
        sss << setw(6) << setfill('0') << frames-minus_num ;
        pred_file += sss.str(); 
        pred_file.append(".label");
         std::cout << "pred_file: " << pred_file << std::endl;

        string semantic_file = semantic_folder;
        stringstream ssss;
        ssss << setw(6) << setfill('0') << frames;
        semantic_file += ssss.str(); 
        semantic_file.append(".label");
        ofstream out;

        string out_file = out_folder;
        stringstream s1;
        s1 << setw(6) << setfill('0') << frames;
        out_file += s1.str(); 
        out_file.append(".label");
        out.open(out_file, ios::out  | ios::binary);

        string bin_file = bin_folder;
        stringstream s2;
        s2 << setw(6) << setfill('0') << frames;
        bin_file += s2.str();
        bin_file.append(".bin");
        std::cout << "binfile: " << bin_file << std::endl;
        // ofstream bin_out;
        // bin_out.open(bin_file, ios::out | ios::binary);
        // if(!bin_out.good())
        // {
        //     std::cout << "could not open bin file" << std::endl;
        // }


        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
        if(!label_input.good())
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        label_input.seekg(0, std::ios::beg);

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        std::fstream semantic_input(semantic_file.c_str(), std::ios::in | std::ios::binary);
        if(!semantic_input.good())
        {
            std::cerr << "Could not read semantic file: " << semantic_file << std::endl;
            exit(EXIT_FAILURE);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr tp_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr fp_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr fn_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr tn_out (new pcl::PointCloud<pcl::PointXYZI>);
        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;
        std::cout << "points size: " << points_in->points.size() << endl;
        for (int i=0; i<points_in->points.size(); i++) 
        {
            pcl::PointXYZI point;
            point.x = points_in->points[i].x;
            point.y = points_in->points[i].y;
            point.z = points_in->points[i].z;

            int label_num = -2, pred_num = -1, semantic_num = -1;
            label_input.read((char *) &label_num, sizeof(int));
            label_num = label_num & 0xFFFF;
            // std::cout << "label: " << label_num << std::endl;

            // pred_input >> pred_num;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;
            int k = -1;
            k = k & 0xFFFF;
            // if(pred_num != 9)

            semantic_input.read((char *) &semantic_num, sizeof(int));
            semantic_num = semantic_num & 0xFFFF;

            point.intensity = 0;
            // if((pred_num >= 251 && pred_num < 65535) || (label_num >= 251 && label_num < 65535))
            // {
            //     bin_out.write((char*)&point.x, 3 * sizeof(float));
            //     bin_out.write((char*)&points_in->points[i].intensity, sizeof(float));
            // }
            if(label_num >= 251 && label_num < 65535)
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                else labels_map[label_num] += 1;
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 10; //tp
                    tp += 1;
                    iou_out->push_back(point);
                    tp_out->push_back(point);
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                    else predicts_map[label_num] += 1;
                    // bin_out.write((char*)&point.x, 3 * sizeof(float));
                    // bin_out.write((char*)&points_in->points[i].intensity, sizeof(float));
                }
                else
                {
                    point.intensity = 20;
                    fn += 1; //
                    iou_out->push_back(point);
                    fn_out->push_back(point);
                }
                count += 1;
            }
            else if (label_num < 251 && label_num < 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 30; //fp
                    fp += 1;
                    iou_out->push_back(point);
                    fp_out->push_back(point);
                    // bin_out.write((char*)&point.x, 3 * sizeof(float));
                    // bin_out.write((char*)&points_in->points[i].intensity, sizeof(float));
                }
                else
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 25; //op
                    fp_out->push_back(point);
                    op += 1;
                    iou_out->push_back(point);
                    // bin_out.write((char*)&point.x, 3 * sizeof(float));
                    // bin_out.write((char*)&points_in->points[i].intensity, sizeof(float));
                }
            }
            // cout << "pred_num : " << pred_num << " semantic_num: " << semantic_num << endl;
            if(pred_num == 251)
            {
                if (semantic_num < 40)
                {
                    int tmp = 251;
                    // cout << "here: " <<endl;
                    out.write((char*)&tmp, sizeof(int));
                }
                else
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                }
            }
            else
            {
                int tmp = 9;
                out.write((char*)&tmp, sizeof(int));
            }
            if(point.intensity == 0 || point.intensity == 20) points_out->push_back(point);
        }
        out.close();
        // bin_out.close();
        iou = ((float)tp)/(float)(tp+fn+fp);
        total_tp += tp;
        total_fn += fn;
        total_fp += fp;
        total_tn += tn;
        total_op += op;
        label_input.close();
        pred_input.close();
        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<< " op: " <<op << " tn: " << tn << " count: "<<count<<" iou: "<<iou<<endl;
        for(auto it = labels_map.begin(); it != labels_map.end(); it++)
        {   
            int class_num = it->first/1000;
            objects_numbers_kitti[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_kitti[class_num]  = average_recalls_kitti[class_num] * (objects_numbers_kitti[class_num] - 1)/objects_numbers_kitti[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_kitti[class_num];
        }
        for(int i = 0; i < objects_numbers_kitti.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_kitti[i] << " is: " << average_recalls_kitti[i] << " objects number: " << objects_numbers_kitti[i]<< endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_out, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "camera_init";
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);

        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*iou_out, pcl_msg);
        pcl_msg.header.frame_id = "camera_init";
        pcl_msg.header.stamp = ros::Time::now();
        pub_iou_view.publish(pcl_msg); 


        sensor_msgs::PointCloud2 tp_ros_msg;
        pcl::toROSMsg(*tp_out, tp_ros_msg);
        tp_ros_msg.header.frame_id = "camera_init";
        tp_ros_msg.header.stamp = ros::Time::now();
        pub_tp.publish(tp_ros_msg);

        sensor_msgs::PointCloud2 fp_ros_msg;
        pcl::toROSMsg(*fp_out, fp_ros_msg);
        fp_ros_msg.header.frame_id = "camera_init";
        fp_ros_msg.header.stamp = ros::Time::now();
        pub_fp.publish(fp_ros_msg);

        sensor_msgs::PointCloud2 fn_ros_msg;
        pcl::toROSMsg(*fn_out, fn_ros_msg);
        fn_ros_msg.header.frame_id = "camera_init";
        fn_ros_msg.header.stamp = ros::Time::now();
        pub_fn.publish(fn_ros_msg);

        visualization_msgs::Marker marker;
        marker.header.frame_id="camera_init";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 255;
        marker.color.a = 1;  
        geometry_msgs::Pose pose;
        pose.position.x =  points_out->points[0].x;
        pose.position.y =  points_out->points[0].y;
        pose.position.z =  points_out->points[0].z;
        ostringstream str;
        str<<"tp: "<<tp<<" fn: "<<fn<<" fp: "<<fp<<" count: "<<count<<" iou: "<<iou;
        marker.text=str.str();
        marker.pose=pose;
        pub_marker.publish(marker);
    }
    
    frames ++;
    // pred_input.seekg(0, std::ios::beg);

    
}

void WaymoPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{

    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *points_in);

    if(frames < minus_num)
    {
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_in, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "camera_init";
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);
    }
    else
    {   
        cout << "frame: " << frames << endl;
        string label_file = label_folder;
        stringstream ss;
        ss << setw(6) << setfill('0') << frames;
        label_file += ss.str(); 
        label_file.append(".label");

        string pred_file = pred_folder;
        stringstream sss;
        sss << setw(6) << setfill('0') << frames-minus_num ;
        pred_file += sss.str(); 
        pred_file.append(".label");



        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
        if(!label_input.good())
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        label_input.seekg(0, std::ios::beg);

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);
        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;
        std::cout << "points size: " << points_in->points.size() << endl;
        for (int i=0; i<points_in->points.size(); i++) 
        {
            pcl::PointXYZI point;
            point.x = points_in->points[i].x;
            point.y = points_in->points[i].y;
            point.z = points_in->points[i].z;

            int label_num = 0, pred_num = 0;
            label_input.read((char *) &label_num, sizeof(int));
            label_num = label_num & 0xFFFF;
            // std::cout << "label: " << label_num << std::endl;

            // pred_input >> pred_num;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;
        

            point.intensity = 0;
            if(label_num >= 251 && label_num < 65535)
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                else labels_map[label_num] += 1;
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 10; //tp
                    tp += 1;
                    iou_out->push_back(point);
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                    else predicts_map[label_num] += 1;
                    
                }
                else
                {
                    point.intensity = 20;
                    fn += 1; //
                    iou_out->push_back(point);
                }
                count += 1;
            }
            else if (label_num < 251 && label_num < 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 30; //fp
                    fp += 1;
                    iou_out->push_back(point);
                    
                }
                else
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 25; //op
                    op += 1;
                    iou_out->push_back(point);
                }
            }
            // cout << "pred_num : " << pred_num << " semantic_num: " << semantic_num << endl;
            
            points_out->push_back(point);
        }

        iou = ((float)tp)/(float)(tp+fn+fp);
        total_tp += tp;
        total_fn += fn;
        total_fp += fp;
        total_tn += tn;
        total_op += op;
        label_input.close();
        pred_input.close();
        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<< " op: " <<op << " tn: " << tn << " count: "<<count<<" iou: "<<iou<<endl;
        for(auto it = labels_map.begin(); it != labels_map.end(); it++)
        {   
            int class_num = it->first/1000;
            objects_numbers_waymo[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_waymo[class_num]  = average_recalls_waymo[class_num] * (objects_numbers_waymo[class_num] - 1)/objects_numbers_waymo[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_waymo[class_num];
        }
        for(int i = 0; i < objects_numbers_waymo.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_waymo[i] << " is: " << average_recalls_waymo[i] << " objects number: " << objects_numbers_waymo[i]<< endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_out, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "camera_init";
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);

        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*iou_out, pcl_msg);
        pcl_msg.header.frame_id = "camera_init";
        pcl_msg.header.stamp = ros::Time::now();
        pub_iou_view.publish(pcl_msg); 


        visualization_msgs::Marker marker;
        marker.header.frame_id="camera_init";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 255;
        marker.color.a = 1;  
        geometry_msgs::Pose pose;
        pose.position.x =  points_out->points[0].x;
        pose.position.y =  points_out->points[0].y;
        pose.position.z =  points_out->points[0].z;
        ostringstream str;
        str<<"tp: "<<tp<<" fn: "<<fn<<" fp: "<<fp<<" count: "<<count<<" iou: "<<iou;
        marker.text=str.str();
        marker.pose=pose;
        pub_marker.publish(marker);
    }
    
    frames ++;
    // pred_input.seekg(0, std::ios::beg);

    
}

void AviaPointsCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{   
    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    points_in->resize(msg_in->point_num);
    std::cout << "points size: " << msg_in->point_num << std::endl;
    if(msg_in->point_num == 0) return;
    for(int i = 0; i < msg_in->point_num; i++)
    {
        points_in->points[i].x = msg_in->points[i].x;
        points_in->points[i].y = msg_in->points[i].y;
        points_in->points[i].z = msg_in->points[i].z;
    }

    if(frames < minus_num)
    {
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_in, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "camera_init";
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);
    }
    else
    {   
        cout << "frame: " << frames << endl;
        string label_file = label_folder;
        stringstream ss;
        ss << setw(6) << setfill('0') << frames-minus_num ;
        label_file += ss.str(); 
        label_file.append(".label");

        string pred_file = pred_folder;
        stringstream sss;
        sss << setw(6) << setfill('0') << frames-minus_num ;
        pred_file += sss.str(); 
        pred_file.append(".label");

        string semantic_file = semantic_folder;
        stringstream ssss;
        ssss << setw(6) << setfill('0') << frames;
        semantic_file += ssss.str(); 
        semantic_file.append(".label");
        ofstream out;

        string out_file = out_folder;
        stringstream s1;
        s1 << setw(6) << setfill('0') << frames;
        out_file += s1.str(); 
        out_file.append(".label");
        out.open(out_file, ios::out  | ios::binary);


        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
        if(!label_input.good())
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        label_input.seekg(0, std::ios::beg);

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        std::fstream semantic_input(semantic_file.c_str(), std::ios::in | std::ios::binary);
        if(!semantic_input.good())
        {
            std::cerr << "Could not read semantic file: " << semantic_file << std::endl;
            exit(EXIT_FAILURE);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);
        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;
        for (int i=0; i<points_in->points.size(); i++) 
        {
            pcl::PointXYZI point;
            point.x = points_in->points[i].x;
            point.y = points_in->points[i].y;
            point.z = points_in->points[i].z;

            int label_num = -2, pred_num = -1, semantic_num = -1;
            label_input.read((char *) &label_num, sizeof(int));
            // label_num = label_num & 0xFFFF;
            // std::cout << "label: " << label_num << std::endl;

            // pred_input >> pred_num;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;
            // if(pred_num != 9)

            semantic_input.read((char *) &semantic_num, sizeof(int));
            semantic_num = semantic_num & 0xFFFF;

            point.intensity = 0;
            if(label_num >= 251 && label_num < 65535)
            {   
                if(!labels_map.count(label_num)) 
                {   
                    std::cout << "------------------------------label_num: " << label_num << std::endl;
                    labels_map[label_num] = 1;
                }
                else labels_map[label_num] += 1;
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 10; //tp
                    tp += 1;
                    iou_out->push_back(point);
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                    else predicts_map[label_num] += 1;
                    // bin_out.write((char*)&point.x, 3 * sizeof(float));
                    // bin_out.write((char*)&points_in->points[i].intensity, sizeof(float));
                }
                else
                {
                    point.intensity = 20;
                    fn += 1; //
                    iou_out->push_back(point);
                }
                count += 1;
            }
            else if (label_num < 251 && label_num < 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 30; //fp
                    fp += 1;
                    iou_out->push_back(point);
                    // bin_out.write((char*)&point.x, 3 * sizeof(float));
                    // bin_out.write((char*)&points_in->points[i].intensity, sizeof(float));
                }
                else
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    point.intensity = 25; //op
                    op += 1;
                    iou_out->push_back(point);
                    // bin_out.write((char*)&point.x, 3 * sizeof(float));
                    // bin_out.write((char*)&points_in->points[i].intensity, sizeof(float));
                }
            }
            // cout << "pred_num : " << pred_num << " semantic_num: " << semantic_num << endl;
            if(pred_num == 251)
            {
                if (semantic_num < 40)
                {
                    int tmp = 251;
                    // cout << "here: " <<endl;
                    out.write((char*)&tmp, sizeof(int));
                }
                else
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                }
            }
            else
            {
                int tmp = 9;
                out.write((char*)&tmp, sizeof(int));
            }
            points_out->push_back(point);
        }
        out.close();
        iou = ((float)tp)/(float)(tp+fn+fp);
        total_tp += tp;
        total_fn += fn;
        total_fp += fp;
        total_tn += tn;
        total_op += op;
        label_input.close();
        pred_input.close();
        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<< " op: " <<op << " tn: " << tn << " count: "<<count<<" iou: "<<iou<<endl;
        for(auto it = labels_map.begin(); it != labels_map.end(); it++)
        {   
            int class_num = it->first/1000;
            objects_numbers_kitti[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_kitti[class_num]  = average_recalls_kitti[class_num] * (objects_numbers_kitti[class_num] - 1)/objects_numbers_kitti[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_kitti[class_num];
        }
        for(int i = 0; i < objects_numbers_kitti.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_kitti[i] << " is: " << average_recalls_kitti[i] << " objects number: " << objects_numbers_kitti[i]<< endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_out, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "camera_init";
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);

        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*iou_out, pcl_msg);
        pcl_msg.header.frame_id = "camera_init";
        pcl_msg.header.stamp = ros::Time::now();
        pub_iou_view.publish(pcl_msg); 


        visualization_msgs::Marker marker;
        marker.header.frame_id="camera_init";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 255;
        marker.color.a = 1;  
        geometry_msgs::Pose pose;
        pose.position.x =  points_out->points[0].x;
        pose.position.y =  points_out->points[0].y;
        pose.position.z =  points_out->points[0].z;
        ostringstream str;
        str<<"tp: "<<tp<<" fn: "<<fn<<" fp: "<<fp<<" count: "<<count<<" iou: "<<iou;
        marker.text=str.str();
        marker.pose=pose;
        pub_marker.publish(marker);
    }
    
    frames ++;
    // pred_input.seekg(0, std::ios::beg);

    
}

void NuscenesPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *points_in);
    if(points_in->size() == 0) return;
    minus_num = 0;
    if(frames < minus_num)
    {
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_in, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "camera_init";
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);
    }
    else
    {   
        cout << "frame: " << frames << endl;
        stringstream ss;
        ss << setw(6) << setfill('0') << frames ;
        
        string label_file = label_folder;
        label_file += ss.str(); 
        label_file.append(".label");
        

        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label");

        string bin_file = bin_folder;
        stringstream s2;
        // s2 << setw(6) << setfill('0') << frames;
        // bin_file += s2.str();
        ros::Time cur_time = msg_in->header.stamp;
        string a = std::to_string(cur_time.toNSec());
        s2 << a.substr(0,16);    
        bin_file += s2.str();
        bin_file.append(".pcd.bin");
        std::cout << "binfile: " << bin_file << std::endl;
        ofstream bin_out;
        bin_out.open(bin_file, ios::out | ios::binary);
        if(!bin_out.good())
        {
            std::cout << "could not open bin file" << std::endl;
        }

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
        if(!label_input.good())
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        label_input.seekg(0, std::ios::beg);

        
        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        std::fstream test(label_file.c_str(), std::ios::in | std::ios::binary);
        // if(!test.good())
        // {
        //     std::cerr << "Could not read label file: " << label_file << std::endl;
        //     exit(EXIT_FAILURE);
        // }
        test.seekg(0, std::ios::beg);
        int class_id;
        test.read((char *) &class_id, sizeof(int));
        test.read((char *) &class_id, sizeof(int));

        // cout<<" label_file: "<<label_file<<" , "<< test.eof() << " " << label_input.eof() << endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);
        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;
        float cur_self_vel;
        // if(frames == 0 )
        // {
        //     cur_self_vel = 0.0;
        // }
        // else
        // {
        //     cur_self_vel = pow(pos[3*frames] - pos[3*(frames-1)], 2) + \
        //                         pow(pos[3*frames + 1] - pos[3*(frames-1) + 1], 2) + \
        //                         pow(pos[3*frames + 2] - pos[3*(frames-1) + 2], 2);
        // }
        // cout<<"self vel: "<<cur_self_vel<<"   "<<pos[3*frames]<<" , "<<pos[3*(frames-1)]\
        //     <<"   "<<pos[3*frames+1]<<" , "<<pos[3*(frames-1)+1]<<"   "\
        //     <<pos[3*frames+2]<<" , "<<pos[3*(frames-1)+2]<<endl;
        
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        if(test.eof())
        {
            cout<<" empty"<<endl;
            for (int i=0; i<points_in->points.size(); i++) 
            {
                pcl::PointXYZI point;
                point.x = points_in->points[i].x;
                point.y = points_in->points[i].y;
                point.z = points_in->points[i].z;

                int pred_num;
                // pred_input.read((char *) &pred_num, sizeof(int));
                // pred_num = pred_num & 0xFFFF;
                
                // if(pred_num >= 251)
                // {
                //     point.intensity = 10; //tp
                // }
                // else
                // {
                //     point.intensity = 20;
                // }
                point.intensity = 10;
                // count += 1;
               
                iou_out->push_back(point);
                // points_out->push_back(point);
            }
        }
        else
        {
            for (int i=0; i<points_in->points.size(); i++) 
            {
                pcl::PointXYZI point;
                point.x = points_in->points[i].x;
                point.y = points_in->points[i].y;
                point.z = points_in->points[i].z;
                point.intensity = 0;
                int timestamp = i%32;

                int pred_num = -1;
                pred_input.read((char *) &pred_num, sizeof(int));
                pred_num = pred_num & 0xFFFF;

                int  label_num = -1, id = -1;
                pred_input >> pred_num;
                label_input.read((char *) &label_num, sizeof(int));
                label_num = label_num & 0xFFFF;

                
                if(pred_num >= 251 && pred_num < 65535)
                {
                    bin_out.write((char*)&point.x, 3 * sizeof(float));
                    bin_out.write((char*)&points_in->points[i].intensity, sizeof(float));
                    bin_out.write((char*)&timestamp, sizeof(float));
                }
                // if(label_num >= 251 && label_num < 65535)
                if(label_num >= 1000 && label_num < 65535)
                {
                    // if(label_num == 251) cout<<" --------251: "<<point.x<<" , "<<point.y<<" , "<<point.z<<" "<<pred_num<<endl;
                    if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                    else labels_map[label_num] += 1;
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                        else predicts_map[label_num] += 1;
                        point.intensity = 10; //tp
                        tp += 1;
                        iou_out->push_back(point);
                    }
                    else
                    {
                        point.intensity = 20;
                        fn += 1; //
                        iou_out->push_back(point);
                        // cout<<" , "<<label_num<<" "<<point.x<<" "<<point.y<<" "<<point.z;
                    }
                    count += 1;
                }
                else if(label_num < 251)
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        point.intensity = 30; //fp
                        fp += 1;
                        iou_out->push_back(point);
                    }
                    else
                    {
                        tn += 1;
                    }
                }
                else if (label_num >= 65535)
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    // if(pred_num < 40)
                    {
                        point.intensity = 25; //op
                        op += 1;
                    }
                }
                else if (label_num >= 251 && label_num < 1000)
                {
                    tn += 1;
                }
                // point.normal_x = label_num;
                // point.normal_y = pred_num;
                // iou_out->push_back(point);
                points_out->push_back(point);
            }
        }
        
        bin_out.close();
        if(tp+fn+fp > 10e-5)iou = ((float)tp)/(float)(tp+fn+fp);
        total_tp += tp;
        total_fn += fn;
        total_fp += fp;
        total_tn += tn;
        total_op += op;
        label_input.close();
        // vel_input.close();
        pred_input.close();
        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<<" count: "<<count<<" iou: "<<iou<<endl;
        for(auto it = labels_map.begin(); it != labels_map.end(); it++)
        {   
            int class_num = it->first/1000;
            // if(objects_class_map_nuscenes.count(class_num) == 0)
            // {
            //     cout<<"we do not find "<< class_num << "  " << it->first <<endl;
            //     getchar();
            //     continue;
            // } 
            cout<<"remap class "<< class_num;  
            class_num = objects_class_map_nuscenes[class_num];\
            cout<<" to "<< class_num << endl;
            objects_numbers_nuscenes[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_nuscenes[class_num]  = average_recalls_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1)/objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_nuscenes[class_num];
        }
        for(int i = 0; i < objects_numbers_nuscenes.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_nuscenes[i] << " is: " << average_recalls_nuscenes[i] << " objects number: " << objects_numbers_nuscenes[i]<< endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_out, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = "camera_init";
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);

        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*iou_out, pcl_msg);
        pcl_msg.header.frame_id = "camera_init";
        pcl_msg.header.stamp = ros::Time::now();
        pub_iou_view.publish(pcl_msg); 


        // visualization_msgs::Marker marker;
        // marker.header.frame_id="camera_init";
        // marker.header.stamp = ros::Time::now();
        // marker.ns = "basic_shapes";
        // marker.action = visualization_msgs::Marker::ADD;
        // marker.pose.orientation.w = 1.0;
        // marker.id =0;
        // marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        // marker.scale.z = 0.2;
        // marker.color.b = 0;
        // marker.color.g = 0;
        // marker.color.r = 255;
        // marker.color.a = 1;  
        // geometry_msgs::Pose pose;
        // pose.position.x =  points_out->points[0].x;
        // pose.position.y =  points_out->points[0].y;
        // pose.position.z =  points_out->points[0].z;
        // ostringstream str;
        // str<<"tp: "<<tp<<" fn: "<<fn<<" fp: "<<fp<<" count: "<<count<<" iou: "<<iou;
        // marker.text=str.str();
        // marker.pose=pose;
        // pub_marker.publish(marker);
    }
    
    frames ++;
    // pred_input.seekg(0, std::ios::beg);

    
}

void SemanticCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *points_in);

    stringstream ss;
    ss << setw(6) << setfill('0') << frames;
    string pred_file = pred_folder;
    pred_file += ss.str(); 
    pred_file.append(".label");

    string semantic_file = semantic_folder;
    semantic_file += ss.str(); 
    semantic_file.append(".label");
    
    string out_file = out_folder;
    out_file += ss.str(); 
    out_file.append(".label");
    ofstream out;
    out.open(out_file, ios::out  | ios::binary);


    std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
    if(!pred_input.good())
    {
        std::cerr << "Could not read prediction file: " << pred_file << std::endl;
        exit(EXIT_FAILURE);
    }

    std::fstream semantic_input(semantic_file.c_str(), std::ios::in | std::ios::binary);
    if(!semantic_input.good())
    {
        std::cerr << "Could not read semantic file: " << semantic_file << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "points size: " << points_in->points.size() << endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);
    for (int i=0; i<points_in->points.size(); i++) 
    {
        pcl::PointXYZI point;
        point.x = points_in->points[i].x;
        point.y = points_in->points[i].y;
        point.z = points_in->points[i].z;

        int pred_num = -1, semantic_num = -1;
        pred_input.read((char *) &pred_num, sizeof(int));
        pred_num = pred_num & 0xFFFF;

        semantic_input.read((char *) &semantic_num, sizeof(int));
        semantic_num = semantic_num & 0xFFFF;

        point.intensity = 0;
        if(pred_num == 251)
        {
            if (semantic_num < 40)
            {
                int tmp = 251;
                point.intensity = 10;
                out.write((char*)&tmp, sizeof(int));
            }
            else
            {
                int tmp = 9;
                point.intensity = 20;
                out.write((char*)&tmp, sizeof(int));
            }
        }
        else
        {
            int tmp = 9;
            point.intensity = 30;
            out.write((char*)&tmp, sizeof(int));
        }
        iou_out->push_back(point);
    }
    out.close();
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*iou_out, pcl_msg);
    pcl_msg.header.frame_id = "camera_init";
    pcl_msg.header.stamp = ros::Time::now();
    pub_iou_view.publish(pcl_msg); 
    
    frames ++;
}

void bbox_cbk(const visualization_msgs::MarkerArray::ConstPtr &msg_in)
{   
    visualization_msgs::MarkerArray clusters_front = *msg_in;
    for (int i = 0; i < clusters_front.markers.size(); i++)
    {
        clusters_front.markers[i].color.a = 1.0;
        clusters_front.markers[i].color.r = 1.0;
        clusters_front.markers[i].color.g = 0.0;
        clusters_front.markers[i].color.b = 0.0;
    }
    cluster_gt_pub.publish(clusters_front);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "display_pc");
    ros::NodeHandle nh;

    nh.param<string>("dyn_obj/pc_file", pc_folder,"");
    nh.param<string>("dyn_obj/label_file", label_folder,"");
    nh.param<string>("dyn_obj/pred_file", pred_folder,"");
    nh.param<string>("dyn_obj/vel_file", vel_folder,"");
    nh.param<string>("dyn_obj/odom_file", odom_file,"");
    nh.param<string>("dyn_obj/semantic_file", semantic_folder,"");
    nh.param<string>("dyn_obj/pred_semantic_file", out_folder,"");
    nh.param<string>("dyn_obj/bin_folder", bin_folder,"");
    nh.param<string>("common/lid_topic", points_topic, "/velodyne_points");
    nh.param<int>("dyn_obj/dataset", type, 0);

    
    objects_types_map_kitti[0] = "Person";
    objects_types_map_kitti[1] = "Truck";
    objects_types_map_kitti[2] = "Car";
    objects_types_map_kitti[3] = "Tram";
    objects_types_map_kitti[4] = "Pedestrain";
    objects_types_map_kitti[5] = "Cyclist";
    objects_types_map_kitti[6] = "Van";

    objects_types_map_nuscenes[0] = "Animal";
    objects_types_map_nuscenes[1] = "Pedestrian";
    objects_types_map_nuscenes[2] = "Movable_object";
    objects_types_map_nuscenes[3] = "Bicycle";
    objects_types_map_nuscenes[4] = "Bus";
    objects_types_map_nuscenes[5] = "Car";
    objects_types_map_nuscenes[6] = "Emergency";
    objects_types_map_nuscenes[7] = "Motorcycle";
    objects_types_map_nuscenes[8] = "Trailer";
    objects_types_map_nuscenes[9] = "Truck";
    objects_types_map_nuscenes[10] = "Ego";
    objects_class_map_nuscenes[1] = 0;
    objects_class_map_nuscenes[2] = 1;
    objects_class_map_nuscenes[3] = 1;
    objects_class_map_nuscenes[4] = 1;
    objects_class_map_nuscenes[5] = 1;
    objects_class_map_nuscenes[6] = 1;
    objects_class_map_nuscenes[7] = 1;
    objects_class_map_nuscenes[8] = 1;
    objects_class_map_nuscenes[9] = 2;
    objects_class_map_nuscenes[10] = 2;
    objects_class_map_nuscenes[11] = 2;
    objects_class_map_nuscenes[12] = 2;
    objects_class_map_nuscenes[14] = 3;
    objects_class_map_nuscenes[15] = 4;
    objects_class_map_nuscenes[16] = 4;
    objects_class_map_nuscenes[17] = 5;
    objects_class_map_nuscenes[18] = 6;
    objects_class_map_nuscenes[19] = 6;
    objects_class_map_nuscenes[20] = 6;
    objects_class_map_nuscenes[21] = 7;
    objects_class_map_nuscenes[22] = 8;
    objects_class_map_nuscenes[23] = 9;
    objects_class_map_nuscenes[31] = 10;


    objects_types_map_waymo[0] = "Vehicle";
    objects_types_map_waymo[1] = "Pedestrian";
    objects_types_map_waymo[2] = "Cyclist";

    int all_num = 0;
    if(pc_folder != "")
    {
        DIR* dir;	
        std::cout << "pc_folder: " << pc_folder << std::endl;
        dir = opendir(pc_folder.c_str());
        struct dirent* ptr;
        while((ptr = readdir(dir)) != NULL)
        {
            if(ptr->d_name[0] == '.') {continue;}
            all_num++;
        }
        closedir(dir);
    }

    cout<<"size: "<<all_num<<endl;

    // int label_num = 0;
    // DIR* label_dir;	
    // label_dir = opendir(label_folder.c_str());
    // struct dirent* label_ptr;
    // while((label_ptr = readdir(label_dir)) != NULL)
    // {
    //     if(label_ptr->d_name[0] == '.') {continue;}
    //     label_num++;
    // }
    // closedir(label_dir);

    // int pred_num = 0;
    // DIR* pred_dir;	
    // pred_dir = opendir(pred_folder.c_str());
    // struct dirent* pred_ptr;
    // while((pred_ptr = readdir(pred_dir)) != NULL)
    // {
    //     if(pred_ptr->d_name[0] == '.') {continue;}
    //     pred_num++;
    // }
    // closedir(pred_dir);

    // minus_num = label_num - pred_num;
    minus_num = 0;
    /*** ROS subscribe initialization ***/
    // ros::Subscriber sub_pcl = nh.subscribe("cloud_registered", 200000, PointsCallback);

    // 

    pub_pointcloud  = nh.advertise<sensor_msgs::PointCloud2>
            ("/result_view", 100000);
    pub_marker = nh.advertise<visualization_msgs::Marker>("text_view", 10);
    pub_iou_view = nh.advertise<sensor_msgs::PointCloud2>
            ("/iou_view", 100000);
    pub_tp = nh.advertise<sensor_msgs::PointCloud2>
            ("/tp_view", 100000);
    pub_fp = nh.advertise<sensor_msgs::PointCloud2>
            ("/fp_view", 100000);
    pub_fn = nh.advertise<sensor_msgs::PointCloud2>
            ("/fn_view", 100000);
    cluster_gt_pub = nh.advertise<visualization_msgs::MarkerArray>
            ("cluster_vis_gt_gl", 100000);
    
    int cur_frame = 1;
    int i = 0;
    int avia_index = 0;
    float total_iou = 0.0;
    while(false)//i<all_num
    {   
        std::cout << "i: " << i << std::endl;
        string pc_name = pc_folder;
        stringstream ss;
        ss << setw(6) << setfill('0') << i ;
        pc_name += ss.str(); 
        pc_name.append(".bin");

        string label_name = label_folder;
        label_name += ss.str(); 
        label_name.append(".label");

        string pred_name = pred_folder;
        pred_name += ss.str(); 
        pred_name.append(".label");

        cout<<pc_name<<endl;
        cout<<label_name<<endl;
        cout<<pred_name<<endl;
        float iou = PointsRead(pc_name, label_name, pred_name);
        // // total_iou += iou / all_num;
        i++;
        // if (iou > -0.01)
        // {
        //     total_iou = total_iou * avia_index / (avia_index + 1) + iou / (avia_index + 1);
        //     // total_iou += iou;
        //     avia_index ++;
        // }
        // sleep(0.1);
        getchar();
        // cin >> cur_frame;
    }
    cout << "avia_frames: " << avia_index << endl; 
    cout << "Average iou: " << total_iou << ", " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;
    cout << "total tp: " << total_tp << "total fn: " << total_fn << "total fp: " << total_fp << endl;
    ros::Subscriber sub_pcl;
    if(type == 0 )
    {
        sub_pcl = nh.subscribe(points_topic, 200000, KittiPointsCallback);
    }
    else if(type == 2)
    {
        sub_pcl = nh.subscribe(points_topic, 200000, WaymoPointsCallback);
    }
    else if(type == 1)
    {
        // get_transforms(odom_file);
        // cout<<"----------size: "<<pos.size()<<endl;
        sub_pcl = nh.subscribe(points_topic, 200000, NuscenesPointsCallback);
    }
    else if(type == 3)
    {   
        sub_pcl = nh.subscribe(points_topic, 200000, AviaPointsCallback);
    }
    else if(type == -1)
    {   
        sub_pcl = nh.subscribe(points_topic, 200000, SemanticCallback);
    }
    ros::Subscriber sub_bb = nh.subscribe("/cluster_vis_gt", 100, bbox_cbk);
    cout<<points_topic<<"type : "<<type<<endl;
    ros::spin();
    return 0;
}