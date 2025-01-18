#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>

#include "DynObjFilter.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <unistd.h> 
#include <dirent.h> 
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>



using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

pcl::PointCloud<pcl::PointXYZINormal> lastcloud;
PointCloudXYZI::Ptr last_pc(new PointCloudXYZI());
ros::Publisher pub_pointcloud, pub_marker, pub_iou_view;


string dataset_folder, pred_folder, pred_nc_folder, recall_folder, recall_nc_folder, label_folder, recall_file, recall_nc_file;
string semantic_folder, out_folder;
ofstream recall_rec, recall_nc_rec;
// Eigen::Vector3d threshold;
// threshold[0] = 0.1;
// threshold[1] = 0.1;
// threshold[2] = 0.1;
// threshold << 0.1, 0.1, 0.1;   
// rosbag::Bag bag;
// bag.open("/home/huajie/event_detection/bag/test.bag", rosbag::bagmode::Write);



int total_tp_nc = 0, total_fn_nc = 0, total_fp_nc = 0, total_op_nc = 0, total_tn_nc = 0.0;
// int objects_number = 0;
// std::vector<int> objects_numbers_kitti(7, 0);
// std::vector<float> average_recalls_kitti(7, 0.0);
std::unordered_map<int, std::string> objects_types_map_kitti;



std::unordered_map<int, std::string> objects_types_map_nuscenes;
std::unordered_map<int, int> objects_class_map_nuscenes;

// std::vector<int> objects_numbers_waymo(3, 0), objects_numbers_waymo_nc(3, 0);
// std::vector<float> average_recalls_waymo(3, 0.0), average_recalls_waymo_nc(3, 0.0);
std::unordered_map<int, std::string> objects_types_map_waymo;

void Init()
{
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
}

void NuscenesCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    int label_file_num = 0;
    if(label_folder != "")
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str());
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL)
        {
            if(label_ptr->d_name[0] == '.') {continue;}
            label_file_num++;
        }
        closedir(label_dir);
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }
    // int pred_nc_num = 0;
    // if(pred_nc_folder != "")
    // {
    //     DIR* pred_nc_dir;	
    //     pred_nc_dir = opendir(pred_nc_folder.c_str());
    //     struct dirent* pred_nc_ptr;
    //     while((pred_nc_ptr = readdir(pred_nc_dir)) != NULL)
    //     {
    //         if(pred_nc_ptr->d_name[0] == '.') {continue;}
    //         pred_nc_num++;
    //     }
    //     closedir(pred_nc_dir);
    // } 
    // else
    // {
    //     cout<<"less pred nc "<<pred_nc_folder<<endl;
    //     return;
    // }
    
    if(label_file_num != pred_file_num)// || pred_num != pred_nc_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    std::vector<int> objects_numbers_nuscenes(11, 0), objects_numbers_nuscenes_nc(11, 0);
    std::vector<float> average_recalls_nuscenes(11, 0.0), average_recalls_nuscenes_nc(11, 0.0);
    std::vector<float> average_recalls2_nuscenes(11, 0.0);
    for(int frames = 0; frames < label_file_num; frames ++)
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

        // string pred_nc_file = pred_nc_folder;
        // pred_nc_file += ss.str(); 
        // pred_nc_file.append(".label");

        

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

        // std::fstream pred_nc_input(pred_nc_file.c_str(), std::ios::in | std::ios::binary);
        // if(!pred_nc_input.good())
        // {
        //     std::cerr << "Could not read prediction file: " << pred_nc_file << std::endl;
        //     exit(EXIT_FAILURE);
        // }
        
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

        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;
        float cur_self_vel;

        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        if(test.eof())
        {
            cout<<" empty"<<endl;
        }
        else
        {
            while(pred_input.good() && !pred_input.eof())
            {
                

                int pred_num = -1;
                pred_input.read((char *) &pred_num, sizeof(int));
                pred_num = pred_num & 0xFFFF;

                int  label_num = -1, id = -1;
                // pred_input >> pred_num;
                label_input.read((char *) &label_num, sizeof(int));
                label_num = label_num & 0xFFFF;

                

                if(label_num >= 1000 && label_num < 65535)
                {
                    // if(label_num == 251) cout<<" --------251: "<<point.x<<" , "<<point.y<<" , "<<point.z<<" "<<pred_num<<endl;
                    if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                    else labels_map[label_num] += 1;
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                        else predicts_map[label_num] += 1;
                        tp += 1;
                    }
                    else
                    {
                        fn += 1; //
                        // cout<<" , "<<label_num<<" "<<point.x<<" "<<point.y<<" "<<point.z;
                    }
                    count += 1;
                }
                else if(label_num < 251)
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        fp += 1;
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
                        op += 1;
                    }
                }
                else if (label_num >= 251 && label_num < 1000)
                {
                    tn += 1;
                }

            }
        }
        
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
            if(objects_class_map_nuscenes.count(class_num) == 0)
            {
                cout<<"we do not find "<< class_num << "  " << it->first <<endl;
                continue;
            } 
            cout<<"remap class "<< class_num;  
            class_num = objects_class_map_nuscenes[class_num];\
            cout<<" to "<< class_num << endl;
            objects_numbers_nuscenes[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_nuscenes[class_num]  = average_recalls_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1)/objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_nuscenes[class_num];
            average_recalls2_nuscenes[class_num] = average_recalls2_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1)/objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first]/(float)it->second * (float)predicts_map[it->first]/(float)it->second/ objects_numbers_nuscenes[class_num];        
            if(class_num == 4 || class_num == 5 || class_num == 7 || class_num == 9) recalls_vehicle.push_back((float)predicts_map[it->first]/(float)it->second);
            if(class_num == 1) recalls_pedestrian.push_back((float)predicts_map[it->first]/(float)it->second);
            if(class_num == 3) recalls_cyclist.push_back((float)predicts_map[it->first]/(float)it->second);
        }
        for(int i = 0; i < objects_numbers_nuscenes.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_nuscenes[i] << " is: " << average_recalls_nuscenes[i] << " objects number: " << objects_numbers_nuscenes[i] << " variance: " << average_recalls2_nuscenes[i] - average_recalls_nuscenes[i] * average_recalls_nuscenes[i]<< endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;
        if((frames + 1) == label_file_num )
        {
            class_nums.clear();
            class_recalls.clear();
            class_var.clear();
            for(int i = 0; i < objects_numbers_nuscenes.size(); i++)
            {
                class_nums.push_back(objects_numbers_nuscenes[i]);
                class_recalls.push_back(average_recalls_nuscenes[i]);
                class_var.push_back(average_recalls2_nuscenes[i] - average_recalls_nuscenes[i] * average_recalls_nuscenes[i]);
            }
        }
    }
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
    // pred_input.seekg(0, std::ios::beg);
}

void WaymoCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist, int seq)
{
    int label_file_num = 0;
    if(label_folder != "")
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str());
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL)
        {
            if(label_ptr->d_name[0] == '.') {continue;}
            label_file_num++;
        }
        closedir(label_dir);
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }
    // int pred_nc_num = 0;
    // if(pred_nc_folder != "")
    // {
    //     DIR* pred_nc_dir;	
    //     pred_nc_dir = opendir(pred_nc_folder.c_str());
    //     struct dirent* pred_nc_ptr;
    //     while((pred_nc_ptr = readdir(pred_nc_dir)) != NULL)
    //     {
    //         if(pred_nc_ptr->d_name[0] == '.') {continue;}
    //         pred_nc_num++;
    //     }
    //     closedir(pred_nc_dir);
    // } 
    // else
    // {
    //     cout<<"less pred nc "<<pred_nc_folder<<endl;
    //     return;
    // }
    
    if(label_file_num != pred_file_num)// || pred_num != pred_nc_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    std::vector<int> objects_numbers_waymo(3, 0);
    std::vector<float> average_recalls_waymo(3, 0.0);
    std::vector<float> average_recalls2_waymo(3, 0.0);
    for(int frames = 0; frames < label_file_num; frames ++)
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

        // string pred_nc_file = pred_nc_folder;
        // pred_nc_file += ss.str(); 
        // pred_nc_file.append(".label");

        

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

        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;

        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        

        while(pred_input.good() && !pred_input.eof())
        {
            

            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;

            int  label_num = -1, id = -1;
            // pred_input >> pred_num;
            label_input.read((char *) &label_num, sizeof(int));
            label_num = label_num & 0xFFFF;

            

            if(label_num >= 251 && label_num < 65535)
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                else labels_map[label_num] += 1;
                if(pred_num >= 251 && pred_num < 65535)
                {
                    tp += 1;
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                    else predicts_map[label_num] += 1;
                    
                }
                else
                {
                    fn += 1; //
                }
                count += 1;
            }
            else if (label_num < 251 && label_num < 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1;               
                }
                else
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    op += 1;
                }
            }

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
            average_recalls2_waymo[class_num] = average_recalls2_waymo[class_num] * (objects_numbers_waymo[class_num] - 1)/objects_numbers_waymo[class_num] + (float)predicts_map[it->first]/(float)it->second * (float)predicts_map[it->first]/(float)it->second/ objects_numbers_waymo[class_num];
            if(seq != 11 && seq !=19 && seq !=30 && seq != 32 && seq != 41 && seq !=46)
            {
                if(class_num == 0) recalls_vehicle.push_back((float)predicts_map[it->first]/(float)it->second);
                if(class_num == 1) recalls_pedestrian.push_back((float)predicts_map[it->first]/(float)it->second);
                if(class_num == 2) recalls_cyclist.push_back((float)predicts_map[it->first]/(float)it->second);
            }
        }
        for(int i = 0; i < objects_numbers_waymo.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_waymo[i] << " is: " << average_recalls_waymo[i] << " objects number: " << objects_numbers_waymo[i] << " variance: " << average_recalls2_waymo[i] - average_recalls_waymo[i] * average_recalls_waymo[i] << endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;
        
        if((frames + 1) == label_file_num )
        {
            class_nums.clear();
            class_recalls.clear();
            class_var.clear();
            for(int i = 0; i < objects_numbers_waymo.size(); i++)
            {
                class_nums.push_back(objects_numbers_waymo[i]);
                class_recalls.push_back(average_recalls_waymo[i]);
                class_var.push_back(average_recalls2_waymo[i] - average_recalls_waymo[i] * average_recalls_waymo[i]);
            }
        }
    }
    
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
    // pred_input.seekg(0, std::ios::beg);
}

void KittiCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    int label_file_num = 0;
    if(label_folder != "")
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str());
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL)
        {
            if(label_ptr->d_name[0] == '.') {continue;}
            label_file_num++;
        }
        closedir(label_dir);
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }
    // int pred_nc_num = 0;
    // if(pred_nc_folder != "")
    // {
    //     DIR* pred_nc_dir;	
    //     pred_nc_dir = opendir(pred_nc_folder.c_str());
    //     struct dirent* pred_nc_ptr;
    //     while((pred_nc_ptr = readdir(pred_nc_dir)) != NULL)
    //     {
    //         if(pred_nc_ptr->d_name[0] == '.') {continue;}
    //         pred_nc_num++;
    //     }
    //     closedir(pred_nc_dir);
    // } 
    // else
    // {
    //     cout<<"less pred nc "<<pred_nc_folder<<endl;
    //     return;
    // }
    
    if(label_file_num != pred_file_num)// || pred_num != pred_nc_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    std::vector<int> objects_numbers_kitti(7, 0);
    std::vector<float> average_recalls_kitti(7, 0.0);
    std::vector<float> average_recalls2_kitti(7, 0.0);
    for(int frames = 0; frames < label_file_num; frames ++)
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

        // string pred_nc_file = pred_nc_folder;
        // pred_nc_file += ss.str(); 
        // pred_nc_file.append(".label");

        

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

        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;

        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        

        while(pred_input.good() && !pred_input.eof())
        {
            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;

            int  label_num = -1, id = -1;
            // pred_input >> pred_num;
            label_input.read((char *) &label_num, sizeof(int));
            label_num = label_num & 0xFFFF;

            

            if(label_num >= 251 && label_num < 65535)
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                else labels_map[label_num] += 1;
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    tp += 1;
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                    else predicts_map[label_num] += 1;
                }
                else if (pred_num == 9)
                {
                    fn += 1; //
                }
                count += 1;
            }
            else if (label_num < 251 && label_num < 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1;
                }
                else if (pred_num == 9)
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    op += 1;
            
                }
            }

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
            objects_numbers_kitti[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_kitti[class_num]  = average_recalls_kitti[class_num] * (objects_numbers_kitti[class_num] - 1)/objects_numbers_kitti[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_kitti[class_num];
            average_recalls2_kitti[class_num] = average_recalls2_kitti[class_num] * (objects_numbers_kitti[class_num] - 1)/objects_numbers_kitti[class_num] + (float)predicts_map[it->first]/(float)it->second * (float)predicts_map[it->first]/(float)it->second/ objects_numbers_kitti[class_num];
            if(class_num == 1 || class_num == 2 || class_num == 3 || class_num == 6) recalls_vehicle.push_back((float)predicts_map[it->first]/(float)it->second);
            if(class_num == 4) recalls_pedestrian.push_back((float)predicts_map[it->first]/(float)it->second);
            if(class_num == 6) recalls_cyclist.push_back((float)predicts_map[it->first]/(float)it->second);
        }
        for(int i = 0; i < objects_numbers_kitti.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_kitti[i] << " is: " << average_recalls_kitti[i] << " objects number: " << objects_numbers_kitti[i] << " variance: " << average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i] << endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

        
        if((frames + 1) == label_file_num )
        {
            class_nums.clear();
            class_recalls.clear();
            class_var.clear();
            for(int i = 0; i < objects_numbers_kitti.size(); i++)
            {
                class_nums.push_back(objects_numbers_kitti[i]);
                class_recalls.push_back(average_recalls_kitti[i]);
                class_var.push_back(average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i]);
            }
        }
    }
    
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
    // pred_input.seekg(0, std::ios::beg);
}

void AviaCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, int seq)
{
    int label_file_num = 0;
    if(label_folder != "")
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str());
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL)
        {
            if(label_ptr->d_name[0] == '.') {continue;}
            label_file_num++;
        }
        closedir(label_dir);
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }
    // int pred_nc_num = 0;
    // if(pred_nc_folder != "")
    // {
    //     DIR* pred_nc_dir;	
    //     pred_nc_dir = opendir(pred_nc_folder.c_str());
    //     struct dirent* pred_nc_ptr;
    //     while((pred_nc_ptr = readdir(pred_nc_dir)) != NULL)
    //     {
    //         if(pred_nc_ptr->d_name[0] == '.') {continue;}
    //         pred_nc_num++;
    //     }
    //     closedir(pred_nc_dir);
    // } 
    // else
    // {
    //     cout<<"less pred nc "<<pred_nc_folder<<endl;
    //     return;
    // }
    
    if(label_file_num != pred_file_num)// || pred_num != pred_nc_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    std::vector<int> objects_numbers_kitti(7, 0);
    std::vector<float> average_recalls_kitti(7, 0.0);
    std::vector<float> average_recalls2_kitti(7, 0.0);
    for(int frames = 0; frames < label_file_num; frames ++)
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

        // string pred_nc_file = pred_nc_folder;
        // pred_nc_file += ss.str(); 
        // pred_nc_file.append(".label");

        

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

        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;

        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        

        while(pred_input.good() && !pred_input.eof())
        {
            

            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;

            int  label_num = -1, id = -1;
            // pred_input >> pred_num;
            label_input.read((char *) &label_num, sizeof(int));
            label_num = label_num & 0xFFFF;

            

            if(label_num >= 251 && label_num < 65535)
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                else labels_map[label_num] += 1;
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    tp += 1;
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                    else predicts_map[label_num] += 1;
                }
                else if (pred_num == 9)
                {
                    fn += 1; //
                }
                count += 1;
            }
            else if (label_num < 251 && label_num < 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1;
                }
                else if (pred_num == 9)
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    op += 1;
            
                }
            }

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
            objects_numbers_kitti[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_kitti[class_num]  = average_recalls_kitti[class_num] * (objects_numbers_kitti[class_num] - 1)/objects_numbers_kitti[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_kitti[class_num];
            average_recalls2_kitti[class_num] = average_recalls2_kitti[class_num] * (objects_numbers_kitti[class_num] - 1)/objects_numbers_kitti[class_num] + (float)predicts_map[it->first]/(float)it->second * (float)predicts_map[it->first]/(float)it->second/ objects_numbers_kitti[class_num];
            if(seq == 10 || seq == 11 || seq == 12 || seq ==13 || seq == 20 || seq ==21 || seq== 45) recalls_pedestrian.push_back((float)predicts_map[it->first]/(float)it->second);
            else if (seq != 22) recalls_vehicle.push_back((float)predicts_map[it->first]/(float)it->second);
        }
        for(int i = 0; i < objects_numbers_kitti.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_kitti[i] << " is: " << average_recalls_kitti[i] << " objects number: " << objects_numbers_kitti[i]<< " variance: " << average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i] << endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

        
        if((frames + 1) == label_file_num )
        {
            class_nums.clear();
            class_recalls.clear();
            class_var.clear();
            for(int i = 0; i < objects_numbers_kitti.size(); i++)
            {
                class_nums.push_back(objects_numbers_kitti[i]);
                class_recalls.push_back(average_recalls_kitti[i]);
                class_var.push_back(average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i]);
            }
        }
    }
    
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
}

void SemanticCombine()
{
    int semantic_file_num = 0;
    if(semantic_folder != "")
    {
        DIR* semantic_dir;	
        semantic_dir = opendir(semantic_folder.c_str());
        struct dirent* semantic_ptr;
        while((semantic_ptr = readdir(semantic_dir)) != NULL)
        {
            if(semantic_ptr->d_name[0] == '.') {continue;}
            semantic_file_num++;
        }
        closedir(semantic_dir);
    } 
    else
    {
        cout<<"less semantic "<<semantic_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }

    
    if(semantic_file_num != pred_file_num)// || pred_num != pred_nc_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    for(int frames = 0; frames < semantic_file_num; frames++)
    {
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

        while(pred_input.good() && !pred_input.eof())
        {
            int pred_num = -1, semantic_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;

            semantic_input.read((char *) &semantic_num, sizeof(int));
            semantic_num = semantic_num & 0xFFFF;

            if(pred_num == 251)
            {
                if (semantic_num < 40)
                {
                    int tmp = 251;
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
        }
        out.close();

    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_dynamic");
    ros::NodeHandle nh;
    Init();

    int se_num = 0, cal_num = 0, dataset = -1;
    bool is_nc = false;
    nh.param<int>("dyn_obj/dataset", dataset, -1);
    nh.param<int>("dyn_obj/se_num", se_num, 0);
    nh.param<int>("dyn_obj/cal_num", cal_num, 0);
    nh.param<bool>("dyn_obj/is_nc", is_nc, false);
    nh.param<string>("dyn_obj/dataset_folder", dataset_folder,"/");

    // nh.param<string>("dyn_obj/dataset_folder", pred_folder,"/");
    // nh.param<string>("dyn_obj/dataset_folder", semantic_folder,"/");
    // nh.param<string>("dyn_obj/dataset_folder", out_folder,"/");

    // nh.param<double>("max_vel_vehicle", max_vel_vehicle, 1.0);
    // nh.param<double>("max_vel_pedestrian", max_vel_pedestrian, 1.0);
    // nh.param<double>("box_z_thr", box_z_thr, 0.1);
    // cout<<"topic  "<<points_topic<<endl;


    if(dataset == 0)
    {
        recall_folder = dataset_folder + "recall/";
        recall_nc_folder = dataset_folder + "recall_nc/";
        string command;
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_nc_folder;
        system(command.c_str());
        std::vector<string> cal_num_vec;
        // cal_num_vec.push_back(4);
        // cal_num_vec.push_back(5);
        // cal_num_vec.push_back(6);
        cal_num_vec.push_back("7_static");
        cal_num_vec.push_back("7_all");
        // cal_num_vec.push_back(8);
        // cal_num_vec.push_back(9);
        // cal_num_vec.push_back(12);
        // cal_num_vec.push_back(13);
        // cal_num_vec.push_back(7);
        // cal_num_vec.push_back(8);
        // cal_num_vec.push_back(5);
        // cal_num_vec.push_back(14);
        // cal_num_vec.push_back(15);

        for(int i = 0; i < cal_num_vec.size(); i++ )
        {
            if(is_nc)
            {
                recall_file = recall_nc_folder + "kitti" + cal_num_vec[i] + + "_nc" + ".txt";
            }
            else
            {   
                recall_file = recall_folder + "kitti" + cal_num_vec[i] + ".txt";
            }
            // recall_nc_file = recall_nc_folder + "nuscenes" + to_string(i) + + "_nc" + ".txt";
            recall_rec.open(recall_file, std::ios::out);
            // recall_nc_rec.open(recall_nc_file, std::ios::out);
            std::vector<float> recalls_vehicle;
            std::vector<float> recalls_pedestrian;
            std::vector<float> recalls_cyclist;
            for(int j = 0;  j < se_num; j++)
            {   
                if(j==1) continue;
                stringstream ss;
                ss << setw(4) << setfill('0') << j ;
                if(is_nc)
                {
                    pred_folder = dataset_folder + "predict_points_" + cal_num_vec[i] + "/" + ss.str() + "_origin" + "/";
                }
                else
                {   
                    // if (cal_num_vec[i] >= 12) pred_folder = dataset_folder + "predict_points_" + to_string(cal_num_vec[i]) + "_withground_combineorigin/" + ss.str() + "/predictions/" ;
                    pred_folder = dataset_folder + "predict_points_" + cal_num_vec[i] + "/" + ss.str() + "/" ;
                }
                
                // pred_nc_folder = dataset_folder + "kitti" + to_string(i) + "_nc/" + ss.str() + "/predictions/";
                label_folder = dataset_folder + "label_points_recall/" + ss.str() + "/";
                cout<<"pred: "<<pred_folder<<endl;
                cout<<label_folder<<endl;
                KittiCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist);
            }
            int q1_vehicle = round(recalls_vehicle.size() / 8);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 7 / 8);

            int q1_pedestrian = round(recalls_pedestrian.size() / 8);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 7 / 8);

            int q1_cyclist = round(recalls_cyclist.size() / 8);
            int q2_cyclist = round(recalls_cyclist.size() / 2);
            int q3_cyclist = round(recalls_cyclist.size() * 7 / 8);
            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            std::sort(recalls_cyclist.begin(), recalls_cyclist.end());
            recall_rec << recalls_vehicle.at(q1_vehicle) <<  " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            recall_rec << recalls_pedestrian.at(q1_pedestrian) <<  " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            recall_rec << recalls_cyclist.at(q1_cyclist) <<  " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl;  
     
            // recall_nc_rec.close();
            recall_rec.close();
            // getchar();
        }
    }
    if(dataset == 1)
    {
        recall_folder = dataset_folder + "recall/";
        recall_nc_folder = dataset_folder + "recall_nc/";
        string command;
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_nc_folder;
        system(command.c_str());

        // for(int i = 0; i < cal_num; i++ )
        for(int i = 8; i < 9; i++ )
        {
            if(is_nc)
            {
                recall_file = recall_nc_folder + "nuscenes" + to_string(i) + "_nc" + ".txt";
            }
            else
            {
                recall_file = recall_folder + "nuscenes" + to_string(i) + ".txt";
            }
            // recall_nc_file = recall_nc_folder + "nuscenes" + to_string(i) + + "_nc" + ".txt";
            recall_rec.open(recall_file, std::ios::out);
            std::vector<float> recalls_vehicle;
            std::vector<float> recalls_pedestrian;
            std::vector<float> recalls_cyclist;
            // recall_nc_rec.open(recall_nc_file, std::ios::out);
            for(int j = 0;  j < se_num; j++)
            {
                stringstream ss;
                ss << setw(2) << setfill('0') << j ;
                stringstream ss1;
                ss1 << setw(4) << setfill('0') << j ;
                if(is_nc)
                {
                    pred_folder = dataset_folder + "nuscenes" + to_string(i) + "_nc/" + ss1.str() + "/predictions/";
                }
                else
                {
                    pred_folder = dataset_folder + "nuscenes" + to_string(i) + "/" + ss1.str() + "/predictions/";
                }
                
                pred_nc_folder = dataset_folder + "nuscenes" + to_string(i) + "_nc/" + ss1.str() + "/predictions/";
                label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/";
                cout<<"pred: "<<pred_folder<<endl;
                cout<<label_folder<<endl;
                NuscenesCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist);
            }
            int q1_vehicle = round(recalls_vehicle.size() / 4);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

            int q1_pedestrian = round(recalls_pedestrian.size() / 4);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

            int q1_cyclist = round(recalls_cyclist.size() / 4);
            int q2_cyclist = round(recalls_cyclist.size() / 2);
            int q3_cyclist = round(recalls_cyclist.size() * 3 / 4);
            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            std::sort(recalls_cyclist.begin(), recalls_cyclist.end());
            recall_rec << recalls_vehicle.at(q1_vehicle) <<  " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            recall_rec << recalls_pedestrian.at(q1_pedestrian) <<  " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            recall_rec << recalls_cyclist.at(q1_cyclist) <<  " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl;  
     
            // recall_nc_rec.close();
            recall_rec.close();
            // getchar();
        }
    }
    if(dataset == 2)
    {
        recall_folder = dataset_folder + "recall/";
        recall_nc_folder = dataset_folder + "recall_nc/";
        string command;
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_nc_folder;
        system(command.c_str());

        for(int i = 6; i < cal_num; i++ )
        {
            if(is_nc)
            {
                recall_file = recall_nc_folder + "waymo" + to_string(i) + "_nc" + ".txt";
            }
            else
            {
                recall_file = recall_folder + "waymo" + to_string(i) + ".txt";
            }
            // recall_nc_file = recall_nc_folder + "nuscenes" + to_string(i) + + "_nc" + ".txt";
            recall_rec.open(recall_file, std::ios::out);
            std::vector<float> recalls_vehicle;
            std::vector<float> recalls_pedestrian;
            std::vector<float> recalls_cyclist;
            // recall_nc_rec.open(recall_nc_file, std::ios::out);
            for(int j = 0;  j < se_num; j++)
            {
                stringstream ss;
                ss << setw(4) << setfill('0') << j ;
                if(is_nc)
                {
                    pred_folder = dataset_folder + "waymo" + to_string(i) + "_nc/" + ss.str() + "/predictions/";
                }
                else
                {
                    pred_folder = dataset_folder + "waymo_" + to_string(i) + "/" + ss.str() + "/";
                }
                
                pred_nc_folder = dataset_folder + "waymo" + to_string(i) + "_nc/" + ss.str() + "/predictions/";
                label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/";
                cout<<"pred: "<<pred_folder<<endl;
                cout<<label_folder<<endl;
                WaymoCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist, j);
            }
            int q1_vehicle = round(recalls_vehicle.size() / 4);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

            int q1_pedestrian = round(recalls_pedestrian.size() / 4);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

            int q1_cyclist = round(recalls_cyclist.size() / 4);
            int q2_cyclist = round(recalls_cyclist.size() / 2);
            int q3_cyclist = round(recalls_cyclist.size() * 3 / 4);
            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            std::sort(recalls_cyclist.begin(), recalls_cyclist.end());
            recall_rec << recalls_vehicle.at(q1_vehicle) <<  " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            recall_rec << recalls_pedestrian.at(q1_pedestrian) <<  " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            recall_rec << recalls_cyclist.at(q1_cyclist) <<  " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl; 
     
            // recall_nc_rec.close();
            recall_rec.close();
            // getchar();
        }
    }
    if(dataset == 3)
    {
        recall_folder = dataset_folder + "recall/";
        recall_nc_folder = dataset_folder + "recall_nc/";
        string command;
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_nc_folder;
        system(command.c_str());
        std::vector<int> cal_num_vec;
        // cal_num_vec.push_back(0);
        cal_num_vec.push_back(0);
        // cal_num_vec.push_back(2);
        // cal_num_vec.push_back(1);
        cal_num_vec.push_back(4);
        // cal_num_vec.push_back(5);
        // cal_num_vec.push_back(6);
        // cal_num_vec.push_back(10);
        // cal_num_vec.push_back(11);
        // cal_num_vec.push_back(1000);

        for(int i = 0; i < cal_num_vec.size(); i++ )
        {
            if(is_nc)
            {
                recall_file = recall_nc_folder + "avia" + to_string(cal_num_vec[i]) + + "_nc" + ".txt";
            }
            else
            {
                recall_file = recall_folder + "avia" + to_string(cal_num_vec[i]) + ".txt";
            }
            // recall_nc_file = recall_nc_folder + "nuscenes" + to_string(i) + + "_nc" + ".txt";
            recall_rec.open(recall_file, std::ios::out);
            // recall_nc_rec.open(recall_nc_file, std::ios::out);
            std::vector<float> recalls_vehicle;
            std::vector<float> recalls_pedestrian;
            for(int j = 0;  j < se_num; j++)
            {   
                if(j==22) continue;
                stringstream ss;
                ss << setw(2) << setfill('0') << j ;
                stringstream ssss;
                ssss << setw(4) << setfill('0') << j ;
                if(is_nc)
                {
                    pred_folder = dataset_folder + ss.str() + "/" + "avia" + to_string(cal_num_vec[i]) + "_origin" + "/predictions/";
                }
                else
                {
                    pred_folder = dataset_folder + ss.str() + "/" + "avia" + to_string(cal_num_vec[i]) + "/predictions/";
                }
                
                // pred_nc_folder = dataset_folder + "kitti" + to_string(i) + "_nc/" + ss.str() + "/predictions/";
                label_folder = dataset_folder + ss.str() + "/" + "labels/";
                cout<<"pred: "<<pred_folder<<endl;
                cout<<label_folder<<endl;
                AviaCalRecall(recalls_vehicle, recalls_pedestrian, j);
            }
            int q1_vehicle = round(recalls_vehicle.size() / 4);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

            int q1_pedestrian = round(recalls_pedestrian.size() / 4);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            recall_rec << recalls_vehicle.at(q1_vehicle) <<  " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            recall_rec << recalls_pedestrian.at(q1_pedestrian) <<  " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            // recall_nc_rec.close();
            recall_rec.close();
            // getchar();
        }
    }
    if(dataset == -1)
    {
        string all_pred = dataset_folder + "residual_1/";
        string all_semantic = dataset_folder + "semantic/";
        string all_out = dataset_folder + "residual_1_semantic/";
        vector<string> seq_names;
        if(dataset_folder != "")
        {
            DIR* pred_dir;	
            pred_dir = opendir(all_pred.c_str());
            struct dirent* pred_ptr;
            while((pred_ptr = readdir(pred_dir)) != NULL)
            {
                if(pred_ptr->d_name[0] == '.') {continue;}
                string cur_folder(pred_ptr->d_name);
                seq_names.push_back(cur_folder);
            }
            closedir(pred_dir);
        }
        
        for(int i = 0; i < seq_names.size(); i++ )
        {       
            pred_folder = all_pred + seq_names[i] + "/predictions/";
            semantic_folder = all_semantic + seq_names[i] + "/predictions/";
            out_folder = all_out + seq_names[i] + "/predictions/";
            cout<<pred_folder<<endl;
            cout<<semantic_folder<<endl;
            cout<<out_folder<< " " <<i <<endl;
            SemanticCombine();
            // for(int j = 0;  j < se_num; j++)
            // {   
            //     if(j==1) continue;
            //     stringstream ss;
            //     ss << setw(4) << setfill('0') << j ;
            //     if(is_nc)
            //     {
            //         pred_folder = dataset_folder + "predict_points_" + to_string(cal_num_vec[i]) + "/" + ss.str() + "_origin" + "/";
            //     }
            //     else
            //     {
            //         pred_folder = dataset_folder + "predict_points_" + to_string(cal_num_vec[i]) + "/" + ss.str() + "/";
            //     }
                
            //     // pred_nc_folder = dataset_folder + "kitti" + to_string(i) + "_nc/" + ss.str() + "/predictions/";
            //     label_folder = dataset_folder + "label_points_recall/" + ss.str() + "/";
            //     cout<<"pred: "<<pred_folder<<endl;
            //     cout<<label_folder<<endl;
            //     KittiCalRecall();
            // }
        }
    }

    ros::spin();
    return 0;
}