#include <cluster_predict/classification.h>
// #include "KM_assignment.h"
#include <cluster_predict/hungarian.h>
#include <cluster_predict/bbox.h>

#include <math.h>

Classifier::Classifier()
{
    delta_t_ = 0.02;
    t_length_ = 50;
    dis_thresh_ = 0.5;
    max_clusters_ = 0;
    file_path_ = "";
    show_num_cur_ = 0;
    show_num_obs_ = 0;
}

void Classifier::SetParam(ros::NodeHandle& nh)
{
    std::string n = "";
    nh.param("dyn/single_time", delta_t_, 0.02);
    nh.param("dyn/time_frames", t_length_, 20);
    nh.param("dyn/dyn_dist", dis_thresh_, 15.0);
    nh.param("dyn/save_path", file_path_, n);

}

Classifier::Classifier(double delta_t, int t_length, double dis_thresh)
    : delta_t_(delta_t), t_length_(t_length), dis_thresh_(dis_thresh)
{
    max_clusters_ = 0;
    show_num_cur_ = 0;
    show_num_obs_ = 0;
    file_path_ = "";
}

void Classifier::SetFile(std::string file_path)
{
    file_path_ = file_path;
}

void Classifier::SetTime(double delta_t, int t_length)
{
    delta_t_ = delta_t;
    t_length_ = t_length;
}

void Classifier::SetThresh(double dis_thresh)
{
    dis_thresh_ = dis_thresh;
}

void Classifier::GetCurrentPredict()
{
    // std::cout<<"predict clusters "<<maintain_clusters_.size()<<std::endl;
    maintain_predict_.clear();
    
    
    for(int i=0;i<maintain_clusters_.size();i++)
    {
        std::vector<geometry_msgs::PoseWithCovarianceStamped> predictions_of_i;
        maintain_tracks_[i].GetPredictions(predictions_of_i);
        int j=0;
        for(;j<predictions_of_i.size();j++)
        {
            if(fabs(current_time_.toSec() - predictions_of_i[j].header.stamp.toSec())<=5*10E-3)
            {
                std::cout << "The " << j << " prediction is get" << std::endl;
                maintain_predict_.push_back(predictions_of_i[j]);
                std::cout << "Its position is " << predictions_of_i[i].pose.pose.position.x << " " << predictions_of_i[i].pose.pose.position.y << " " << predictions_of_i[i].pose.pose.position.z << std::endl;
                break;
            }
        }
        if(j==predictions_of_i.size())
        {
            std::cout<<"no prediction!!!!"<<std::endl;
            geometry_msgs::PoseWithCovarianceStamped wrong;
            wrong.header.frame_id = "wrong";
            maintain_predict_.push_back(wrong);
        }
    }
}

void Classifier::GetCorMatrix()
{
    int n = std::max(maintain_clusters_.size(), current_clusters_.size());
    // std::cout<<"maintain clusters: "<<maintain_clusters_.size()<< \
    //             "current clusters: "<<current_clusters_.size()<<std::endl;
    // correlations_ = Eigen::MatrixXd::Zero(maintain_clusters_.size(), current_clusters_.size());
    correlations_ = Eigen::MatrixXd::Zero(n, n);
    // std::cout<<"new length: "<<current_clusters_.size()<<" maintain length: "<<maintain_clusters_.size()<<std::endl;
    for(int i=0;i<current_clusters_.size();i++)
    {
        for(int j=0;j<maintain_clusters_.size();j++)
        {
            correlations_(i,j) = CalCorrelation_oobb(current_clusters_[i], maintain_predict_[j]);

            std::cout << "AABB IoU between i: " << i << " and j: " << j << " is " << CalCorrelation_aabb(current_clusters_[i], maintain_predict_[j]) << std::endl;
            std::cout << "OOBB IoU between i: " << i << " and j: " << j << " is " << CalCorrelation_oobb(current_clusters_[i], maintain_predict_[j]) << std::endl;
        }
    }
    // std::ofstream out_file;
    // if(!(file_path_ == ""))
    // {     
        
    //     for(int i=0;i<current_clusters_.size();i++)
    //     {
    //         out_file<<"current "<<i<<" : "<< current_clusters_[i].pose.pose.position.x<<"  "<<
    //                     current_clusters_[i].pose.pose.position.y<<"  "<<
    //                     current_clusters_[i].pose.pose.position.z<<std::endl;
    //     }
        
    //     out_file<<correlations_<<std::endl<<std::endl<<std::endl;
    // }
    
}

double Classifier::CalCorrelation_aabb(geometry_msgs::PoseWithCovarianceStamped &current, \
                                geometry_msgs::PoseWithCovarianceStamped &predict)
{
    
    // double x_mean = predict.pose.pose.position.x;
    // double y_mean = predict.pose.pose.position.y;
    // double z_mean = predict.pose.pose.position.z;
    // double x = current.pose.pose.position.x;
    // double y = current.pose.pose.position.y;
    // double z = current.pose.pose.position.z;
    // double x_cov = predict.pose.covariance[0*6+0] + current.pose.covariance[0*6+0];
    // double y_cov = predict.pose.covariance[1*6+1] + current.pose.covariance[1*6+1];
    // double z_cov = predict.pose.covariance[2*6+2] + current.pose.covariance[2*6+2];
    // double e_pow = pow((x-x_mean),2)/(x_cov) + pow((y-y_mean),2)/(y_cov) + pow((z-z_mean),2)/(z_cov);//
    // double prob = sqrt(pow(PI*2, -1)/(x_cov*y_cov))*exp(-0.5*e_pow);//*z_cov
    // if(prob > 0.0003)
    // {
    //     if(fabs(z-z_mean)<1.0)
    //         return prob*100 - fabs(z-z_mean)*10;
    // }

    // double dis = pow((predict.pose.pose.position.x - current.pose.pose.position.x),2)\
                      + pow((predict.pose.pose.position.y - current.pose.pose.position.y),2)\
                      + pow((predict.pose.pose.position.z - current.pose.pose.position.z),2);
    // double size_dis = fabs(predict.pose.covariance[3*6+3] - current.pose.covariance[3*6+3]) + \
    //                   fabs(predict.pose.covariance[4*6+4] - current.pose.covariance[4*6+4]) + \
    //                   fabs(predict.pose.covariance[5*6+5] - current.pose.covariance[5*6+5]);
    // std::cout<<"raw x: "<<current.pose.pose.position.x<<" raw y: "<<current.pose.pose.position.y<< \
    //          " raw z: "<<current.pose.pose.position.z<<std::endl;
    // std::cout<<"test x: "<<predict.pose.pose.position.x<<" test y: "<<predict.pose.pose.position.y<< \
    //          " test z: "<<predict.pose.pose.position.z<<std::endl;
    // if(dis_thresh_*dis_thresh_ > dis)
    // {
        // return (1 - size_dis)*10 + (500 - e_pow);
    // }
    // return 0;

    Eigen::Vector3f size_current(current.pose.covariance[3*6+3],current.pose.covariance[4*6+4],current.pose.covariance[5*6+5]);
    Eigen::Vector3f size_predict(predict.pose.covariance[3*6+3],predict.pose.covariance[4*6+4],predict.pose.covariance[5*6+5]);
    Eigen::Vector3f min_current(current.pose.covariance[3*6+2], current.pose.covariance[4*6+3], current.pose.covariance[5*6+4]);
    Eigen::Vector3f max_current(current.pose.covariance[2*6+3], current.pose.covariance[3*6+4], current.pose.covariance[4*6+5]);
    Eigen::Vector3f min_predict(predict.pose.covariance[3*6+2], predict.pose.covariance[4*6+3], predict.pose.covariance[5*6+4]);
    Eigen::Vector3f max_predict(predict.pose.covariance[2*6+3], predict.pose.covariance[3*6+4], predict.pose.covariance[4*6+5]);

    Eigen::Vector3f inter_length;
    for (int i = 0; i < 3; i++)
    {   
        float total_length = std::max(max_current[i], max_predict[i]) - std::min(min_current[i], min_predict[i]);
        inter_length[i] = size_current[i] + size_predict[i] - total_length;
    }
    if (inter_length[0] > 0 && inter_length[1] > 0 && inter_length[2] > 0)
    {
        float volume_inter = inter_length[0] * inter_length[1] * inter_length[2];
        float volume_total = size_current[0] * size_current[1] * size_current[2] + size_predict[0] * size_predict[1] * size_predict[2] - volume_inter;
        return volume_inter / volume_total;
    }
    else
    {
        return 0;
    }

    // Eigen::Matrix3d cov;
    // cov << predict.pose.covariance[0*6+0], 0, 0,
    //        0, predict.pose.covariance[1*6+1], 0,
    //        0, 0, predict.pose.covariance[2*6+2];
    // Eigen::Vector3d pos_diff;
    // pos_diff << current.pose.pose.position.x - predict.pose.pose.position.x,
    //             current.pose.pose.position.y - predict.pose.pose.position.y,
    //             current.pose.pose.position.z - predict.pose.pose.position.z;
    // double dis = pos_diff.transpose()*cov.inverse()*pos_diff;
    // std::cout<<"dis: "<<dis<<std::endl; 
    // return 1000-dis;
    

    // double cover_volume = (cover_x_max - cover_x_min) * (cover_y_max - cover_y_min) * \
    //                       (cover_z_max - cover_z_min);
    // double all_volume = (cur_x_max - cur_x_min) * (cur_y_max - cur_y_min) * \
    //                     (cur_z_max - cur_z_min) + (pre_x_max - pre_x_min) * \
    //                     (pre_y_max - pre_y_min) * (pre_z_max - pre_z_min) - cover_volume;

    // // std::cout<<"get update"<<std::endl;
    // return cover_volume/all_volume*100;
    
}

double Classifier::CalCorrelation_oobb(geometry_msgs::PoseWithCovarianceStamped &current, \
                                geometry_msgs::PoseWithCovarianceStamped &predict)
{
    Eigen::Vector3f size_current(current.pose.covariance[3*6+3],current.pose.covariance[4*6+4],current.pose.covariance[5*6+5]);
    Eigen::Vector3f size_predict(predict.pose.covariance[3*6+3],predict.pose.covariance[4*6+4],predict.pose.covariance[5*6+5]);
    // Eigen::Vector3f min_current(current.pose.covariance[3*6+2], current.pose.covariance[4*6+3], current.pose.covariance[5*6+4]);
    // Eigen::Vector3f max_current(current.pose.covariance[2*6+3], current.pose.covariance[3*6+4], current.pose.covariance[4*6+5]);
    // Eigen::Vector3f min_predict(predict.pose.covariance[3*6+2], predict.pose.covariance[4*6+3], predict.pose.covariance[5*6+4]);
    // Eigen::Vector3f max_predict(predict.pose.covariance[2*6+3], predict.pose.covariance[3*6+4], predict.pose.covariance[4*6+5]);
    Eigen::Vector3f center_current_g(current.pose.pose.position.x, current.pose.pose.position.y, current.pose.pose.position.z);
    Eigen::Vector3f center_predict_g(predict.pose.pose.position.x, predict.pose.pose.position.y, predict.pose.pose.position.z);
    Eigen::Quaternionf q_current(current.pose.pose.orientation.w, current.pose.pose.orientation.x, current.pose.pose.orientation.y, current.pose.pose.orientation.z);
    Eigen::Quaternionf q_predict(predict.pose.pose.orientation.w, predict.pose.pose.orientation.x, predict.pose.pose.orientation.y, predict.pose.pose.orientation.z);
    Eigen::Matrix3f R_current(q_current);
    Eigen::Matrix3f R_predict(q_predict);
    // Eigen::Matrix3f R;
    // R.setIdentity();
    Eigen::Vector3f min_current = R_current.transpose() * center_current_g - 0.5f * size_current;
    Eigen::Vector3f max_current = R_current.transpose() * center_current_g + 0.5f * size_current;
    Eigen::Vector3f min_predict = R_predict.transpose() * center_predict_g - 0.5f * size_predict;
    Eigen::Vector3f max_predict = R_predict.transpose() * center_predict_g + 0.5f * size_predict;
    
    bbox_s box_pre(min_predict, max_predict, R_predict);
    bbox_s box_cur(min_current, max_current, R_current);
    bbox_iou iou(box_pre, box_cur);
    return iou.Iou_calculation();
}

void Classifier::MatchClusters()
{
    GetCorMatrix();
    matched_clusters_.clear();
    int n = std::max(maintain_clusters_.size(), current_clusters_.size());
    // int n = 3;
    
    // std::vector<int> match_indices<n, -1>;
    
    
    hungarian<int> solver(current_clusters_.size(), maintain_clusters_.size());
    // hungarian<int> solver(n, n);
    // Eigen::Matrix<int, 3, 3> test;
    // test << 15, 12, 8,
    //         14, 6, 8,
    //         13, 12, 10;
    for(int i = 0; i < n; i++)
    {
        // std::vector<int> temp;
        for(int j = 0; j < n; j++)
        {
            // temp.push_back((int)(cor(i,j)*10000));
            solver.addEdge(i,j,(int)(correlations_(i,j)*10000));
            // std::cout<<"add edge "<<i<<" "<<j<<std::endl;
            // solver.addEdge(i,j,test(i,j));
        }
        // c.push_back(temp);
    }

    // KMAlgorithm::maxCompatibilitySum(n, correlations_, match_indices);
    solver.solve();
    solver.GetSolution(current_clusters_.size(), matched_clusters_);
    for (int i = 0; i < matched_clusters_.size(); i++)
    {
        std::cout << "The mathced cluster of " << i << " is "<< matched_clusters_[i] << std::endl;
    }
    // for(int i=0;i<matched_clusters_.size();i++)
    // {
    //     // matched_clusters_.push_back(match_indices[i]);
    //     std::cout<<i<<" matched index: "<<matched_clusters_[i]<<std::endl;
    // }
}

void Classifier::UpdateClusters()
{
    std::ofstream out_file;
    if(!(file_path_ == ""))
    {     
        out_file.open(file_path_, std::ios::app);
        
    }
    
    for(int i=0;i<maintain_states_.size();i++)
    {
        // if(maintain_states_[i] == NEW)
        // {
        //     maintain_states_[i] = LOST;
        // }
        maintain_states_[i] = LOST;
    }
    // std::cout<<"current size:"<<current_clusters_.size()<<" match size:"<<matched_clusters_.size()<<std::endl;
    for(int i=0;i<matched_clusters_.size();i++)
    {
        // std::cout<<"i: "<<i<<" matched with "<<matched_clusters_[i]<<std::endl;
        if(matched_clusters_[i] >= 0)
        {
            if(maintain_predict_[matched_clusters_[i]].header.frame_id != "wrong")
            {
                if(out_file)
                {
                    out_file<<maintain_ids_[matched_clusters_[i]]<<","<<current_time_.toSec()<<","<<current_clusters_[i].pose.pose.position.x<<","<<\
                            current_clusters_[i].pose.pose.position.y<<","<<current_clusters_[i].pose.pose.position.z<<\
                            ","<<maintain_predict_[matched_clusters_[i]].pose.pose.position.x<<","<<\
                            maintain_predict_[matched_clusters_[i]].pose.pose.position.y<<","<<
                            maintain_predict_[matched_clusters_[i]].pose.pose.position.z<<","<<std::endl;
                }
            }
            
            if(maintain_states_[matched_clusters_[i]] == MATCHED_ONCE || \
               maintain_states_[matched_clusters_[i]] == MATCHED_MORE)
            {
                maintain_states_[matched_clusters_[i]] = MATCHED_MORE;  
            }
            else
            {
                maintain_states_[matched_clusters_[i]] = MATCHED_ONCE;
            }
            // std::cout<<"update matched clusters"<<std::endl;
            // cluster_num_++;
            // maintain_ids_.push_back(cluster_num_);
            std::cout << "The " << matched_clusters_[i] << " object is tracked" << std::endl;
            maintain_clusters_[matched_clusters_[i]] = current_clusters_[i];
            maintain_tracks_[matched_clusters_[i]].FilterProcess(current_clusters_[i]);
            // std::cout<<"updated matched clusters"<<std::endl;
        }
        else
        {
            // std::cout<<"to add new cluster "<<std::endl;
            // std::cout<<"main size: "<<maintain_clusters_.size()<<" "<<maintain_states_.size()<<" "\
            //          <<maintain_tracks_.size()<<std::endl;
            maintain_clusters_.push_back(current_clusters_[i]);
            std::cout<<"add new cluster x: "<<current_clusters_[i].pose.pose.position.x<<" y: "<< \
                      current_clusters_[i].pose.pose.position.y<<" z: "<<\
                      current_clusters_[i].pose.pose.position.z<<std::endl;
            std::cout << "The " << maintain_clusters_.size() -1 << " object is created" << std::endl;
            maintain_states_.push_back(NEW);
            cluster_num_++;
            maintain_ids_.push_back(cluster_num_);
            KalmanFilter new_kalman(delta_t_, t_length_);
            new_kalman.SetInitCov(current_clusters_[i].pose.covariance[0*6+0], \
                                  current_clusters_[i].pose.covariance[1*6+1], \
                                  current_clusters_[i].pose.covariance[2*6+2], current_clusters_[i].header.stamp);
            new_kalman.FilterProcess(current_clusters_[i]);
            maintain_tracks_.push_back(new_kalman);

            // std::vector<geometry_msgs::PoseWithCovarianceStamped> temp_pre;
            // maintain_tracks_[maintain_clusters_.size()-1].GetPredictions(temp_pre);
            // std::cout<<"predict x0: "<<temp_pre[0].pose.pose.position.x<<" y0:"<<temp_pre[0].pose.pose.position.y<< \
            //         " z0:"<<temp_pre[0].pose.pose.position.z<<std::endl;
            // std::cout<<"added new cluster "<<std::endl;
        }
    }
}

void Classifier::DeleteClusters()
{
    // std::cout<<"delete"<<std::endl;
    std::vector<int> delete_flags;
    for(int i=0;i<maintain_states_.size();i++)
    {
        // std::cout<<"i: "<<i<<" all: "<<maintain_states_.size()<<" maintain_states: "<<maintain_states_[i]<<std::endl;
        ros::Time temp = maintain_tracks_[i].GetCurrentTime();
        // std::cout<<"t1: "<<current_time_.toSec()<<" t2:"<<temp.toSec()<<" thresh: "<<delta_t_*t_length_<<\
        //          " result: "<<((current_time_.toSec() - temp.toSec()) > delta_t_*t_length_)<<std::endl;
        if((current_time_.toSec() - temp.toSec()) > delta_t_*t_length_)
        {
            delete_flags.push_back(i);
        }
    }
    // std::cout<<"delete num: "<<delete_flags.size()<<std::endl;
    for(int i=delete_flags.size()-1;i>=0;i--)
    {
        // std::cout<<"delete cluster "<<maintain_ids_[delete_flags[i]]<<" at "<<current_time_.toSec()<<std::endl;
        maintain_clusters_.erase(maintain_clusters_.begin() + delete_flags[i]);
        maintain_states_.erase(maintain_states_.begin() + delete_flags[i]);
        maintain_ids_.erase(maintain_ids_.begin() + delete_flags[i]);
        maintain_tracks_.erase(maintain_tracks_.begin() + delete_flags[i]);
    }
    // std::cout<<"after delete num: "<<maintain_clusters_.size()<<std::endl;
    // std::cout<<"deleted"<<std::endl;
}

void Classifier::AddNewClusters(std::vector<geometry_msgs::PoseWithCovarianceStamped> &new_clusters, \
                                double time)
{
    // std::cout<<"add"<<std::endl;
    current_time_ = ros::Time().fromSec(time);
    DeleteClusters();
    // std::cout<<"delete done"<<std::endl;
    // if((int)(new_clusters.size())==0)
    // {
    //     current_clusters_.clear();
    //     maintain_predict_.clear();
    //     if(set_pub_)
    //     {
    //         ShowPoints();
    //     }
        
    //     return;
    // }
    
    // std::cout<<"get new clusters "<<new_clusters.size()<<std::endl;
    current_clusters_ = new_clusters;
    // current_time_ = current_clusters_[0].header.stamp;
    // if((int)(last_clusters_.size())>=10)
    // {
    //     last_clusters_.pop();
    // }
    // last_clusters_.push(current_clusters_);
    
    // for(int i=0; i < new_clusters.size();i++)
    // {
    //     std::cout<<"111---------"<<new_clusters[i].pose.covariance[2*6+2]<<", "<<\
    //             new_clusters[i].pose.covariance[5*6+5]<<"------"<<std::endl;
    // }

    if((int)(maintain_clusters_.size())==0)
    {
        maintain_clusters_ = current_clusters_;
        for(int i=0;i<maintain_clusters_.size();i++)
        {
            cluster_num_ ++;
            maintain_states_.push_back(NEW);
            maintain_ids_.push_back(cluster_num_);
            KalmanFilter predict_of_i(delta_t_, t_length_);
            predict_of_i.SetInitCov(maintain_clusters_[i].pose.covariance[0*6+0], \
                                    maintain_clusters_[i].pose.covariance[1*6+1], \
                                    maintain_clusters_[i].pose.covariance[2*6+2], maintain_clusters_[i].header.stamp);
            std::cout << "The position of the " << i <<  " cluster is " << maintain_clusters_[i].pose.pose.position.x << " " << maintain_clusters_[i].pose.pose.position.y << " " << maintain_clusters_[i].pose.pose.position.z << std::endl;
            predict_of_i.FilterProcess(maintain_clusters_[i]);
            maintain_tracks_.push_back(predict_of_i);
        }
    }
    GetCurrentPredict();
    // std::cout<<"get predictions"<<maintain_predict_.size()<<std::endl;
    if(set_pub_)
    {
        ShowPoints();
    }
    
    MatchClusters();
    UpdateClusters();
}

void Classifier::DisplayPredict(ros::Publisher current_predict)
{
    // ros::NodeHandle nh;
    static std_msgs::Header temp_header;
    int cur_len = maintain_predict_.size();
    
    // ros::Publisher current_predict = nh.advertise<visualization_msgs::MarkerArray>("/predict_path", 100);
    visualization_msgs::MarkerArray pre_pathes;
    visualization_msgs::MarkerArray pre_numbers;
    for(int j=0;j<maintain_predict_.size();j++)
    {   
        visualization_msgs::Marker pre_path;
        pre_path.ns = "predict";
        pre_path.id = j;
        pre_path.action = visualization_msgs::Marker::ADD;
        pre_path.type = visualization_msgs::Marker::LINE_STRIP;
        pre_path.scale.x = 0.1;
        pre_path.color.g = 1.0;   
        pre_path.color.a = 1.0;  
        pre_path.header = maintain_predict_[j].header;
        Eigen::Vector3f size_predict(std::abs(maintain_predict_[j].pose.covariance[3*6+3]),std::abs(maintain_predict_[j].pose.covariance[4*6+4]),std::abs(maintain_predict_[j].pose.covariance[5*6+5]));
        Eigen::Vector3f center_predict_g(maintain_predict_[j].pose.pose.position.x, maintain_predict_[j].pose.pose.position.y, maintain_predict_[j].pose.pose.position.z);
        Eigen::Quaternionf q_predict(maintain_predict_[j].pose.pose.orientation.w, maintain_predict_[j].pose.pose.orientation.x, maintain_predict_[j].pose.pose.orientation.y, maintain_predict_[j].pose.pose.orientation.z);
        Eigen::Matrix3f R_predict(q_predict);
        // Eigen::Matrix3f R;
        // R.setIdentity();
        Eigen::Vector3f min_predict = R_predict.transpose() * center_predict_g - 0.5f * size_predict;
        Eigen::Vector3f max_predict = R_predict.transpose() * center_predict_g + 0.5f * size_predict;
        bbox_s predict_bbox(min_predict, max_predict, R_predict);
        predict_bbox.draw_bbox(pre_path);
        
        pre_path.lifetime = ros::Duration();
        pre_pathes.markers.push_back(pre_path);
    }
    std::cout << "The size of cur_len: " << cur_len << std::endl;
    if(max_clusters_ > cur_len)
    {
        for(int i=cur_len;i<max_clusters_;i++)
        {
            visualization_msgs::Marker pre_path;
            pre_path.ns = "predict";
            pre_path.id = i;
            pre_path.header.frame_id = "camera_init";
            pre_path.header.stamp = ros::Time::now();
            pre_path.action = visualization_msgs::Marker::ADD;
            pre_path.type = visualization_msgs::Marker::LINE_LIST;
            geometry_msgs::Point p;
            p.x = 0.0;
            p.y = 0.0; 
            p.z = 0.0;
            pre_path.points.push_back(p);
            p.z = 0.1;
            pre_path.points.push_back(p);
            pre_path.scale.x = 0.02;
            pre_path.color.b = 1.0;   
            pre_path.color.a = 0.0;
            pre_pathes.markers.push_back(pre_path);
        }
    }
    else
    {
        max_clusters_ = cur_len;
    }
    current_predict.publish(pre_pathes);
}

void Classifier::DisplayUpdate(ros::Publisher current_update)
{
    // ros::NodeHandle nh;
    static std_msgs::Header temp_header;
    int cur_len = 0;
    // ros::Publisher current_predict = nh.advertise<visualization_msgs::MarkerArray>("/predict_path", 100);
    visualization_msgs::MarkerArray update_pathes;
    visualization_msgs::MarkerArray pre_numbers;
    for(int j=0;j<matched_clusters_.size();j++)
    {   
        if(matched_clusters_[j] > -1)
        {   
            cur_len++;
            visualization_msgs::Marker update_path;
            update_path.ns = "update";
            update_path.id = j;
            update_path.action = visualization_msgs::Marker::ADD;
            update_path.type = visualization_msgs::Marker::LINE_STRIP;
            update_path.scale.x = 0.1;
            update_path.color.r = 210.0/255.;
            update_path.color.g = 105.0/255.;
            update_path.color.b = 30.0/255.;   
            update_path.color.a = 1.0;  
            std::vector<geometry_msgs::PoseWithCovarianceStamped> predictions_after_update;
            maintain_tracks_[matched_clusters_[j]].GetPredictions(predictions_after_update);
            geometry_msgs::PoseWithCovarianceStamped update_state = predictions_after_update[0];
            update_path.header = update_state.header;
            Eigen::Vector3f size_predict(std::abs(update_state.pose.covariance[3*6+3]),std::abs(update_state.pose.covariance[4*6+4]),std::abs(update_state.pose.covariance[5*6+5]));
            Eigen::Vector3f center_predict_g(update_state.pose.pose.position.x, update_state.pose.pose.position.y, update_state.pose.pose.position.z);
            Eigen::Quaternionf q_predict(update_state.pose.pose.orientation.w, update_state.pose.pose.orientation.x, update_state.pose.pose.orientation.y, update_state.pose.pose.orientation.z);
            Eigen::Matrix3f R_predict(q_predict);
            // Eigen::Matrix3f R;
            // R.setIdentity();
            Eigen::Vector3f min_predict = R_predict.transpose() * center_predict_g - 0.5f * size_predict;
            Eigen::Vector3f max_predict = R_predict.transpose() * center_predict_g + 0.5f * size_predict;
            bbox_s update_bbox(min_predict, max_predict, R_predict);
            update_bbox.draw_bbox(update_path);
            
            update_path.lifetime = ros::Duration();
            update_pathes.markers.push_back(update_path);
        }
    }
    if(max_clusters_ > cur_len)
    {
        for(int i=cur_len;i<max_clusters_;i++)
        {
            visualization_msgs::Marker update_path;
            update_path.ns = "update";
            update_path.id = i;
            update_path.header.frame_id = "camera_init";
            update_path.header.stamp = ros::Time::now();
            update_path.action = visualization_msgs::Marker::ADD;
            update_path.type = visualization_msgs::Marker::LINE_LIST;
            geometry_msgs::Point p;
            p.x = 0.0;
            p.y = 0.0; 
            p.z = 0.0;
            update_path.points.push_back(p);
            p.z = 0.1;
            update_path.points.push_back(p);
            update_path.scale.x = 0.02;
            update_path.color.b = 1.0;   
            update_path.color.a = 0.0;
            update_pathes.markers.push_back(update_path);
        }
    }
    current_update.publish(update_pathes);
}

void Classifier::GetPredictPath(std::vector<geometry_msgs::PoseWithCovarianceStamped> &obs_path)
{
    obs_path.clear();

    for(int i=0;i<maintain_clusters_.size();i++)
    {
        std::vector<geometry_msgs::PoseWithCovarianceStamped> predictions_of_i;
        maintain_tracks_[i].GetPredictions(predictions_of_i);
        obs_path.insert(obs_path.end(), predictions_of_i.begin(), predictions_of_i.end());
    }
}

void Classifier::ShowPredictPoint(geometry_msgs::PoseWithCovarianceStamped &obs_point, \
                                int show_id, ros::Publisher &pub_predict)
{
    if(obs_point.header.frame_id == "null")
    {
        // std::cout<<"null 1"<<std::endl;
        visualization_msgs::Marker ellip_show;
        ellip_show.header.frame_id = obs_point.header.frame_id;
        ellip_show.header.stamp = ros::Time::now();
        ellip_show.ns = "predict";
        ellip_show.id = show_id;
        ellip_show.type = visualization_msgs::Marker::SPHERE;
        ellip_show.action = visualization_msgs::Marker::ADD;
        ellip_show.color.r = 0;
        ellip_show.color.g = 0;
        ellip_show.color.b = 0;
        ellip_show.color.a = 0;
        ellip_show.pose.position.x = 0;
        ellip_show.pose.position.y = 0;
        ellip_show.pose.position.z = 0;
        ellip_show.scale.x = 0;
        ellip_show.scale.y = 0;
        ellip_show.scale.z = 0;
        pub_predict.publish(ellip_show);
        return;
    }
    double confidence = 2.97;
    visualization_msgs::Marker ellip_show;
    ellip_show.header.frame_id = obs_point.header.frame_id;
    ellip_show.header.stamp = ros::Time::now();
    ellip_show.ns = "predict";
    ellip_show.id = show_id;
    ellip_show.type = visualization_msgs::Marker::SPHERE;
    ellip_show.action = visualization_msgs::Marker::ADD;
    ellip_show.color.r = 0;
    ellip_show.color.g = 0;
    ellip_show.color.b = 1;
    ellip_show.color.a = 0.6;
    ellip_show.pose.position.x = obs_point.pose.pose.position.x;
    ellip_show.pose.position.y = obs_point.pose.pose.position.y;
    ellip_show.pose.position.z = obs_point.pose.pose.position.z;
    Eigen::Matrix3d cov;
    cov = Eigen::Matrix3d::Zero();
    cov(0,0) = obs_point.pose.covariance[0*6+0];
    cov(1,1) = obs_point.pose.covariance[1*6+1];
    cov(2,2) = obs_point.pose.covariance[2*6+2];
    ellip_show.scale.x = 2*fabs(confidence*sqrt(cov(0,0)));
    ellip_show.scale.y = 2*fabs(confidence*sqrt(cov(1,1)));//2*sqrt(2*cov(1,1))*log(1/(confidence*sqrt(2*PI*cov(1,1))));
    ellip_show.scale.z = 2*fabs(confidence*sqrt(cov(2,2)));//2*sqrt(2*cov(2,2))*log(1/(confidence*sqrt(2*PI*cov(2,2))));
    pub_predict.publish(ellip_show);
}

void Classifier::ShowCurrentPoint(geometry_msgs::PoseWithCovarianceStamped &cur_point, \
                                int show_id, ros::Publisher &pub_predict)
{
    if(cur_point.header.frame_id == "null")
    {
        // std::cout<<"null 2"<<std::endl;
        visualization_msgs::Marker ellip_show;
        ellip_show.header.frame_id = cur_point.header.frame_id;
        ellip_show.header.stamp = ros::Time::now();
        ellip_show.ns = "current";
        ellip_show.id = show_id;
        ellip_show.type = visualization_msgs::Marker::SPHERE;
        ellip_show.action = visualization_msgs::Marker::ADD;
        ellip_show.color.r = 0;
        ellip_show.color.g = 0;
        ellip_show.color.b = 0;
        ellip_show.color.a = 0;
        ellip_show.pose.position.x = 0;
        ellip_show.pose.position.y = 0;
        ellip_show.pose.position.z = 0;
        ellip_show.scale.x = 0;
        ellip_show.scale.y = 0;
        ellip_show.scale.z = 0;
        pub_predict.publish(ellip_show);
        return;
    }
    double confidence = 2.97;
    visualization_msgs::Marker ellip_show;
    ellip_show.header.frame_id = cur_point.header.frame_id;
    ellip_show.header.stamp = ros::Time::now();
    ellip_show.ns = "current";
    ellip_show.id = show_id;
    ellip_show.type = visualization_msgs::Marker::SPHERE;
    ellip_show.action = visualization_msgs::Marker::ADD;
    ellip_show.color.r = 1;
    ellip_show.color.g = 0;
    ellip_show.color.b = 0;
    ellip_show.color.a = 0.6;
    ellip_show.pose.position.x = cur_point.pose.pose.position.x;
    ellip_show.pose.position.y = cur_point.pose.pose.position.y;
    ellip_show.pose.position.z = cur_point.pose.pose.position.z;
    Eigen::Matrix3d cov;
    cov = Eigen::Matrix3d::Zero();
    cov(0,0) = cur_point.pose.covariance[0*6+0];
    cov(1,1) = cur_point.pose.covariance[1*6+1];
    cov(2,2) = cur_point.pose.covariance[2*6+2];
    ellip_show.scale.x = 2*fabs(confidence*sqrt(cov(0,0)));
    ellip_show.scale.y = 2*fabs(confidence*sqrt(cov(1,1)));//2*sqrt(2*cov(1,1))*log(1/(confidence*sqrt(2*PI*cov(1,1))));
    ellip_show.scale.z = 2*fabs(confidence*sqrt(cov(2,2)));//2*sqrt(2*cov(2,2))*log(1/(confidence*sqrt(2*PI*cov(2,2))));
    pub_predict.publish(ellip_show);
}

void Classifier::ShowPoints()
{
    if(show_num_cur_ > max_clusters_)
    {
        ROS_ERROR(" cur error");
    }
    if(show_num_obs_ > max_clusters_)
    {
        ROS_ERROR(" obs error");
    }
    if(show_num_obs_ < maintain_predict_.size())
    {
        show_num_obs_ = maintain_predict_.size();
    }
    if(show_num_cur_ < current_clusters_.size())
    {
        show_num_cur_ = current_clusters_.size();
    }

    for(int i=0;i<maintain_predict_.size();i++)
    {
        ShowPredictPoint(maintain_predict_[i], i, pub_predict_);
    }
    for(int i=maintain_predict_.size(); i<show_num_obs_; i++)
    {
        geometry_msgs::PoseWithCovarianceStamped null_predict;
        null_predict.header.frame_id = "null";
        ShowPredictPoint(null_predict, i, pub_predict_);
    }
    for(int i=0;i<current_clusters_.size();i++)
    {
        ShowCurrentPoint(current_clusters_[i], i, pub_predict_);
    }
    for(int i=current_clusters_.size(); i<show_num_cur_; i++)
    {
        geometry_msgs::PoseWithCovarianceStamped null_predict;
        null_predict.header.frame_id = "null";
        ShowCurrentPoint(null_predict, i, pub_predict_);
    }


}

void Classifier::SetPub(ros::Publisher &pub_predict)
{
    set_pub_ = true;
    pub_predict_ = pub_predict;
}
