#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H

#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector> 
#include <queue> 
#include <visualization_msgs/MarkerArray.h>
#include <cluster_predict/kalman.h>
#include <fstream>
#include <sstream> 

class Classifier
{
private:
    enum MAINTAIN_STATE
    {
      MATCHED_ONCE = 1,
      MATCHED_MORE = 2,
      LOST = 3,
      NEW = 4,
    };

public:
    Classifier();
    
    Classifier(double delta_t, int t_length, double dis_thresh);
    void SetParam(ros::NodeHandle& nh);
    void AddNewClusters(std::vector<geometry_msgs::PoseWithCovarianceStamped> &new_clusters, \
                        double time);
    void DisplayPredict(ros::Publisher current_predict);//ros::NodeHandle &nh
    void DisplayUpdate(ros::Publisher current_update);
    void SetFile(std::string file_path);
    void SetTime(double delta_t, int t_length);
    void SetPub(ros::Publisher &pub_predict);
    void SetThresh(double dis_thresh);
    void GetPredictPath(std::vector<geometry_msgs::PoseWithCovarianceStamped> &obs_path);
    void DeleteClusters();
    void ShowPoints();
    
    void ShowPredictPoint(geometry_msgs::PoseWithCovarianceStamped &obs_point, int show_id,\
                        ros::Publisher &pub_predict);
    void ShowCurrentPoint(geometry_msgs::PoseWithCovarianceStamped &cur_point, int show_id,\
                        ros::Publisher &pub_predict);

private:
    double delta_t_;
    bool set_pub_ = false;
    int t_length_;
    double dis_thresh_;
    int max_clusters_;
    int show_num_cur_;
    int show_num_obs_;
    ros::Publisher pub_predict_;
    std::string file_path_;
    std::queue<std::vector<geometry_msgs::PoseWithCovarianceStamped>> last_clusters_; //main the last 10 frames' cluster result
    std::vector<geometry_msgs::PoseWithCovarianceStamped> current_clusters_;
    std::vector<int> matched_clusters_; //>=0 means its match to maintain_clusters, -1 means there is no match
    std::vector<geometry_msgs::PoseWithCovarianceStamped> maintain_clusters_;
    std::vector<MAINTAIN_STATE> maintain_states_;
    std::vector<int> maintain_ids_;
    int cluster_num_ = -1;
    std::vector<KalmanFilter> maintain_tracks_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> maintain_predict_;
    ros::Time current_time_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> correlations_;
    void GetCurrentPredict();
    void MatchClusters();
    void UpdateClusters();
    double CalCorrelation_aabb(geometry_msgs::PoseWithCovarianceStamped &current, \
                        geometry_msgs::PoseWithCovarianceStamped &predict);
    double CalCorrelation_oobb(geometry_msgs::PoseWithCovarianceStamped &current, \
                        geometry_msgs::PoseWithCovarianceStamped &predict);
    void GetCorMatrix();
    
};

#endif
