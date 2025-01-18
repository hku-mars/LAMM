#ifndef KALMAN_H
#define KALMAN_H

#include <math.h>
#include <time.h>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class KalmanFilter
{
public:
    KalmanFilter();
    KalmanFilter(double delta_t, int t_length);
    void SetInitCov(double x_cov, double y_cov, double z_cov, ros::Time init_time);
    void FilterProcess(const geometry_msgs::PoseWithCovarianceStamped &pose);
    void SetPredictTime(double delta_t, int t_length);
    void GetPredictions(std::vector<geometry_msgs::PoseWithCovarianceStamped> &predictions);
    ros::Time GetCurrentTime();
private:
    ros::Subscriber     sub_;
    ros::Publisher      pub_;
    ros::Publisher      odo_pub_;

    Eigen::Matrix<double, 9, 1>     x_measured_;          // measured position information

    //! Time information for filter
    ros::Time           current_time_;            // time stamp of current measurement
    ros::Time           last_time_;   // time stamp of last measurement
    double              dt_;                    // time difference between two measurements

    double              delta_t_;               
    int                 t_length_; 
    double Q_pos_, Q_vel_;                      


    Eigen::Matrix<double, 15, 1> state_estimated_;      
    Eigen::Matrix<double, 15, 15> state_cov_estimated_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> predictions_;  

    

};



#endif
