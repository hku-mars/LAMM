#include <cluster_predict/kalman.h>

#define PI 3.1415926
#define SIGN(y) (y>=0?1:-1)

double euler_wrap(double x)
{
    return std::fmod(x + SIGN(x) * PI, 2*PI) - SIGN(x) * PI;
}

void R_to_RPY(const Eigen::Matrix3d &R, Eigen::Vector3d &RPY)
{
    RPY(1,0) = -asin(R(2,0));
    RPY(0,0) = atan2(R(2,1)/cos(RPY(1,0)), R(2,2)/cos(RPY(1,0)));
    RPY(2,0) = atan2(R(1,0)/cos(RPY(1,0)), R(0,0)/cos(RPY(1,0)));
}

KalmanFilter::KalmanFilter()
{
    delta_t_ = 0.02;
    t_length_ = 50;
    double cov_pos = 1^2;
    double cov_vel = 10^2;
    double cov_rot = 1^2;
    double cov_rotv = 10^2;
    double cov_dim = 1^2;
    state_cov_estimated_.setZero();
    state_cov_estimated_(0, 0) = cov_pos;
    state_cov_estimated_(1, 1) = cov_pos;
    state_cov_estimated_(2, 2) = cov_pos;
    state_cov_estimated_(3, 3) = cov_vel;
    state_cov_estimated_(4, 4) = cov_vel;
    state_cov_estimated_(5, 5) = cov_vel;
    state_cov_estimated_(6, 6) = cov_rot;
    state_cov_estimated_(7, 7) = cov_rot;
    state_cov_estimated_(8, 8) = cov_rot;
    state_cov_estimated_(9, 9) = cov_rotv;
    state_cov_estimated_(10, 10) = cov_rotv;
    state_cov_estimated_(11, 11) = cov_rotv;
    state_cov_estimated_(12, 12) = cov_dim;
    state_cov_estimated_(13, 13) = cov_dim;
    state_cov_estimated_(14, 14) = cov_dim;


    // Other initialization
    x_measured_.setZero();
    state_estimated_.setZero();

    // current_time_ = ros::Time::now();
    // last_time_ = ros::Time::now();
    dt_ = 0.01;
}

KalmanFilter::KalmanFilter(double delta_t, int t_length): delta_t_(delta_t), t_length_(t_length)
{
    // std::cout<<"start to initialize filter"<<std::endl;
    double cov_pos = 0.2*0.2;
    double cov_vel = 10^2;
    double cov_rot = 0.2*0.2;
    double cov_rotv = 10^2;
    double cov_dim = 0.1*0.1;
    state_cov_estimated_.setZero();
    state_cov_estimated_(0, 0) = cov_pos;
    state_cov_estimated_(1, 1) = cov_pos;
    state_cov_estimated_(2, 2) = cov_pos;
    state_cov_estimated_(3, 3) = cov_vel;
    state_cov_estimated_(4, 4) = cov_vel;
    state_cov_estimated_(5, 5) = cov_vel;
    state_cov_estimated_(6, 6) = cov_rot;
    state_cov_estimated_(7, 7) = cov_rot;
    state_cov_estimated_(8, 8) = cov_rot;
    state_cov_estimated_(9, 9) = cov_rotv;
    state_cov_estimated_(10, 10) = cov_rotv;
    state_cov_estimated_(11, 11) = cov_rotv;
    state_cov_estimated_(12, 12) = cov_dim;
    state_cov_estimated_(13, 13) = cov_dim;
    state_cov_estimated_(14, 14) = cov_dim;

    // Other initialization
    x_measured_.setZero();
    state_estimated_.setZero();

    // current_time_ = ros::Time::now();
    // last_time_ = ros::Time::now();
    dt_ = 0.01;
    // std::cout<<"initialized filters"<<std::endl;
}

void KalmanFilter::SetInitCov(double x_cov, double y_cov, double z_cov, ros::Time init_time)
{
    // delta_t_ = 0.02;
    // t_length_ = 50;
    double cov_vel = 10*10*delta_t_*delta_t_;
    // state_cov_estimated_.setZero();
    state_cov_estimated_(0, 0) = x_cov;
    state_cov_estimated_(1, 1) = y_cov;
    state_cov_estimated_(2, 2) = z_cov;
    state_cov_estimated_(3, 3) = cov_vel;
    state_cov_estimated_(4, 4) = cov_vel;
    state_cov_estimated_(5, 5) = cov_vel;
    // std::cout<< "The init state cov: " << state_cov_estimated_ << std::endl;

    // Other initialization
    x_measured_.setZero();
    state_estimated_.setZero();
    last_time_ = init_time;

}

void KalmanFilter::SetPredictTime(double delta_t, int t_length)
{ 
    delta_t_ = delta_t;
    t_length_ = t_length;
}

void KalmanFilter::GetPredictions(std::vector<geometry_msgs::PoseWithCovarianceStamped> &predictions)
{
    predictions.clear();
    predictions = predictions_;
}

ros::Time KalmanFilter::GetCurrentTime()
{
    return current_time_;
}

void KalmanFilter::FilterProcess(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    // static ros::Time last_time_ = current_time_; // bug

    // std::cout<<"start kalman filters"<<std::endl;
    x_measured_(0,0) = pose.pose.pose.position.x;
    x_measured_(1,0) = pose.pose.pose.position.y;
    x_measured_(2,0) = pose.pose.pose.position.z;
    Eigen::Quaterniond q;
    q.x() = pose.pose.pose.orientation.x;
    q.y() = pose.pose.pose.orientation.y;
    q.z() = pose.pose.pose.orientation.z;
    q.w() = pose.pose.pose.orientation.w;
    // Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    Eigen::Matrix3d rot_mat = q.toRotationMatrix();
    Eigen::Vector3d euler;
    Eigen::Vector3d vel_dir(state_estimated_(3,0), state_estimated_(4,0), state_estimated_(5,0));
    Eigen::Vector3d mea_dir = rot_mat.col(0);
    if(vel_dir.norm() > 0.001 && vel_dir.dot(mea_dir) < 0.)
    {
        rot_mat.col(0) = rot_mat.col(0) * -1.0;
        rot_mat.col(1) = rot_mat.col(1) * -1.0;
    }
    R_to_RPY(rot_mat, euler);
    // std::cout << "R: " << rot_mat << std::endl;
    // std::cout << "euler: " << euler.transpose() << std::endl;
    // x_measured_(3,0) = euler(2);
    // x_measured_(4,0) = euler(1);
    // x_measured_(5,0) = euler(0);
    x_measured_(3,0) = euler(0);
    x_measured_(4,0) = euler(1);
    x_measured_(5,0) = euler(2);
    x_measured_(6,0) = pose.pose.covariance[3*6+3];
    x_measured_(7,0) = pose.pose.covariance[4*6+4];
    x_measured_(8,0) = pose.pose.covariance[5*6+5];
    current_time_ = pose.header.stamp;
    
    dt_ = current_time_.toSec() - (last_time_).toSec();
    // std::cout<<"dt: "<<dt_<<" last_time:"<<last_time_.toSec() << " current time: " << current_time_.toSec() <<std::endl;
    last_time_ = current_time_;
    Eigen::Matrix<double, 15, 15> F;
    F << 1, 0, 0, dt_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, dt_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, dt_, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0, dt_, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt_, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt_, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    // std::cout<<"dt:"<<dt_<<std::endl;
    // std::cout<<state_estimated_<<"*********"<<std::endl;
    state_estimated_ = F*state_estimated_; //
    state_estimated_(6,0) = euler_wrap(state_estimated_(6,0));
    state_estimated_(7,0) = euler_wrap(state_estimated_(7,0));
    state_estimated_(8,0) = euler_wrap(state_estimated_(8,0));
    // std::cout<<F<<"*********"<<std::endl;
    double Q_acc = 10 ; 
    double Q_discrete = Q_acc*dt_*dt_*dt_;             
    // double Q_pos = Q_acc*dt_*dt_*0.5 + Q_discrete;//0;
    double Q_pos = 0;
    double Q_vel = Q_acc*dt_;
    Q_pos = Q_pos*Q_pos;
    Q_vel = Q_vel*Q_vel;
    double Q_rota = 1;
    double Q_rp = 0.1;
    double Q_y = 0;
    double Q_rotv = Q_rota * dt_;
    Q_rp = Q_rp * Q_rp;
    Q_y = Q_y * Q_y;
    Q_rotv = Q_rotv * Q_rotv;
    double Q_dim = 0.01;
    Eigen::Matrix<double, 15, 15> Q;
    Q.setZero();
    Q.diagonal() << Q_pos, Q_pos, Q_pos, Q_vel, Q_vel, Q_vel, Q_rp, Q_rp, Q_y, Q_rotv, Q_rotv, Q_rotv, Q_dim, Q_dim, Q_dim;
    Eigen::Matrix<double, 9, 15> H;
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    if (state_estimated_.norm() < 0.001)
    {
        state_estimated_ = H.transpose() * x_measured_;
    }
    else
    {   
        state_cov_estimated_ = F*state_cov_estimated_*F.transpose() + Q;
        // std::cout<<"dt: "<<dt_<<"  Qpos: "<<Q_pos<<" Qvel: "<<Q_vel<<std::endl;
        // std::cout<<"predicted"<<std::endl;

        // double R_pos = 10E-4;
        Eigen::Matrix<double, 9, 9> R;      // observation noise covariance
        R.setZero();
        // R << R_pos, 0, 0,
        //      0, R_pos, 0,
        //      0, 0, R_pos;
        // R.block(0,0,3,3) << pose.pose.covariance[0*6+0]/100, 0, 0,
        //                     0, pose.pose.covariance[1*6+1]/100, 0,
        //                     0, 0, pose.pose.covariance[2*6+2]/100;
        R.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.1;
        Eigen::Matrix<double, 9, 9> S;
        S = H*state_cov_estimated_*H.transpose() + R;
        Eigen::Matrix<double, 15, 9> K;
        // std::cout<<"R: "<<R<<std::endl;
        K = (state_cov_estimated_*H.transpose()) * S.inverse();
        Eigen::Matrix<double, 9, 1> x_residual;
        x_residual = x_measured_ - H * state_estimated_;
        x_residual(3,0) =  euler_wrap(x_residual(3,0));
        x_residual(4,0) =  euler_wrap(x_residual(4,0));
        x_residual(5,0) =  euler_wrap(x_residual(5,0));
        // std::cout << "x_predict: " << state_estimated_.transpose() << std::endl;
        Eigen::AngleAxisd rollangle1(state_estimated_(6,0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchangle1(state_estimated_(7,0), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawangle1(state_estimated_(8,0), Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawangle1 * pitchangle1 * rollangle1;
        // std::cout << "R predict: " << q.toRotationMatrix() << std::endl;
        Eigen::Vector3d pre_dir = q.toRotationMatrix().col(0);

        // std::cout<<"x_measured_: "<<x_measured_.transpose() << std::endl << " x_residual: "<<x_residual.transpose() <<std::endl;
        // std::cout<<state_estimated_<<"*************"<<std::endl;
        state_estimated_ = state_estimated_ + K*x_residual;
        Eigen::Vector3d vel_upd(state_estimated_(3,0), state_estimated_(4,0), state_estimated_(5,0));
        if(vel_dir.norm() < 0.001 && vel_upd.norm() > 0.001)
        {   
            bool re_update = false;
            Eigen::Matrix<double, 15, 1> state_origin = state_estimated_ - K*x_residual;
            if(vel_upd.dot(mea_dir) < 0)
            {   
                rot_mat.col(0) = rot_mat.col(0) * -1.0;
                rot_mat.col(1) = rot_mat.col(1) * -1.0;
                R_to_RPY(rot_mat, euler);
                x_measured_(3,0) = euler(0);
                x_measured_(4,0) = euler(1);
                x_measured_(5,0) = euler(2);
                re_update = true;
            }
            if(vel_upd.dot(pre_dir) < 0)
            {
                state_origin(8,0) = euler_wrap(state_origin(8,0) + PI);
                re_update = true;
            }
            if(re_update)
            {
                x_residual = x_measured_ - H * state_origin;
                x_residual(3,0) =  euler_wrap(x_residual(3,0));
                x_residual(4,0) =  euler_wrap(x_residual(4,0));
                x_residual(5,0) =  euler_wrap(x_residual(5,0));
                state_estimated_ = state_origin + K*x_residual;
            }
        }
        // std::cout << "x_update before wrap: " << state_estimated_.transpose() << std::endl;
        state_estimated_(6,0) = euler_wrap(state_estimated_(6,0));
        state_estimated_(7,0) = euler_wrap(state_estimated_(7,0));
        state_estimated_(8,0) = euler_wrap(state_estimated_(8,0));
        // std::cout << "x_updated: " << state_estimated_.transpose() << std::endl;
        Eigen::Matrix<double, 15, 15> I;
        I.setIdentity();              
        state_cov_estimated_ = (I - K*H) * state_cov_estimated_;
    }
    // std::cout<<"updated"<<std::endl;
    
    // std::cout<<"state cov: " << state_cov_estimated_ <<std::endl;
    Eigen::Matrix<double,15, 15> F_pre;
    F_pre << 1, 0, 0, delta_t_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, delta_t_, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, delta_t_, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, delta_t_, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, delta_t_, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, delta_t_, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 15, 1> state_now;
    Eigen::Matrix<double, 15, 1> state_next;
    state_now = state_estimated_;                
    // Q_pos = 0;//Q_acc * delta_t_ * delta_t_ * 0.5;
    // Q_vel = Q_acc * delta_t_;
    Eigen::Matrix<double, 15, 15> Q_pre;
    Q_pre.setZero();
    Q_pre.diagonal() << Q_pos, Q_pos, Q_pos, Q_vel, Q_vel, Q_vel, Q_rp, Q_rp, Q_y, Q_rotv, Q_rotv, Q_rotv, Q_dim, Q_dim, Q_dim;
    Eigen::Matrix<double, 15, 15> cov_now;
    Eigen::Matrix<double, 15, 15> cov_next;
    cov_now = state_cov_estimated_;
    // nav_msgs::Path predict_path;
    // predict_path.header = pose.header;
    predictions_.clear();
    for (int i = 0; i < t_length_; i++)
    {
        
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = pose.header.frame_id;
        poseStamped.header.stamp = ros::Time().fromSec(pose.header.stamp.toSec() + i*delta_t_);
        poseStamped.pose.position.x = state_now(0,0);
        poseStamped.pose.position.y = state_now(1,0);
        poseStamped.pose.position.z = state_now(2,0); 
        Eigen::AngleAxisd rollangle(state_now(6,0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchangle(state_now(7,0), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawangle(state_now(8,0), Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawangle * pitchangle * rollangle;
        poseStamped.pose.orientation.x = q.x();
        poseStamped.pose.orientation.y = q.y();
        poseStamped.pose.orientation.z = q.z();
        poseStamped.pose.orientation.w = q.w();
        // predict_path.poses.push_back(poseStamped);

        
        geometry_msgs::PoseWithCovarianceStamped pose_covariance;
        pose_covariance.header = poseStamped.header;
        pose_covariance.pose.pose.position = poseStamped.pose.position;
        pose_covariance.pose.pose.orientation = poseStamped.pose.orientation;
        pose_covariance.pose.covariance[0*6+0] = cov_now(0,0);
        pose_covariance.pose.covariance[1*6+1] = cov_now(1,1);
        pose_covariance.pose.covariance[2*6+2] = cov_now(2,2);
        // pose_covariance.pose.covariance[3*6+3] = pose.pose.covariance[3*6+3];
        // pose_covariance.pose.covariance[4*6+4] = pose.pose.covariance[4*6+4];
        // pose_covariance.pose.covariance[5*6+5] = pose.pose.covariance[5*6+5];
        pose_covariance.pose.covariance[3*6+3] = state_now(12,0);
        pose_covariance.pose.covariance[4*6+4] = state_now(13,0);
        pose_covariance.pose.covariance[5*6+5] = state_now(14,0);
        pose_covariance.pose.covariance[2*6+3] = pose.pose.covariance[2*6+3] + pose_covariance.pose.pose.position.x - pose.pose.pose.position.x;
        pose_covariance.pose.covariance[3*6+4] = pose.pose.covariance[3*6+4] + pose_covariance.pose.pose.position.y - pose.pose.pose.position.y;
        pose_covariance.pose.covariance[4*6+5] = pose.pose.covariance[4*6+5] + pose_covariance.pose.pose.position.z - pose.pose.pose.position.z;
        pose_covariance.pose.covariance[3*6+2] = pose.pose.covariance[3*6+2] + pose_covariance.pose.pose.position.x - pose.pose.pose.position.x;
        pose_covariance.pose.covariance[4*6+3] = pose.pose.covariance[4*6+3] + pose_covariance.pose.pose.position.y - pose.pose.pose.position.y;
        pose_covariance.pose.covariance[5*6+4] = pose.pose.covariance[5*6+4] + pose_covariance.pose.pose.position.z - pose.pose.pose.position.z;
        // std::cout<<"------"<<pose_covariance.pose.covariance[2*6+2]<<","<<\
        //             pose_covariance.pose.covariance[5*6+5]<<std::endl;
        // for(int i=0;i<3;i++)
        // {
        //     for(int j=0;j<3;j++)
        //     {
        //         pose_covariance.pose.covariance[i*6+j] = cov_now(i,j);
        //     }
        // }   
        // std::cout<<"x: "<<pose_covariance.pose.pose.position.x<<" y: "<<pose_covariance.pose.pose.position.y\
        //         <<" z: "<<pose_covariance.pose.pose.position.z<<std::endl;
        predictions_.push_back(pose_covariance);
        
        state_next = F_pre*state_now;
        state_next(6,0) = euler_wrap(state_next(6,0));
        state_next(7,0) = euler_wrap(state_next(7,0));
        state_next(8,0) = euler_wrap(state_next(8,0));
        state_now = state_next;
        cov_next = F_pre*cov_now*F_pre.transpose() + Q_pre;
        cov_now = cov_next;
    }


}

