#include <nav_msgs/Odometry.h>
#include <fstream>
#include <ros/ros.h>

using namespace std;

bool is_rec = false;
string out_file = "";
ofstream out;

void OdomCallback(const nav_msgs::Odometry &cur_odom)
{
    float pos_x = cur_odom.pose.pose.position.x;
    float pos_y = cur_odom.pose.pose.position.y;
    float pos_z = cur_odom.pose.pose.position.z;

    if(is_rec) 
    {
        out.write((char*)&pos_x, sizeof(float));
        out.write((char*)&pos_y, sizeof(float));
        out.write((char*)&pos_z, sizeof(float));
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_odom");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom = nh.subscribe("/aft_mapped_to_init", 200000, OdomCallback);
    nh.param<string>("dyn_obj/odom_file",  out_file, "");

    if(out_file.size() > 0)
    {
        out.open(out_file, ios::out  | ios::binary);
        if (out.is_open()) 
        {
            is_rec = true;
            float pos_x = 0;
            float pos_y = 0;
            float pos_z = 0;
            out.write((char*)&pos_x, sizeof(float));
            out.write((char*)&pos_y, sizeof(float));
            out.write((char*)&pos_z, sizeof(float));

        }
        
    }
    

    ros::spin();
    return 0;
}
