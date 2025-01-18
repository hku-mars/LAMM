#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
    std::string input_dir = argv[1];
    std::string output_dir = argv[2];
    // std::string input_file = input_dir + "/pose_lio/" + sequence_num + ".txt";
    // std::string output_file = input_dir + "/pose_lio_kitti/" + sequence_num + ".txt";
    std::string input_file = input_dir;
    std::string output_file = output_dir;
    std::ifstream posefile(input_file, std::ios::in);
    std::ofstream outfile;
    outfile.open(output_file, std::ios::out);

    std::string line;
    Eigen::Matrix3d R_vel_c;
    R_vel_c << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    // std::cout << "R_vel_c: " << R_vel_c << std::endl;
    while (std::getline(posefile, line)) {
        std::stringstream line_stream(line);
        std::vector<double> parsed_doubles;

        while (!line_stream.eof()) {
            std::string element;
            std::getline(line_stream, element, ' ');
            if (element.empty()) {
                continue;
            }
            try {
                parsed_doubles.emplace_back(std::stod(element));
            } catch (const std::exception& exception) {
            std::cout << "Could not parse number in import file.\n";
            }
        }
        Eigen::Matrix<double, 9, 1> V(parsed_doubles.data());
        geometry_msgs::Pose pose;
        // pose.position.x = V[2];
        // pose.position.y = V[3];
        // pose.position.z = V[4];
        // pose.orientation.x = V[5];
        // pose.orientation.y = V[6];
        // pose.orientation.z = V[7];
        // pose.orientation.w = V[8];
        pose.position.x = V[0];
        pose.position.y = V[1];
        pose.position.z = V[2];
        pose.orientation.w = V[3];
        pose.orientation.x = V[4];
        pose.orientation.y = V[5];
        pose.orientation.z = V[6];
        Eigen::Vector3d t_vel_gl(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond q_vel_gl(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Matrix3d R_vel_gl(q_vel_gl);
        // Eigen::Matrix3d R_c_gl = R_vel_c.transpose() * R_vel_gl * R_vel_c;
        // t_vel_gl = R_vel_c.transpose() * t_vel_gl;
        outfile << R_vel_gl(0,0) << " " << R_vel_gl(0,1) << " "  << R_vel_gl(0,2) << " "  << t_vel_gl(0) << " ";
        outfile << R_vel_gl(1,0) << " " << R_vel_gl(1,1) << " "  << R_vel_gl(1,2) << " "  << t_vel_gl(1) << " ";
        outfile << R_vel_gl(2,0) << " " << R_vel_gl(2,1) << " "  << R_vel_gl(2,2) << " "  << t_vel_gl(2) << std::endl;
    }
    outfile.close();
}