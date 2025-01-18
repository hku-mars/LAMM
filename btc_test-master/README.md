### **功能介绍**

具备全局优化gtsam的功能，可以高效区分不回环的部分

BTC_ws是新方案（只遍历bag节点），BTC_test是旧方案（遍历frame节点）

运行all_multi_mapping.launch

输入为odom和每帧点云

### 更改数据集步骤

可以用FastLio2_ws获取odom文件

对检测的launch文件和load的launch文件都要

1. 保存数据集（bag或者pcd，以及odom文件）去某一路径，在launch文件中相应修改root_dir，在multi_mapping和load_cloud的代码中相应修改pcd_file；
2. 在root_dir路径下建立文件夹pose_correct存放修正后的位姿，修改load的launch文件内的load_dir；
3. 数据集从0开始命名，存储格式如下，txt为odom文件，读取路径在代码中根据root_file已经写好为pose_file，无需改动，文件夹内为pcd文件；
4. 修改参数表，如果有外参需要加入外参；
5. 修改data_num，修改icp_threshold；

# **Prerequisites**

## **1. Ubuntu and [ROS](https://www.ros.org/)**
We tested our code on Ubuntu18.04 with ros melodic and Ubuntu20.04 with noetic. Additional ROS package is required:
```
sudo apt-get install ros-xxx-pcl-conversions
```

## **2. Eigen**
Following the official [Eigen installation](eigen.tuxfamily.org/index.php?title=Main_Page), or directly install Eigen by:
```
sudo apt-get install libeigen3-dev
```
## **3. ceres-solver (version>=2.1)**
Please kindly install ceres-solver by following the guide on [ceres Installation](http://ceres-solver.org/installation.html). Notice that the version of ceres-solver should higher than [ceres-solver 2.1.0](https://github.com/ceres-solver/ceres-solver/releases/tag/2.1.0)

## **4. GTSAM**
Following the official [GTSAM installation](https://gtsam.org/get_started/), or directly install GTSAM 4.x stable release by:
```
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
**!! IMPORTANT !!**: Please do not install the GTSAM of ***develop branch***, which are not compatible with our code! We are still figuring out this issue.
