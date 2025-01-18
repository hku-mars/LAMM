#!/bin/bash

clear;
declare -i i=0;
declare -i num_raw=0;
str_raw=`echo ${num_raw}|awk '{printf("%04d\n", $0)}'`;
str_cur=`echo ${i}|awk '{printf("%04d\n", $0)}'`;

for ((i=0; i<50; i++))
do
    str_raw=`echo ${num_raw}|awk '{printf("%04d\n", $0)}'`;
    str_cur=`echo ${i}|awk '{printf("%04d\n", $0)}'`;
    sed -i "100,105 s/ ${str_raw} / ${str_cur} /" /home/huajie/event_detection/detection_ws/src/FAST_LIO/config/waymo1.yaml;
    
    # mkdir -p /home/huajie/event_detection/waymo/dataset/sequences/${str_cur}/predictions/
    # roslaunch fast_lio mapping_waymo1.launch & sleep 1;
    # rosbag play /home/huajie/event_detection/bag/waymo${str_cur}.bag --pause;
    num_raw=$i;
done


exit 0;
