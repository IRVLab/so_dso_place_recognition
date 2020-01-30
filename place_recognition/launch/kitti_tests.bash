#!/bin/bash

for s in 00 05 06 07
do
    for m in delight m2dp sc 
    do
        roslaunch so_dso_place_recognition kitti_lidar.launch seq:=$s method:=$m
    done
    roslaunch so_dso_place_recognition kitti_gist.launch seq:=$s
    roslaunch so_dso_place_recognition kitti_bow.launch seq:=$s
done