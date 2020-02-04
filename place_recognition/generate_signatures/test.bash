#!/bin/bash

# KITTI
# for s in seq00 seq05 seq06 seq07
# do
#     for m in delight m2dp sc 
#     do
#         roslaunch so_dso_place_recognition lidar.launch dataset:=KITTI seq:=$s method:=$m
#     done
#     roslaunch so_dso_place_recognition gist.launch dataset:=KITTI img_topic:=/left/image_raw seq:=$s 
#     roslaunch so_dso_place_recognition bow.launch dataset:=KITTI img_topic:=/left/image_raw seq:=$s
# done

# RobotCar
for s in 2014-07-14-14-49-50 2014-11-28-12-07-13 2014-12-12-10-45-15 2015-02-10-11-58-05 2015-05-19-14-06-38 2015-05-22-11-14-30 2015-08-13-16-02-58 2015-10-30-13-52-14
do
    # for m in delight sc m2dp
    # do
    #     roslaunch so_dso_place_recognition lidar.launch dataset:=RobotCar seq:=$s method:=$m
    # done
    roslaunch so_dso_place_recognition gist.launch dataset:=RobotCar img_topic:=camera/left/image_raw seq:=$s
    roslaunch so_dso_place_recognition bow.launch dataset:=RobotCar img_topic:=camera/left/image_raw seq:=$s
done