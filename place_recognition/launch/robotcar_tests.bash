#!/bin/bash

for d in 2014-07-14-14-49-50 2014-11-28-12-07-13 2014-12-12-10-45-15 2015-02-10-11-58-05 2015-05-19-14-06-38 2015-05-22-11-14-30 2015-08-13-16-02-58 2015-10-30-13-52-14
do
    for m in delight sc m2dp
    do
        roslaunch so_dso_place_recognition robotcar_lidar.launch date:=$d method:=$m
    done
    roslaunch so_dso_place_recognition robotcar_gist.launch date:=$d
    roslaunch so_dso_place_recognition robotcar_bow.launch date:=$d
done