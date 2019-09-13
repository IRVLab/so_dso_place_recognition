parpool(5)
parfor K = 1 : 10
if K==1
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t1.xml');
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t2.xml');
end
if K==2
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t3.xml');
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t4.xml');
end
if K==3
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t5.xml');
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t6.xml');
end
if K==4
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t7.xml');
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t8.xml');
end
if K==5
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t9.xml');
    OpenSeqSLAM2('/home/jiawei/Workspace/catkin_ws/src/place_recognition/place_recognition/src/openseqslam2/.config/robotcar/t10.xml');
end
end