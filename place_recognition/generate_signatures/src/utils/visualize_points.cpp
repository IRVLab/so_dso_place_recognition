#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>

#include <cmath>
#include <fstream>
// #include <string>
#include "place_recognition/generate_signatures/src/utils/pts_preprocess.h"
#include <boost/foreach.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr
create_point_clouds(const std::vector<std::pair<Eigen::Vector3d, float>> &pts,
                    const std::vector<int> &rgb) {

  pcl::PointCloud<pcl::PointXYZRGB> pc;
  pc.width = pts.size();
  pc.height = 1;
  pc.is_dense = false;
  pc.points.resize(pts.size());

  for (size_t i = 0; i < pts.size(); i++) {
    pc.points[i].x = pts[i].first[0];
    pc.points[i].y = pts[i].first[2];
    pc.points[i].z = -pts[i].first[1];
    pc.points[i].r = rgb[0];
    pc.points[i].g = rgb[1];
    pc.points[i].b = rgb[2];
  }
  
  auto pc_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pc);

  return pc_ptr;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualize_points");
  ros::NodeHandle nhPriv("~");
  std::string poses_history_file, pts_history_file, bag_file, img_topic, incoming_id_file;
  if (!nhPriv.getParam("poses_history_file", poses_history_file) ||
      !nhPriv.getParam("pts_history_file", pts_history_file) ||
      !nhPriv.getParam("bag", bag_file) ||
      !nhPriv.getParam("img_topic", img_topic) ||
      !nhPriv.getParam("incoming_id_file", incoming_id_file)) {
    ROS_INFO("Fail to get params, exit.");
    return 1;
  }

  ros::Publisher pc_pub = nhPriv.advertise<PointCloud>("sphere_points", 1);
  pcl::PLYWriter *plyWriter = new pcl::PLYWriter();

  double lidarRange;
  nhPriv.param("lidarRange", lidarRange, 45.0);

  // process points
  std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> pts_spherical_vec;
  pts_preprocess(poses_history_file, pts_history_file, incoming_id_file,
                 lidarRange, pts_spherical_vec, true);
  std::vector<int> incoming_id_vec;
  if (incoming_id_file != "") {
    std::ifstream infile(incoming_id_file);
    int iid;
    while (infile >> iid) {
      incoming_id_vec.push_back(iid);
    }
    infile.close();
  }

  rosbag::Bag bag;
  printf("Loading bag file: %s\n", bag_file.c_str());
  bag.open(bag_file, rosbag::bagmode::Read);
  printf("Loaded!\n");

  std::vector<std::string> topics;
  topics.push_back(img_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  int id_i(0), img_i(-1), pts_i(0);

  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    if (incoming_id_file != "") {
      img_i++;
      if (id_i >= incoming_id_vec.size())
        break;
      if (incoming_id_vec[id_i] > img_i)
        continue;
      // printf("incoming_id: %d\n", incoming_id_vec[id_i]);
      id_i++;
    }

    sensor_msgs::Image::ConstPtr img_ros = m.instantiate<sensor_msgs::Image>();
    cv::Mat img_cv;
    try {
      img_cv = cv_bridge::toCvShare(img_ros, "mono8")->image;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::imshow(img_topic, img_cv);
    cv::waitKey(100);

    auto pc_ptr = create_point_clouds(pts_spherical_vec[pts_i++], {0, 255, 0});
    pc_ptr->header.frame_id = "map";
    pcl_conversions::toPCL(ros::Time::now(), pc_ptr->header.stamp);
    pc_pub.publish(pc_ptr);

    plyWriter->write("points.ply", *pc_ptr);
  }
  bag.close();
  cv::destroyAllWindows();
}
