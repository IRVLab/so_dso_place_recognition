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

#include "../include/gist.h"
#include "../include/standalone_image.h"

#include <cmath>
#include <fstream>
// #include <string>
#include "loop_closure/src/utils/print_progress.h"
#include <boost/foreach.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_gist");
  ros::NodeHandle nhPriv("~");
  std::string incoming_id_file, bag_file, img_topic, gist_file;
  if (!nhPriv.getParam("incoming_id_file", incoming_id_file) ||
      !nhPriv.getParam("bag", bag_file) ||
      !nhPriv.getParam("img_topic", img_topic) ||
      !nhPriv.getParam("gist_file", gist_file)) {
    ROS_INFO("Fail to get params, exit.");
    return 1;
  }

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

  int id_i(0), img_i(-1);

  const cls::GISTParams DEFAULT_PARAMS{false, 256, 256, 4, 4, {8, 8, 8, 8}};
  cls::GIST gist_extractor(DEFAULT_PARAMS);
  std::ofstream outfile(gist_file);
  float total_time = 0.0;
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
    cv::waitKey(1);

    std::vector<float> result;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    gist_extractor.extract(img_cv, result);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;
    // std::cout << "GIST time: " << ttOpt << std::endl;

    for (const auto &val : result) {
      outfile << val << " ";
    }
    outfile << std::endl;
  }
  std::cout << std::endl
            << "GIST average time: "
            << 1000.0 * total_time / incoming_id_vec.size() << "ms"
            << std::endl;
  bag.close();
  outfile.close();
  cv::destroyAllWindows();
}
