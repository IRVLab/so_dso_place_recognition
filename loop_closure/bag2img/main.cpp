#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <cmath>
#include <fstream>
// #include <string>
#include <boost/foreach.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_fbow");
  ros::NodeHandle nhPriv("~");
  std::string id_file, bag_file, bag2, img_topic, output_folder;
  if (!nhPriv.getParam("incoming_id_file", id_file) ||
      !nhPriv.getParam("bag", bag_file) ||
      !nhPriv.getParam("img_topic", img_topic) ||
      !nhPriv.getParam("output_folder", output_folder)) {
    ROS_INFO("Fail to get params, exit.");
    return 1;
  }

  std::ifstream infile(id_file);
  std::vector<int> incoming_id_vec;
  int iid;
  while (infile >> iid) {
    incoming_id_vec.push_back(iid);
  }
  infile.close();

  rosbag::Bag bag;
  printf("Loading bag file: %s\n", bag_file.c_str());
  bag.open(bag_file, rosbag::bagmode::Read);
  printf("Loaded!\n");

  std::vector<std::string> topics;
  topics.push_back(img_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ORB_SLAM2::ORBextractor *mpORBextractor =
      new ORB_SLAM2::ORBextractor(2000, 1.2, 8, 12, 7);
  // cv::Ptr<cv::Feature2D> fdetector=cv::ORB::create(2000);
  int id_i(0), img_i(-1);
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    img_i++;

    if (id_i >= incoming_id_vec.size())
      break;
    if (incoming_id_vec[id_i] > img_i)
      continue;

    // printf("incoming_id: %d\n", incoming_id_vec[id_i]);
    id_i++;

    sensor_msgs::Image::ConstPtr img_ros = m.instantiate<sensor_msgs::Image>();
    cv::Mat img_cv;
    try {
      img_cv = cv_bridge::toCvShare(img_ros, "mono8")->image;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::imwrite(output_folder + std::to_string(id_i) + ".png", img_cv);
  }
  bag.close();
}
