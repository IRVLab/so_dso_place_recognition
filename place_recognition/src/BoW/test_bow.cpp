#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include "DBoW2/DBoW2/BowVector.h"
#include "DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include <chrono>
#include <cmath>
#include <fstream>
// #include <string>
#include <boost/foreach.hpp>

#include "place_recognition/src/utils/print_progress.h"

std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors) {
  std::vector<cv::Mat> vDesc;
  vDesc.reserve(Descriptors.rows);
  for (int j = 0; j < Descriptors.rows; j++)
    vDesc.push_back(Descriptors.row(j));

  return vDesc;
}

void extractDescFromBagByID(std::string incoming_id_file, std::string bag_file,
                            std::string img_topic,
                            std::vector<cv::Mat> &descriptors_vec) {
  std::ifstream infile(incoming_id_file);
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
      new ORB_SLAM2::ORBextractor(4000, 1.2, 8, 20, 7);
  int id_i(0), img_i(-1);
  float total_time = 0.0;
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

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    (*mpORBextractor)(img_cv, cv::Mat(), keypoints, descriptors);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;

    cv::drawKeypoints(img_cv, keypoints, img_cv);
    cv::imshow(img_topic, img_cv);
    cv::waitKey(1);

    descriptors_vec.push_back(descriptors);
  }
  std::cout << std::endl
            << "BoW extraction time: "
            << 1000.0 * total_time / incoming_id_vec.size() << "ms"
            << std::endl;
  bag.close();
  cv::destroyAllWindows();
  delete mpORBextractor;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_bow");
  ros::NodeHandle nhPriv("~");
  std::string voc_file, id, bag, img_topic, output_file;
  if (!nhPriv.getParam("voc_file", voc_file) ||
      !nhPriv.getParam("incoming_id_file", id) ||
      !nhPriv.getParam("bag", bag) ||
      !nhPriv.getParam("img_topic", img_topic) ||
      !nhPriv.getParam("output_file", output_file)) {
    ROS_INFO("Fail to get params, exit.");
    return 1;
  }

  // Load ORB Vocabulary
  std::cout << std::endl
            << "Loading ORB Vocabulary from " << voc_file
            << ". This could take a while..." << std::endl;

  ORB_SLAM2::ORBVocabulary *mpORBVocabulary = new ORB_SLAM2::ORBVocabulary();
  mpORBVocabulary->loadFromTextFile(voc_file);
  std::cout << "Vocabulary loaded!" << std::endl << std::endl;

  // Extract descriptors for each frame by ID
  std::vector<cv::Mat> desc_vec;
  extractDescFromBagByID(id, bag, img_topic, desc_vec);

  // Transform to BoW vectors
  std::vector<DBoW2::BowVector> vbv;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  for (size_t j = 0; j < desc_vec.size(); ++j) {
    std::vector<cv::Mat> desc_vecj = toDescriptorVector(desc_vec[j]);
    DBoW2::BowVector bv;
    DBoW2::FeatureVector fv;
    mpORBVocabulary->transform(desc_vecj, bv, fv, 4);
    vbv.push_back(bv);
  }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  float ttOpt =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
          .count();
  std::cout << std::endl
            << "BoW vector transform time: " << 1000.0 * ttOpt / desc_vec.size()
            << "ms" << std::endl;

  std::ofstream outfile(output_file);
  for (auto bv : vbv) {
    DBoW2::BowVector::const_iterator bit;
    for (bit = bv.begin(); bit != bv.end(); ++bit) {
      outfile << bit->first << " ";
    }
    for (size_t i = bv.size(); i < 4000; i++) {
      outfile << "-1 ";
    }
    outfile << std::endl;
    for (bit = bv.begin(); bit != bv.end(); ++bit) {
      outfile << bit->second << " ";
    }
    for (size_t i = bv.size(); i < 4000; i++) {
      outfile << "-1 ";
    }
    outfile << std::endl;
  }
  printf("Saved to %s\n", output_file.c_str());
  outfile.close();
}
