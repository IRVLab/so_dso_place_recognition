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

void readDescFromBagByID(std::string incoming_id_file, std::string bag_file,
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
      new ORB_SLAM2::ORBextractor(2000, 1.2, 8, 20, 7);
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
  std::string voc_file, id1, id2, bag1, bag2, img_topic, output_file;
  if (!nhPriv.getParam("voc_file", voc_file) ||
      !nhPriv.getParam("incoming_id_file1", id1) ||
      !nhPriv.getParam("incoming_id_file2", id2) ||
      !nhPriv.getParam("bag1", bag1) || !nhPriv.getParam("bag2", bag2) ||
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

  std::vector<cv::Mat> desc_vec1, desc_vec2;
  readDescFromBagByID(id1, bag1, img_topic, desc_vec1);
  readDescFromBagByID(id2, bag2, img_topic, desc_vec2);

  std::vector<double> _tmp(desc_vec2.size(), 0.0);
  std::vector<std::vector<double>> scores(desc_vec1.size(), _tmp);
  int avgScore = 0;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  std::vector<DBoW2::BowVector> vbv1, vbv2;
  for (size_t j = 0; j < desc_vec1.size(); ++j) {
    std::vector<cv::Mat> desc_vec1j = toDescriptorVector(desc_vec1[j]);
    DBoW2::BowVector bv;
    DBoW2::FeatureVector fv;
    mpORBVocabulary->transform(desc_vec1j, bv, fv, 4);
    vbv1.push_back(bv);
  }
  for (size_t j = 0; j < desc_vec2.size(); ++j) {
    std::vector<cv::Mat> desc_vec2j = toDescriptorVector(desc_vec2[j]);
    DBoW2::BowVector bv;
    DBoW2::FeatureVector fv;
    mpORBVocabulary->transform(desc_vec2j, bv, fv, 4);
    vbv2.push_back(bv);
  }

  for (size_t i = 0; i < vbv1.size(); ++i) {
    std::vector<double> score;
    for (size_t j = 0; j < vbv2.size(); ++j) {
      scores[i][j] = mpORBVocabulary->score(vbv1[i], vbv2[j]);
    }
    printProgress(double(i) / desc_vec1.size());
  }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  float ttOpt =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
          .count();
  std::cout << std::endl
            << "BoW matching time: " << 1000.0 * ttOpt / desc_vec1.size()
            << "ms" << std::endl;

  std::ofstream outfile(output_file);
  for (auto ss : scores) {
    for (auto s : ss) {
      outfile << s << " ";
    }
    outfile << std::endl;
  }
  printf("Saved to %s\n", output_file.c_str());
  outfile.close();
}
