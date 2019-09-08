#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include "ORBextractor.h"

#include <cmath>
#include <fstream>
// #include <string>
#include <boost/foreach.hpp>

#include "loop_closure/src/utils/print_progress.h"
#include <fbow/vocabulary_creator.h>

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
      new ORB_SLAM2::ORBextractor(2000, 1.2, 8, 12, 7);
  // cv::Ptr<cv::Feature2D> fdetector=cv::ORB::create(2000);
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

    // cv::GaussianBlur( img_cv, img_cv, cv::Size( 5, 5 ), 0, 0);
    std::vector<cv::KeyPoint> keypoints;

    /*{
      int nr(5), nc(5);
      int sr(ceil(float(img_cv.rows)/nr)), sc(ceil(float(img_cv.cols)/nc));
      for(int r=0; r<(nr-1); r++)
      {
        for(int c=0; c<(nc-1); c++)
        {
          cv::Mat tmp = img_cv(cv::Rect(c*sc, r*sr, sc, sr)).clone();
          // cv::imshow("tmp", tmp);
          // cv::waitKey();
          std::vector<cv::Point2f> corners;
          cv::goodFeaturesToTrack(img_cv(cv::Rect(c*sc, r*sr, sc, sr)), corners,
    5, 0.1, 10); for(auto& p:corners) keypoints.push_back(cv::KeyPoint(p.x+c*sc,
    p.y+r*sr, 1.f));
          // std::vector<cv::KeyPoint> kps;
          // fdetector->detect(img_cv(cv::Rect(c*sc, r*sr, sc, sr)), kps);
          // for(auto& kp:kps) keypoints.push_back(cv::KeyPoint(kp.pt.x+c*sc,
    kp.pt.y+r*sr, 1.f));
        }
      }
      for(int r=0; r<(nr-1); r++)
      {
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(img_cv(cv::Rect((nc-1)*sc, r*sr,
    img_cv.cols-(nc-1)*sc, sr)), corners, 5, 0.1, 10); for(auto& p:corners)
    keypoints.push_back(cv::KeyPoint(p.x+(nc-1)*sc, p.y+r*sr, 1.f));
        // std::vector<cv::KeyPoint> kps;
        // fdetector->detect(img_cv(cv::Rect((nc-1)*sc, r*sr,
    img_cv.cols-(nc-1)*sc, sr)), kps);
        // for(auto& kp:kps) keypoints.push_back(cv::KeyPoint(kp.pt.x+(nc-1)*sc,
    kp.pt.y+r*sr, 1.f));
      }
      for(int c=0; c<(nc-1); c++)
      {
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(img_cv(cv::Rect(c*sc, (nr-1)*sr, sc,
    img_cv.rows-(nr-1)*sr)), corners, 5, 0.1, 10); for(auto& p:corners)
    keypoints.push_back(cv::KeyPoint(p.x+c*sc, p.y+(nr-1)*sr, 1.f));
        // std::vector<cv::KeyPoint> kps;
        // fdetector->detect(img_cv(cv::Rect(c*sc, (nr-1)*sr, sc,
    img_cv.rows-(nr-1)*sr)), kps);
        // for(auto& kp:kps) keypoints.push_back(cv::KeyPoint(kp.pt.x+c*sc,
    kp.pt.y+(nr-1)*sr, 1.f));
      }
      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(img_cv(cv::Rect((nc-1)*sc, (nr-1)*sr,
    img_cv.cols-(nc-1)*sc, img_cv.rows-(nr-1)*sr)), corners, 5, 0.1, 10);
      for(auto& p:corners) keypoints.push_back(cv::KeyPoint(p.x+(nc-1)*sc,
    p.y+(nr-1)*sr, 1.f));
      // std::vector<cv::KeyPoint> kps;
      // fdetector->detect(img_cv(cv::Rect((nc-1)*sc, (nr-1)*sr,
    img_cv.cols-(nc-1)*sc, img_cv.rows-(nr-1)*sr)), kps);
      // for(auto& kp:kps) keypoints.push_back(cv::KeyPoint(kp.pt.x+(nc-1)*sc,
    kp.pt.y+(nr-1)*sr, 1.f));
    }*/

    // printf("keypoints %d\n", keypoints.size());
    cv::Mat descriptors;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    (*mpORBextractor)(img_cv, cv::Mat(), keypoints, descriptors);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;
    // fdetector->detectAndCompute(img_cv, cv::Mat(), keypoints, descriptors);
    // std::vector<cv::Point2f> corners;
    // cv::goodFeaturesToTrack(img_cv, corners, 800, 0.01, 1);
    // for( size_t i = 0; i < corners.size(); i++ ) {
    //     keypoints.push_back(cv::KeyPoint(corners[i], 1.f));
    // }
    // fdetector->compute(img_cv,keypoints, descriptors);

    cv::drawKeypoints(img_cv, keypoints, img_cv);
    cv::imshow(img_topic, img_cv);
    cv::waitKey(1);

    descriptors_vec.push_back(descriptors);
  }
  std::cout << std::endl
            << "FBoW average time: "
            << 1000.0 * total_time / incoming_id_vec.size() << "ms"
            << std::endl;
  bag.close();
  cv::destroyAllWindows();
  delete mpORBextractor;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_fbow");
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

  std::vector<cv::Mat> desc_vec1, desc_vec2;
  readDescFromBagByID(id1, bag1, img_topic, desc_vec1);
  readDescFromBagByID(id2, bag2, img_topic, desc_vec2);

  // Load ORB Vocabulary
  std::cout << std::endl
            << "Loading ORB Vocabulary. This could take a while..."
            << std::endl;

  fbow::Vocabulary voc;
  voc.readFromFile(voc_file);
  std::cout << "Vocabulary loaded!" << std::endl << std::endl;

  std::vector<double> _tmp(desc_vec2.size(), 0.0);
  std::vector<std::vector<double>> scores(desc_vec1.size(), _tmp);
  std::vector<fbow::fBow> vv1, vv2;
  int avgScore = 0;
  auto t_start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < desc_vec1.size(); ++i) {
    vv1.push_back(voc.transform(desc_vec1[i]));
  }
  for (size_t j = 0; j < desc_vec2.size(); ++j) {
    vv2.push_back(voc.transform(desc_vec2[j]));
  }
#pragma omp parallel for simd
  for (size_t i = 0; i < desc_vec1.size(); ++i) {
    std::vector<double> score;
    for (size_t j = 0; j < desc_vec2.size(); ++j) {
      scores[i][j] = vv1[i].score(vv1[i], vv2[j]);
    }
    printProgress(double(i) / desc_vec1.size());
  }

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
