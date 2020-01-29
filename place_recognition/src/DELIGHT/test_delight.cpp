#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "DELIGHT.h"
#include "place_recognition/src/utils/print_progress.h"
#include "place_recognition/src/utils/pts_preprocess.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_delight");
  ros::NodeHandle nhPriv("~");
  ros::Publisher pub = nhPriv.advertise<PointCloud>("sphere_points", 1);

  // get parameters
  std::string poses_history_file, pts_history_file, delight_file,
      incoming_id_file;
  int init_frame;
  if (!nhPriv.getParam("poses_history_file", poses_history_file) ||
      !nhPriv.getParam("pts_history_file", pts_history_file) ||
      !nhPriv.getParam("delight_file", delight_file) ||
      !nhPriv.getParam("incoming_id_file", incoming_id_file)) {
    ROS_INFO("Fail to get params, exit.");
    return 1;
  }

  double lidarRange;
  nhPriv.param("lidarRange", lidarRange, 45.0);

  // process points
  std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> pts_spherical_vec;
  pts_preprocess(poses_history_file, pts_history_file, incoming_id_file,
                 lidarRange, pts_spherical_vec, true);

  DELIGHT *delight = new DELIGHT();
  Eigen::MatrixXd history_delight = Eigen::MatrixXd(
      16 * pts_spherical_vec.size(), delight->getSignatureSize());

  float total_time = 0.0;
  for (int pts_i = 0; pts_i < pts_spherical_vec.size(); pts_i++) {
    Eigen::MatrixXd signature;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    delight->getSignature(pts_spherical_vec[pts_i], signature);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;

    // record history_delight
    history_delight.block(16 * pts_i, 0, 16, history_delight.cols()) =
        signature;

    printProgress(float(pts_i) / pts_spherical_vec.size());
  }
  std::cout << std::endl
            << "DELIGHT average time: "
            << 1000.0 * total_time / pts_spherical_vec.size() << "ms"
            << std::endl;

  std::ofstream delight_file_stream;
  delight_file_stream.open(delight_file);
  delight_file_stream << history_delight;
  delight_file_stream.close();

  delete delight;
}
