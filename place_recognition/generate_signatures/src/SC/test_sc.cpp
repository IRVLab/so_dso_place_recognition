#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "SC.h"
#include "place_recognition/generate_signatures/src/utils/print_progress.h"
#include "place_recognition/generate_signatures/src/utils/pts_preprocess.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_sc");
  ros::NodeHandle nhPriv("~");

  // get parameters
  std::string poses_history_file, pts_history_file, sc_file, incoming_id_file;
  int init_frame;
  if (!nhPriv.getParam("poses_history_file", poses_history_file) ||
      !nhPriv.getParam("pts_history_file", pts_history_file) ||
      !nhPriv.getParam("sc_file", sc_file) ||
      !nhPriv.getParam("incoming_id_file", incoming_id_file)) {
    ROS_INFO("Fail to get params, exit.");
    return 1;
  }

  double lidarRange;
  nhPriv.param("lidarRange", lidarRange, 45.0);

  // process points
  std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> pts_spherical_vec;
  pts_preprocess(poses_history_file, pts_history_file, incoming_id_file,
                 lidarRange, pts_spherical_vec, false);

  SC *sc = new SC(lidarRange);
  Eigen::MatrixXd history_sc =
      Eigen::MatrixXd(pts_spherical_vec.size(), 2 * sc->getSignatureSize());

  float total_time = 0.0;
  for (int pts_i = 0; pts_i < pts_spherical_vec.size(); pts_i++) {
    Eigen::VectorXd signature_structure, signature_intensity;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    sc->getSignature(pts_spherical_vec[pts_i], signature_structure,
                     signature_intensity);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;

    // record history_sc
    Eigen::VectorXd signature(history_sc.cols());
    signature << signature_structure, signature_intensity;
    history_sc.row(pts_i) = signature.transpose();

    printProgress(float(pts_i) / pts_spherical_vec.size());
  }
  std::cout << std::endl
            << "SC average time: "
            << 1000.0 * total_time / pts_spherical_vec.size() << "ms"
            << std::endl;

  std::ofstream sc_file_stream;
  sc_file_stream.open(sc_file);
  sc_file_stream << history_sc;
  sc_file_stream.close();

  delete sc;
}
