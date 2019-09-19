#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "SC.h"
#include "place_recognition/src/utils/print_progress.h"
#include "place_recognition/src/utils/pts_align.h"
#include "place_recognition/src/utils/pts_preprocess.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_sc");
  ros::NodeHandle nhPriv("~");
  ros::Publisher pub = nhPriv.advertise<PointCloud>("sphere_points", 1);

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

  double lidarRange, voxelAngle;
  nhPriv.param("lidarRange", lidarRange, 45.0);
  nhPriv.param("voxelAngle", voxelAngle, 1.0);

  // process points
  std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> pts_spherical_vec;
  pts_preprocess(poses_history_file, pts_history_file, incoming_id_file,
                 lidarRange, voxelAngle, pts_spherical_vec);

  SC *sc = new SC();
  Eigen::MatrixXd historySC =
      Eigen::MatrixXd(pts_spherical_vec.size(), 2 * sc->getSignatureSize());

  float total_time = 0.0;
  for (int pts_i = 0; pts_i < pts_spherical_vec.size(); pts_i++) {
    std::vector<std::pair<Eigen::Vector3d, float>> pts_spherical_aligned;
    align_points_PCA(pts_spherical_vec[pts_i], pts_spherical_aligned);

    Eigen::VectorXd signature_structure, signature_intensity;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    sc->getSignature(pts_spherical_aligned, signature_structure,
                     signature_intensity, lidarRange);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;

    // record historySC
    Eigen::VectorXd signature(historySC.cols());
    signature << signature_structure, signature_intensity;
    historySC.row(pts_i) = signature.transpose();

    printProgress(float(pts_i) / pts_spherical_vec.size());
  }
  std::cout << std::endl
            << "SC average time: "
            << 1000.0 * total_time / pts_spherical_vec.size() << "ms"
            << std::endl;

  std::ofstream sc_file_stream;
  sc_file_stream.open(sc_file);
  sc_file_stream << historySC;
  sc_file_stream.close();

  delete sc;
}
