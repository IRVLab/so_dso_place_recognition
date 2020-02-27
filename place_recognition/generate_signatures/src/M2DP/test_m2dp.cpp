#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "M2DP.h"
#include "place_recognition/generate_signatures/src/utils/print_progress.h"
#include "place_recognition/generate_signatures/src/utils/pts_align.h"
#include "place_recognition/generate_signatures/src/utils/pts_preprocess.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_m2dp");
  ros::NodeHandle nhPriv("~");

  // get parameters
  std::string poses_history_file, pts_history_file, m2dp_file, incoming_id_file;
  int init_frame;
  if (!nhPriv.getParam("poses_history_file", poses_history_file) ||
      !nhPriv.getParam("pts_history_file", pts_history_file) ||
      !nhPriv.getParam("m2dp_file", m2dp_file) ||
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

  M2DP *m2dp = new M2DP(lidarRange);
  Eigen::MatrixXd history_m2dp = Eigen::MatrixXd(4 * pts_spherical_vec.size(),
                                                 2 * m2dp->getSignatureSize());

  float total_time = 0.0;
  for (int pts_i = 0; pts_i < pts_spherical_vec.size(); pts_i++) {
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    std::vector<std::pair<Eigen::Vector3d, float>> pts_spherical_aligned;
    align_points_PCA(pts_spherical_vec[pts_i], pts_spherical_aligned);
    int history_subrow = 0;
    for (int direction_x = -1; direction_x < 2; direction_x += 2) {
      for (int direction_y = -1; direction_y < 2; direction_y += 2) {
        // point directions
        std::vector<std::pair<Eigen::Vector3d, float>>
            pts_spherical_aligned_direction;
        for (auto &pc : pts_spherical_aligned) {
          Eigen::Vector3d pts;
          pts << direction_x * pc.first[0], direction_y * pc.first[1],
              (direction_x * direction_y) * pc.first[2];
          pts_spherical_aligned_direction.push_back({pts, pc.second});
        }

        Eigen::VectorXd signature_ct, signature_ci;
        m2dp->getSignature(pts_spherical_aligned_direction, signature_ct,
                           signature_ci);

        // record history_m2dp
        Eigen::VectorXd signature(history_m2dp.cols());
        signature << signature_ct, signature_ci;
        history_m2dp.row(4 * pts_i + history_subrow++) = signature.transpose();
      }
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;

    printProgress(float(pts_i) / pts_spherical_vec.size());
  }
  std::cout << std::endl
            << "M2DP average time: "
            << 1000.0 * total_time / pts_spherical_vec.size() << "ms"
            << std::endl;

  std::ofstream m2dp_file_stream;
  m2dp_file_stream.open(m2dp_file);
  m2dp_file_stream << history_m2dp;
  m2dp_file_stream.close();

  delete m2dp;
}
