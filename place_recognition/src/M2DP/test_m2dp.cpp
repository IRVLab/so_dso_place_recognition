#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "M2DP.h"
#include "place_recognition/src/utils/print_progress.h"
#include "place_recognition/src/utils/pts_align.h"
#include "place_recognition/src/utils/pts_preprocess.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_m2dp");
  ros::NodeHandle nhPriv("~");
  ros::Publisher pub = nhPriv.advertise<PointCloud>("sphere_points", 1);

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

  double lidarRange, voxelAngle;
  nhPriv.param("lidarRange", lidarRange, 45.0);
  nhPriv.param("voxelAngle", voxelAngle, 1.0);

  // process points
  std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> pts_sphere_vec;
  pts_preprocess(poses_history_file, pts_history_file, incoming_id_file,
                 lidarRange, voxelAngle, pts_sphere_vec);

  M2DP *m2dp = new M2DP();
  Eigen::MatrixXd historyM2DP =
      Eigen::MatrixXd(4 * pts_sphere_vec.size(), 2 * m2dp->getSignatureSize());

  float total_time = 0.0;
  for (int pts_i = 0; pts_i < pts_sphere_vec.size(); pts_i++) {
    std::vector<std::pair<Eigen::Vector3d, float>> cur_pts_sphere;
    align_points_PCA(pts_sphere_vec[pts_i], cur_pts_sphere);

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    int history_subrow = 0;
    for (int direction_x = -1; direction_x < 2; direction_x += 2) {
      for (int direction_y = -1; direction_y < 2; direction_y += 2) {
        // point directions
        std::vector<std::pair<Eigen::Vector3d, float>> cur_pts_sphere_direction;
        for (auto &pc : cur_pts_sphere) {
          Eigen::Vector3d pts;
          pts << direction_x * pc.first[0], direction_y * pc.first[1],
              (direction_x * direction_y) * pc.first[2];
          cur_pts_sphere_direction.push_back({pts, pc.second});
        }

        Eigen::VectorXd signature_ct, signature_ci;
        m2dp->getSignature(cur_pts_sphere_direction, signature_ct, signature_ci,
                           lidarRange);

        // record historyM2DP
        Eigen::VectorXd signature(historyM2DP.cols());
        signature << signature_ct, signature_ci;
        historyM2DP.row(4 * pts_i + history_subrow++) = signature.transpose();
      }
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;

    printProgress(float(pts_i) / pts_sphere_vec.size());
  }
  std::cout << std::endl
            << "M2DP average time: "
            << 1000.0 * total_time / pts_sphere_vec.size() << "ms" << std::endl;

  std::ofstream m2dp_file_stream;
  m2dp_file_stream.open(m2dp_file);
  m2dp_file_stream << historyM2DP;
  m2dp_file_stream.close();

  delete m2dp;
}
