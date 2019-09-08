#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "M2DP.h"
#include "loop_closure/src/utils/print_progress.h"
#include "loop_closure/src/utils/process_pts.h"

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

  double loopRange, voxelAngle;
  nhPriv.param("loopRange", loopRange, 45.0);
  nhPriv.param("voxelAngle", voxelAngle, 1.0);

  // process points
  std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> pts_sphere_vec;
  process_pts(poses_history_file, pts_history_file, incoming_id_file, loopRange,
              voxelAngle, pts_sphere_vec);

  M2DP *m2dp = new M2DP();
  Eigen::MatrixXd historyM2DP =
      Eigen::MatrixXd(4 * pts_sphere_vec.size(), 2 * m2dp->getSignatureSize());

  float total_time = 0.0;
  for (int pts_i = 0; pts_i < pts_sphere_vec.size(); pts_i++) {
    std::vector<std::vector<std::pair<Eigen::Vector3d, float>>>
        cur_pts_sphere_vec;
    m2dp->PCARotationInvariant(pts_sphere_vec[pts_i], cur_pts_sphere_vec);

    PointCloud::Ptr cloud(new PointCloud);
    for (auto &p : cur_pts_sphere_vec[0]) {
      pcl::PointXYZI p_xyzi(p.second);
      p_xyzi.x = p.first(0);
      p_xyzi.y = p.first(1);
      p_xyzi.z = p.first(2);
      cloud->points.push_back(p_xyzi);
    }
    cloud->header.frame_id = "map";
    cloud->height = 1;
    cloud->width = cur_pts_sphere_vec[0].size();
    cloud->header.stamp = pts_i;
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub.publish(cloud);

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    for (int config_i = 0; config_i < 4; config_i++) {
      Eigen::VectorXd signature_ct, signature_ci;
      m2dp->getSignature(cur_pts_sphere_vec[config_i], signature_ct,
                         signature_ci, loopRange);

      // record historyM2DP
      Eigen::VectorXd signature(historyM2DP.cols());
      signature << signature_ct, signature_ci;
      historyM2DP.row(4 * pts_i + config_i) = signature.transpose();
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
