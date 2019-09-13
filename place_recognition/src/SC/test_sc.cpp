#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "SC.h"
#include "place_recognition/src/utils/print_progress.h"
#include "place_recognition/src/utils/process_pts.h"

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

  double loopRange, voxelAngle;
  nhPriv.param("loopRange", loopRange, 45.0);
  nhPriv.param("voxelAngle", voxelAngle, 1.0);

  // process points
  std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> pts_sphere_vec;
  process_pts(poses_history_file, pts_history_file, incoming_id_file, loopRange,
              voxelAngle, pts_sphere_vec);

  SC *sc = new SC();
  Eigen::MatrixXd historySC =
      Eigen::MatrixXd(pts_sphere_vec.size(), 2 * sc->getSignatureSize());

  float total_time = 0.0;
  for (int pts_i = 0; pts_i < pts_sphere_vec.size(); pts_i++) {
    std::vector<std::pair<Eigen::Vector3d, float>> cur_pts_sphere;
    sc->PCARotationInvariant(pts_sphere_vec[pts_i], cur_pts_sphere);

    PointCloud::Ptr cloud(new PointCloud);
    for (auto &p : cur_pts_sphere) {
      pcl::PointXYZI p_xyzi(p.second);
      p_xyzi.x = p.first(0);
      p_xyzi.y = p.first(1);
      p_xyzi.z = p.first(2);
      cloud->points.push_back(p_xyzi);
    }
    cloud->header.frame_id = "map";
    cloud->height = 1;
    cloud->width = cur_pts_sphere.size();
    cloud->header.stamp = pts_i;
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub.publish(cloud);

    Eigen::VectorXd signature_ct, signature_ci;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    sc->getSignature(cur_pts_sphere, signature_ct, signature_ci, loopRange);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    float ttOpt =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();
    total_time += ttOpt;

    // record historySC
    Eigen::VectorXd signature(historySC.cols());
    signature << signature_ct, signature_ci;
    historySC.row(pts_i) = signature.transpose();

    printProgress(float(pts_i) / pts_sphere_vec.size());
  }
  std::cout << std::endl
            << "SC average time: "
            << 1000.0 * total_time / pts_sphere_vec.size() << "ms" << std::endl;

  std::ofstream sc_file_stream;
  sc_file_stream.open(sc_file);
  sc_file_stream << historySC;
  sc_file_stream.close();

  delete sc;
}
